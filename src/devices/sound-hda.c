/*
sound-hda.c - HD Audio
Copyright (C) 2025  David Korenchuk <github.com/epoll-reactor-2>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "sound-hda.h"
#include "atomics.h"
#include "compiler.h"
#include "mem_ops.h"
#include "rvtimer.h"
#include "threading.h"
#include "spinlock.h"
#include "utils.h"

PUSH_OPTIMIZATION_SIZE

#define SOUND_VENDOR_ID_CMEDIA        0x13f6 // C-Media
#define SOUND_DEVICE_ID_CMEDIA        0x5011 // CM8888 HDA Controller
#define SOUND_CLASS_CODE_CMEDIA       0x0403 // Audio device

#define SOUND_HDA_FIFO_SIZE           0x100

#define SOUND_HDA_GCAP                0x00 // Global capabilities
#define SOUND_HDA_VS                  0x02 // 0x02 minor, 0x03 major
#define SOUND_HDA_OUTPAY              0x04 // Output payload capability
#define SOUND_HDA_INPAY               0x06 // Input payload capability
#define SOUND_HDA_GLOBAL_CTRL         0x08 // Global Control
#define SOUND_HDA_WAKEEN              0x0C // Wake enable
#define SOUND_HDA_STATESTS            0x0E // State Change Status
#define SOUND_HDA_GTS                 0x10 // Global status
#define SOUND_HDA_OUTSTRMPAY          0x18 // Output stream payload capability
#define SOUND_HDA_INSTRMPAY           0x1A // Input stream payload capability
#define SOUND_HDA_INTR_CTRL           0x20 // Interrupt Control
#define SOUND_HDA_INTSTS              0x24 // Interrupt status
#define SOUND_HDA_WALL_CLOCK          0x30 // Wall clock counter
#define SOUND_HDA_STREAM_SYNC         0x38 // Stream Synchronization
#define SOUND_HDA_CORB_LO             0x40 // CORB Lower Base Address
#define SOUND_HDA_CORB_HI             0x44 // CORB Upper Base Address
#define SOUND_HDA_CORB_WP             0x48 // CORB Write Pointer
#define SOUND_HDA_CORB_RP             0x4A // CORB Read Pointer
#define SOUND_HDA_CORB_CTRL           0x4C // CORB Control
#define SOUND_HDA_CORB_STATUS         0x4D // CORB Status
#define SOUND_HDA_CORB_SIZE           0x4E // CORB Size
#define SOUND_HDA_RIRB_LO             0x50 // RIRB Lower Base Address
#define SOUND_HDA_RIRB_HI             0x54 // RIRB Upper Base Address
#define SOUND_HDA_RIRB_WP             0x58 // RIRB Write Pointer
#define SOUND_HDA_RIRB_INTR_CNT       0x5A // RIRB Response Interrupt Count
#define SOUND_HDA_RIRB_CTRL           0x5C // RIRB Control
#define SOUND_HDA_RIRB_STATUS         0x5D // RIRB Status
#define SOUND_HDA_RIRB_SIZE           0x5E // RIRB Size
#define SOUND_HDA_DMA_LO              0x70 // DMA Position Lower Base Address
#define SOUND_HDA_DMA_HI              0x74 // DMA Position Upper Base Address

// Stream descriptor blocks (HDA spec §3.3.34, page 27): identical 0x20-byte
// register layout for every input, output, and bidirectional descriptor.
// Per-register offsets aren't enumerated here — the SD register table near
// the end of the typedef block is the single source of truth, and the
// MMIO dispatch resolves (offset → stream index, sub-offset → register)
// automatically.
//
// Today: NO_IN=1 → ISD0 at 0x80; NO_OUT=1 → OSD0 at 0xA0; NO_BSS=0.

#define SOUND_HDA_PARAM_V             0x103 // Version 1.03
#define SOUND_HDA_PARAM_NO_OUT        0x01  // Number of output streams supported
#define SOUND_HDA_PARAM_NO_IN         0x01  // Number of input streams supported
#define SOUND_HDA_PARAM_NO_BSS        0x00  // Number of bidirectional streams supported
#define SOUND_HDA_PARAM_NO_NSDO       0x00  // Number of serial data out signals
#define SOUND_HDA_PARAM_64_BIT        0x00  // 64-bit support

#define SOUND_HDA_PARAM_GCAP          ((SOUND_HDA_PARAM_NO_OUT  & 15) << 12) \
                                    | ((SOUND_HDA_PARAM_NO_IN   & 15) <<  8) \
                                    | ((SOUND_HDA_PARAM_NO_BSS  & 31) <<  3) \
                                    | ((SOUND_HDA_PARAM_NO_NSDO &  2) <<  1) \
                                    | ((SOUND_HDA_PARAM_64_BIT  &  1))

#define SOUND_HDA_PARAM_CORBSZCAP     1 /*   8 bytes =  2 entries */ \
                                    | 2 /*  64 bytes = 16 entries */ \
                                    | 4 /* 256 bytes = 32 entries */ \
                                    | 8

#define SOUND_HDA_PARAM_RIRBSZCAP     1 /*   16 bytes =   2 entries */ \
                                    | 2 /*  128 bytes =  16 entries */ \
                                    | 4 /* 2048 bytes = 256 entries */ \
                                    | 8

#define SOUND_HDA_PARAM_CORBSIZE      2 /*  256 bytes =  32 entries */
#define SOUND_HDA_PARAM_RIRBSIZE      2 /* 2048 bytes = 256 entries */

// Parameters are described in 7.3.4 Parameters
#define VERB_GET_PARAMETER                               0xF00
#define VERB_GET_CONN_SELECT_CONTROL                     0xF01
#define VERB_SET_CONN_SELECT_CONTROL                     0x701
#define VERB_GET_CONN_LIST_ENTRY                         0xF02
#define VERB_GET_PROCESSING_STATE                        0xF03
#define VERB_SET_PROCESSING_STATE                        0x703
#define VERB_GET_COEFF_INDEX                             0xD
#define VERB_SET_COEFF_INDEX                             0x5
#define VERB_GET_PROCESSING_COEFF                        0xC
#define VERB_SET_PROCESSING_COEFF                        0x4

#define VERB_GET_AMP_GAIN_MUTE                           0xB
#define VERB_GET_AMP_GAIN_MUTE_INPUT                     0x0000
#define VERB_GET_AMP_GAIN_MUTE_OUTPUT                    0x8000
#define VERB_GET_AMP_GAIN_MUTE_RIGHT                     0x0000
#define VERB_GET_AMP_GAIN_MUTE_LEFT                      0x2000

#define VERB_SET_AMP_GAIN_MUTE                           0x3
#define VERB_SET_AMP_GAIN_MUTE_MUTE                      0x80   // Payload bit 7
#define VERB_SET_AMP_GAIN_MUTE_GAIN_MASK                 0x7F   // Payload bits 6:0
#define VERB_SET_AMP_GAIN_MUTE_OUTPUT                    0x8000 // Bit 15: Set Output Amp
#define VERB_SET_AMP_GAIN_MUTE_INPUT                     0x4000 // Bit 14: Set Input Amp
#define VERB_SET_AMP_GAIN_MUTE_LEFT                      0x2000 // Bit 13: Set Left  (channel 0)
#define VERB_SET_AMP_GAIN_MUTE_RIGHT                     0x1000 // Bit 12: Set Right (channel 1)

#define VERB_GET_CONV_FMT                                0xA
#define VERB_SET_CONV_FMT                                0x2
#define VERB_GET_DIGITAL_CONV_FMT1                       0xF0D
#define VERB_GET_DIGITAL_CONV_FMT2                       0xF0E
#define VERB_SET_DIGITAL_CONV_FMT1                       0x70D
#define VERB_SET_DIGITAL_CONV_FMT2                       0x70E
#define VERB_GET_POWER_STATE                             0xF05
#define VERB_SET_POWER_STATE                             0x705
#define VERB_GET_CONV_STREAM_CHAN                        0xF06
#define VERB_SET_CONV_STREAM_CHAN                        0x706
#define VERB_GET_INPUT_CONVERTER_SDI_SELECT              0xF04
#define VERB_SET_INPUT_CONVERTER_SDI_SELECT              0x704

#define VERB_GET_PIN_WIDGET_CTRL                         0xF07
#define VERB_GET_PIN_WIDGET_CTRL_HPHN_ENABLE             (1 << 7)
#define VERB_GET_PIN_WIDGET_CTRL_OUT_ENABLE              (1 << 6)
#define VERB_GET_PIN_WIDGET_CTRL_IN_ENABLE               (1 << 5)
#define VERB_GET_PIN_WIDGET_CTRL_VREF_ENABLE             (1 << 0)

#define VERB_SET_PIN_WIDGET_CTRL                         0x707
#define VERB_GET_UNSOLICITED_RESPONSE                    0xF08
#define VERB_SET_UNSOLICITED_RESPONSE                    0x708

#define VERB_GET_PIN_SENSE                               0xF09
#define VERB_GET_PIN_SENSE_PRESENSE_PLUGGED              (1 << 31)

#define VERB_SET_PIN_SENSE                               0x709
#define VERB_GET_EAPD_BTL_ENABLE                         0xF0C
#define VERB_SET_EAPD_BTL_ENABLE                         0x70C
#define VERB_GET_GPI_DATA                                0xF10
#define VERB_SET_GPI_DATA                                0x710
#define VERB_GET_GPI_WAKE_ENABLE_MASK                    0xF11
#define VERB_SET_GPI_WAKE_ENABLE_MASK                    0x711
#define VERB_GET_GPI_UNSOLICITED_ENABLE_MASK             0xF12
#define VERB_SET_GPI_UNSOLICITED_ENABLE_MASK             0x712
#define VERB_GET_GPI_STICKY_MASK                         0xF13
#define VERB_SET_GPI_STICKY_MASK                         0x713
#define VERB_GET_GPO_DATA                                0xF14
#define VERB_SET_GPO_DATA                                0x714
#define VERB_GET_GPIO_DATA                               0xF15
#define VERB_SET_GPIO_DATA                               0x715
#define VERB_GET_GPIO_ENABLE_MASK                        0xF16
#define VERB_SET_GPIO_ENABLE_MASK                        0x716
#define VERB_GET_GPIO_DIRECTION                          0xF17
#define VERB_SET_GPIO_DIRECTION                          0x717
#define VERB_GET_GPIO_WAKE_ENABLE_MASK                   0xF18
#define VERB_SET_GPIO_WAKE_ENABLE_MASK                   0x718
#define VERB_GET_GPIO_UNSOLICITED_ENABLE_MASK            0xF19
#define VERB_SET_GPIO_UNSOLICITED_ENABLE_MASK            0x719
#define VERB_GET_GPIO_STICKY_MASK                        0xF1A
#define VERB_SET_GPIO_STICKY_MASK                        0x71A
#define VERB_GET_BEEP_GENERATION                         0xF0A
#define VERB_SET_BEEP_GENERATION                         0x70A
#define VERB_GET_VOLUME_KNOB                             0xF0F
#define VERB_SET_VOLUME_KNOB                             0x70F
#define VERB_GET_SUBSYSTEM_ID                            0xF20
#define VERB_SET_SUSBYSTEM_ID1                           0x720
#define VERB_SET_SUBSYSTEM_ID2                           0x721
#define VERB_SET_SUBSYSTEM_ID3                           0x722
#define VERB_SET_SUBSYSTEM_ID4                           0x723

#define VERB_GET_CONFIG_DEFAULT                          0xF1C
#define VERB_GET_CONFIG_DEFAULT_ASSOCIATION_DEFAULT      ( 1 <<  4)
#define VERB_GET_CONFIG_DEFAULT_COLOR_UNKNOWN            ( 0 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_BLACK              ( 1 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_GREY               ( 2 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_BLUE               ( 3 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_GREEN              ( 4 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_RED                ( 5 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_ORANGE             ( 6 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_YELLOW             ( 7 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_PURPLE             ( 8 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_PINK               ( 9 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_WHITE              (14 << 12)
#define VERB_GET_CONFIG_DEFAULT_COLOR_OTHER              (15 << 12)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_LINE_OUT          ( 0 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_SPEAKER           ( 1 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_HP_OUT            ( 2 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_CD                ( 3 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_SPDIF_OUT         ( 4 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_DIGITAL_OTHER_OUT ( 5 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_MODEM_LINE        ( 6 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_MODEM_HANDSET     ( 7 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_LIVE_IN           ( 8 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_AUX               ( 9 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_MIC_IN            (10 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_TELEPHONY         (11 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_SPDIF_IN          (12 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_DIGITAL_OTHER_IN  (13 << 20)
#define VERB_GET_CONFIG_DEFAULT_DEVICE_OTHER             (15 << 20)
#define VERB_GET_CONFIG_DEFAULT_CONNECTIVITY_JACK        ( 0 << 30)
#define VERB_GET_CONFIG_DEFAULT_CONNECTIVITY_NONE        ( 1 << 30)
#define VERB_GET_CONFIG_DEFAULT_CONNECTIVITY_FIXED       ( 2 << 30)
#define VERB_GET_CONFIG_DEFAULT_CONNECTIVITY_BOTH        ( 3 << 30)

#define VERB_SET_CONFIG_DEFAULT1                         0x71C
#define VERB_SET_CONFIG_DEFAULT2                         0x71D
#define VERB_SET_CONFIG_DEFAULT3                         0x71E
#define VERB_SET_CONFIG_DEFAULT4                         0x71F
#define VERB_GET_STRIPE_CONTROL                          0xF24
#define VERB_SET_STRIPE_CONTROL                          0x724
#define VERB_GET_CONV_CHAN_COUNT                         0xF2D
#define VERB_SET_CONV_CHAN_COUNT                         0x72D
#define VERB_FUNCTION_RESET                              0x7FF

// Applicable for VERB_GET_PARAMETER. We should assign static
// values to these parameter to emulate codec.
#define CODEC_PARAM_VENDOR_ID                            0x00
#define CODEC_PARAM_REVISION_ID                          0x02
#define CODEC_PARAM_SUB_NODE_COUNT                       0x04

#define CODEC_PARAM_FUNC_GROUP_TYPE                      0x05
#define CODEC_PARAM_FUNC_GROUP_TYPE_AUDIO                0x01
#define CODEC_PARAM_FUNC_GROUP_TYPE_MODEM                0x02

#define CODEC_PARAM_AUDIO_FUNC_GROUP_TYPE                0x08

#define CODEC_PARAM_AUDIO_WIDGET_CAPS                    0x09
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_STEREO             (1 <<  0)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_AMP_IN             (1 <<  1)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_AMP_OUT            (1 <<  2)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_AMP_OVR            (1 <<  3)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_FORMAT_OVR         (1 <<  4)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_CONN_LIST          (1 <<  8)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_OUTPUT             (0 << 20)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_INPUT              (1 << 20)
#define CODEC_PARAM_AUDIO_WIDGET_CAPS_PIN                (4 << 20)

#define CODEC_PARAM_SUPP_PCM_SIZE_RATES                  0x0A

// HDA stream rates. Single source of truth for both the
// CODEC_PARAM_SUPP_PCM_SIZE_RATES advertisement bitmask (parameter
// 0x0A, bits 0..10) and the stream format register encoding (HDA
// spec 7.3.3.10: base / multiplier / divisor in SDnFMT bits 14, 13:11,
// 10:8). Mirrors the autodetected portion of Linux's rate_bits[] in
// sound/hda/hdac_device.c. 384 kHz (bit 11) is reserved on real HW
// and intentionally omitted.
//
// X(hz, bit_pos, base_khz, mult, div):
//   hz       — sample rate in Hz
//   bit_pos  — bit position in SUPP_PCM_SIZE_RATES
//   base_khz — 44 or 48 (selects base-rate bit 14 in SDnFMT)
//   mult     — rate multiplier (1..4), encoded as (N-1) in bits 13:11
//   div      — rate divisor    (1..8), encoded as (N-1) in bits 10:8
#define HDA_RATE_TABLE(X)         \
    X(  8000,  0, 48, 1, 6)       \
    X( 11025,  1, 44, 1, 4)       \
    X( 16000,  2, 48, 1, 3)       \
    X( 22050,  3, 44, 1, 2)       \
    X( 32000,  4, 48, 2, 3)       \
    X( 44100,  5, 44, 1, 1)       \
    X( 48000,  6, 48, 1, 1)       \
    X( 88200,  7, 44, 2, 1)       \
    X( 96000,  8, 48, 2, 1)       \
    X(176400,  9, 44, 4, 1)       \
    X(192000, 10, 48, 4, 1)

enum {
#define HDA_RATE_BIT_ENUM(hz, bit, base, mult, div) HDA_RATE_BIT_POS_##hz = bit,
    HDA_RATE_TABLE(HDA_RATE_BIT_ENUM)
#undef HDA_RATE_BIT_ENUM
};

enum {
#define HDA_FMT_ENUM(hz, bit, base, mult, div) \
    HDA_FMT_RATE_##hz = (((base) == 44 ? 1u : 0u) << 14) \
                      | (((mult) - 1) << 11) \
                      | (((div)  - 1) <<  8),
    HDA_RATE_TABLE(HDA_FMT_ENUM)
#undef HDA_FMT_ENUM
};

// Bit in CODEC_PARAM_SUPP_PCM_SIZE_RATES advertising a given Hz rate.
// Compile-time constant; pass a literal Hz value present in
// HDA_RATE_TABLE (otherwise expands to an undeclared identifier).
#define HDA_RATE_BIT(hz)         (1u << HDA_RATE_BIT_POS_##hz)

// HDA bit-depth table. Single source of truth for both the SDnFMT BITS
// field (spec 3.3.41 / 3.7.1, bits 6:4) and the matching advertisement
// bits in CODEC_PARAM_SUPP_PCM_SIZE_RATES (spec 7.3.4.7, bits 16..20).
//
// Container size is *not* ceil(bits/8): the spec mandates 32-bit
// containers (4 bytes) for both 20-bit and 24-bit samples. Encoding the
// container size in the same table the worker reads is the whole point —
// the previous nested ternary in the worker derived bytes_per_sample
// from the bit count and got 20/24-bit wrong by 1 byte (33% pacing
// error), masked only because the codec advertised 16-bit only.
//
// X(bits, code, container_bytes, advert_bit):
//   bits            — sample width, used only for naming
//   code            — value of SDnFMT BITS field (0..4); 5..7 reserved
//   container_bytes — bytes per sample in memory (1, 2, or 4)
//   advert_bit      — bit position in SUPP_PCM_SIZE_RATES (16..20)
#define HDA_BITS_TABLE(X)        \
    X( 8, 0, 1, 16)              \
    X(16, 1, 2, 17)              \
    X(20, 2, 4, 18)              \
    X(24, 3, 4, 19)              \
    X(32, 4, 4, 20)

// Advertisement-bit constants HDA_PCM_SIZE_8..HDA_PCM_SIZE_32 generated
// from the bit-depth table; OR them into a SUPP_PCM_SIZE_RATES response.
enum {
#define HDA_PCM_SIZE_ENUM(bits, code, bytes, ad) HDA_PCM_SIZE_##bits = (1u << (ad)),
    HDA_BITS_TABLE(HDA_PCM_SIZE_ENUM)
#undef HDA_PCM_SIZE_ENUM
};

// Container size in bytes indexed by SDnFMT BITS field (0..7). Reserved
// codes 5..7 default-init to 0; callers treat 0 as an invalid format and
// bail (see sound_hda_stream_drain).
static const uint8_t hda_fmt_container_bytes[8] = {
#define HDA_BITS_BYTES_INIT(bits, code, bytes, ad) [code] = (bytes),
    HDA_BITS_TABLE(HDA_BITS_BYTES_INIT)
#undef HDA_BITS_BYTES_INIT
};

// Sized MMIO load/store. Use these in every register handler instead of
// bare read_uint{8,16,32}_le — they respect the access width the bus
// reports, so 1-byte writes to a 4-byte register don't pull garbage from
// the surrounding payload (which is uninitialised stack on the dispatch
// path), and reads return only the bytes the guest asked for. Static
// inline because they're 100% used inside this file; no point in
// promoting to a shared header until a second device wants them.
static inline uint32_t mmio_load(const void *data, uint8_t size)
{
    switch (size) {
        case 1:  return read_uint8(data);
        case 2:  return read_uint16_le(data);
        case 4:  return read_uint32_le(data);
        default: return 0;
    }
}

static inline void mmio_store(void *data, uint8_t size, uint32_t val)
{
    switch (size) {
        case 1: write_uint8(data, (uint8_t)val); break;
        case 2: write_uint16_le(data, (uint16_t)val); break;
        case 4: write_uint32_le(data, val); break;
    }
}

#define CODEC_PARAM_SUPP_STREAM_FMTS                     0x0B
#define CODEC_PARAM_SUPP_STREAM_FMTS_PCM                 (1 << 0)
#define CODEC_PARAM_SUPP_STREAM_FMTS_FLOAT32             (1 << 1)
#define CODEC_PARAM_SUPP_STREAM_FMTS_AC3                 (1 << 2)

#define CODEC_PARAM_PIN_CAPS                             0x0C
#define CODEC_PARAM_PIN_CAPS_IMP_SENSE                   (1 <<  0)
#define CODEC_PARAM_PIN_CAPS_TRIGGER_REQD                (1 <<  1)
#define CODEC_PARAM_PIN_CAPS_PRESENSE_DETECT             (1 <<  2)
#define CODEC_PARAM_PIN_CAPS_HEADPHONE                   (1 <<  3)
#define CODEC_PARAM_PIN_CAPS_OUTPUT                      (1 <<  4)
#define CODEC_PARAM_PIN_CAPS_INPUT                       (1 <<  5)
#define CODEC_PARAM_PIN_CAPS_BALANCED_IO                 (1 <<  6)
#define CODEC_PARAM_PIN_CAPS_HDMI                        (1 <<  7)
#define CODEC_PARAM_PIN_CAPS_VREF_CTRL_HIZ               (1 <<  8)
#define CODEC_PARAM_PIN_CAPS_VREF_CTRL_50                (1 <<  9)
#define CODEC_PARAM_PIN_CAPS_VREF_CTRL_GROUND            (1 << 10)
#define CODEC_PARAM_PIN_CAPS_VREF_CTRL_80                (1 << 12)
#define CODEC_PARAM_PIN_CAPS_VREF_CTRL_100               (1 << 13)

#define CODEC_PARAM_INPUT_AMP_CAPS                       0x0D

#define CODEC_PARAM_OUTPUT_AMP_CAPS                      0x12
#define CODEC_PARAM_OUTPUT_AMP_CAPS_MUTE_CAP             (   1 << 31)
#define CODEC_PARAM_OUTPUT_AMP_CAPS_STEPSIZE             (   3 << 16)
#define CODEC_PARAM_OUTPUT_AMP_CAPS_NUMSTEPS             (0x4a <<  8)
#define CODEC_PARAM_OUTPUT_AMP_CAPS_OFFSET               (0x4a <<  0) // Why 0x4a?

#define CODEC_PARAM_CONN_LIST_LEN                        0x0E
#define CODEC_PARAM_SUPP_POWER_STATES                    0x0F
#define CODEC_PARAM_PROCESSING_CAPS                      0x10
#define CODEC_PARAM_GPIO_CNT                             0x11
#define CODEC_PARAM_VOLUME_KNOB                          0x12

// Stream descriptor address layout (HDA spec §3.3 page 27):
//   ISDn at 0x80 + n*0x20                   for n in [0, ISS)
//   OSDn at 0x80 + (ISS + n)*0x20           for n in [0, OSS)
//   BSDn at 0x80 + (ISS + OSS + n)*0x20     for n in [0, BSS)
// Descriptor index — used as the SIE/SIS bit in INTCTL/INTSTS per
// §3.3.14 — increases sequentially in that order.
#define HDA_STREAM_BASE         0x80
#define HDA_STREAM_STRIDE       0x20
#define HDA_STREAMS_TOTAL       (SOUND_HDA_PARAM_NO_IN  \
                               + SOUND_HDA_PARAM_NO_OUT \
                               + SOUND_HDA_PARAM_NO_BSS)
#define HDA_STREAM_REGION_END   (HDA_STREAM_BASE + HDA_STREAMS_TOTAL * HDA_STREAM_STRIDE)

typedef enum {
    HDA_STREAM_DIR_INPUT  = 0,
    HDA_STREAM_DIR_OUTPUT = 1,
    HDA_STREAM_DIR_BIDIR  = 2,
} hda_stream_dir_t;

// Forward decl so sound_hda_stream_t can keep a back-pointer for the
// worker thread. Worker arg is a sound_hda_stream_t*; the dev pointer
// rides along inside the stream so we don't need a heap-allocated
// (dev, stream) tuple per stream.
struct sound_hda_dev_s;
typedef struct sound_hda_dev_s sound_hda_dev_t;

typedef struct {
    sound_hda_dev_t *hda;      // Back-ref to owning dev — set at init,
                               // never reseated. Lets the worker find
                               // pci_func / lock from a stream pointer.
    uint8_t     index;         // Descriptor index (0..HDA_STREAMS_TOTAL-1).
                               // Equals the SIE/SIS bit position per spec
                               // §3.3.14 — caches it as intsts_bit for the
                               // hot IRQ path even though they're identical.
    uint8_t     intsts_bit;
    uint8_t     dir;           // hda_stream_dir_t
    uint32_t    bdl_lvi;       // Spec stores 8 bits; widened to 32 here so
                               // the storage-register dispatch can pass a
                               // single offsetof()/sizeof() pair.
    uint32_t    bdl_lo;
    uint32_t    bdl_hi;
    uint32_t    bdl_len;
    uint32_t    lpib;
    uint8_t     ioce;
    uint8_t     feie;          // SDnCTL bit 3 — stored for RMW round-trip
    uint8_t     deie;          // SDnCTL bit 4 — stored for RMW round-trip
    uint8_t     srst;          // SDnCTL bit 0 — mirrored for the SRST 0→1 edge
    uint8_t     stripe;        // SDnCTL bits 17:16 — stored for round-trip
    uint8_t     tp;            // SDnCTL bit 18 — stored for round-trip
    uint8_t     ctl_strm;      // SDnCTL bits 23:20 — stream tag set by guest
                               // driver. Mirrors the codec-side `stream` field
                               // below (verb VERB_SET_CONV_STREAM_CHAN); the
                               // two are required to match per spec §7.3.3.8
                               // and the codec uses the verb-side one.
    uint8_t     stream;        // Codec-side stream tag from
                               // VERB_SET_CONV_STREAM_CHAN.
    uint8_t     channel;
    uint32_t    running;       // Guest intent: 1 = stream should be running
    uint32_t    worker_alive;  // Worker lifetime: 1 = worker thread exists
    uint16_t    fmt;           // SDnFMT register (HDA spec 7.3.3.10): full 16
                               // bits — channels(0:3), size(4:6), divisor(8:10),
                               // multiplier(11:13), base(14). Worker reads this
                               // to derive the pacing rate; truncating to 8 bits
                               // loses divisor / multiplier / base and leaves the
                               // worker pacing as 48 kHz 8-bit mono regardless of
                               // what the guest configured.
    uint8_t     status;
    uint8_t     left_gain;
    uint8_t     right_gain;
    uint8_t     left_mute;
    uint8_t     right_mute;
} sound_hda_stream_t;

struct sound_hda_dev_s {
    pci_func_t* pci_func;
    spinlock_t  lock;
    uint32_t    gctl;
    uint32_t    intr_ctrl;
    uint32_t    corb_lo;
    uint32_t    corb_hi;
    uint16_t    corb_rp;
    uint16_t    corb_wp;
    uint32_t    corb_size;
    uint32_t    rirb_lo;
    uint32_t    rirb_hi;
    uint32_t    rirb_rp;
    uint32_t    rirb_wp;
    uint32_t    rirb_size;
    uint32_t    rirb_cnt;
    uint32_t    rirb_status;
    uint32_t    power_state;

    sound_hda_stream_t streams[HDA_STREAMS_TOTAL];
    sound_subsystem_t  subsystem;
};

// Convenience accessor for the (currently single) output stream. The codec
// verbs route there directly; the MMIO dispatch addresses streams by
// descriptor index and doesn't need it.
#define HDA_OUTPUT_STREAM_INDEX  SOUND_HDA_PARAM_NO_IN
static inline sound_hda_stream_t *hda_output_stream(sound_hda_dev_t *hda) {
    return &hda->streams[HDA_OUTPUT_STREAM_INDEX];
}

// Forward decl — SDnCTL action body lives further down with the rest of
// the worker / start-stop machinery.
static void sound_hda_stream_ctl_action(sound_hda_dev_t *hda,
                                        sound_hda_stream_t *stream,
                                        uint32_t value);

// === Stream descriptor register dispatch ===
//
// All input, output, and bidirectional stream descriptors share the same
// 0x20-byte register layout (HDA spec §3.3.34). One table services every
// stream — adding ISD0 / multi-stream means changing HDA_STREAMS_TOTAL,
// not the table.
//
// Registers are categorised by `kind`:
//   SD_REG_RW_FIELD — read returns the stored field, writes store into it
//   SD_REG_RO_FIELD — read returns the stored field, writes ignored
//   SD_REG_RO_FN    — read calls a compute callback, writes ignored
//   SD_REG_ACTION   — both directions go through callbacks (used for
//                     SDnCTL where bits compose from multiple fields and
//                     the write must spawn / signal the worker, and for
//                     SDnSTS where reads compute FIFORDY and writes are
//                     RW1C against `status`).
typedef enum {
    SD_REG_RW_FIELD = 0,
    SD_REG_RO_FIELD,
    SD_REG_RO_FN,
    SD_REG_ACTION,
} sd_reg_kind_t;

typedef uint32_t (*sd_reg_read_fn) (sound_hda_dev_t*, sound_hda_stream_t*);
typedef void     (*sd_reg_write_fn)(sound_hda_dev_t*, sound_hda_stream_t*, uint32_t);

typedef struct {
    uint16_t        sub_off;     // offset within the 0x20 stream block
    uint8_t         width;       // canonical register width (1, 2, or 4)
    sd_reg_kind_t   kind;
    uint16_t        field_off;   // offsetof(sound_hda_stream_t, field)
    uint8_t         field_size;  // sizeof(field) — 1, 2, or 4
    sd_reg_read_fn  read_fn;     // ACTION / RO_FN
    sd_reg_write_fn write_fn;    // ACTION
    const char     *name;        // diagnostics; matches the spec mnemonic
} sd_reg_t;

// Read/write a stream-struct field by (offset, size) — treats it as a
// little-endian unsigned integer of the requested width. Used by the
// storage-kind dispatch entries.
static uint32_t sd_field_load(sound_hda_stream_t *s, uint16_t off, uint8_t sz)
{
    uint8_t *base = (uint8_t*)s + off;
    switch (sz) {
        case 1:  return *(uint8_t  *)base;
        case 2:  return *(uint16_t *)base;
        case 4:  return *(uint32_t *)base;
        default: return 0;
    }
}

static void sd_field_store(sound_hda_stream_t *s, uint16_t off, uint8_t sz, uint32_t v)
{
    uint8_t *base = (uint8_t*)s + off;
    switch (sz) {
        case 1: *(uint8_t  *)base = (uint8_t)v;  break;
        case 2: *(uint16_t *)base = (uint16_t)v; break;
        case 4: *(uint32_t *)base = v;           break;
    }
}

// SDnCTL (HDA spec §3.3.35): 24-bit register; bits 0 SRST, 1 RUN, 2
// IOCE, 3 FEIE, 4 DEIE, 17:16 STRIPE, 18 TP, 19 DIR (RO 0 unless bidir),
// 23:20 STRM (stream tag). Composed on read; write defers to the
// action body further down (worker spawn, SRST edge, etc.).
static uint32_t sd_ctl_read(sound_hda_dev_t *hda, sound_hda_stream_t *s)
{
    (void)hda;
    uint32_t v = 0;
    v |= (s->srst       & 1u) << 0;
    v |= (atomic_load_uint32_relax(&s->running) ? 1u : 0u) << 1;
    v |= (s->ioce       & 1u) << 2;
    v |= (s->feie       & 1u) << 3;
    v |= (s->deie       & 1u) << 4;
    v |= ((uint32_t)s->stripe  & 0x3u) << 16;
    v |= (s->tp         & 1u) << 18;
    v |= (s->dir == HDA_STREAM_DIR_BIDIR ? 1u : 0u) << 19;
    v |= ((uint32_t)s->ctl_strm & 0xFu) << 20;
    return v;
}

static void sd_ctl_write(sound_hda_dev_t *hda, sound_hda_stream_t *s, uint32_t v)
{
    sound_hda_stream_ctl_action(hda, s, v);
}

// SDnSTS (HDA spec §3.3.36): bits 2 BCIS, 3 FIFOE, 4 DESE are RW1C
// against `status`; bit 5 FIFORDY is computed RO. The worker latches
// BCIS into `status` under hda->lock; reads return it as-is.
static uint32_t sd_sts_read(sound_hda_dev_t *hda, sound_hda_stream_t *s)
{
    (void)hda;
    uint32_t v = s->status & 0x1Cu;
    if (s->lpib < s->bdl_len) v |= (1u << 5);
    return v;
}

static void sd_sts_write(sound_hda_dev_t *hda, sound_hda_stream_t *s, uint32_t v)
{
    (void)hda;
    s->status &= ~(uint8_t)(v & 0x1Cu);
}

// SDnFIFOS (HDA spec §3.3.40): max FIFO depth in bytes. Reset value is
// implementation-specific; we report a constant for both directions.
static uint32_t sd_fifos_read(sound_hda_dev_t *hda, sound_hda_stream_t *s)
{
    (void)hda; (void)s;
    return SOUND_HDA_FIFO_SIZE;
}

#define SD_FIELD_OFF(field)   offsetof(sound_hda_stream_t, field)
#define SD_FIELD_SIZE(field)  sizeof(((sound_hda_stream_t*)0)->field)
#define SD_RW(off, w, field) \
    { (off), (w), SD_REG_RW_FIELD, SD_FIELD_OFF(field), SD_FIELD_SIZE(field), NULL, NULL, #field }
#define SD_RO(off, w, field) \
    { (off), (w), SD_REG_RO_FIELD, SD_FIELD_OFF(field), SD_FIELD_SIZE(field), NULL, NULL, #field }

// Stream descriptor register table (§3.3.35–§3.3.43). Adding a new
// register or wiring a missing one is a one-row change; the dispatch
// below has no per-register knowledge.
static const sd_reg_t sd_regs[] = {
    { 0x00, 4, SD_REG_ACTION, 0, 0, sd_ctl_read,   sd_ctl_write,  "SDnCTL"   },
    { 0x03, 1, SD_REG_ACTION, 0, 0, sd_sts_read,   sd_sts_write,  "SDnSTS"   },
    SD_RO(0x04, 4, lpib),       // SDnLPIB — RO; worker advances this
    SD_RW(0x08, 4, bdl_len),    // SDnCBL  — Cyclic Buffer Length
    SD_RW(0x0C, 2, bdl_lvi),    // SDnLVI  — Last Valid Index
    { 0x10, 2, SD_REG_RO_FN,  0, 0, sd_fifos_read, NULL,          "SDnFIFOS" },
    SD_RW(0x12, 2, fmt),        // SDnFMT  — decoded by stream worker
    SD_RW(0x18, 4, bdl_lo),     // SDnBDPL — BDL pointer low (128-B aligned)
    SD_RW(0x1C, 4, bdl_hi),     // SDnBDPU — BDL pointer high (RO 0 if !64-bit)
};

#undef SD_RW
#undef SD_RO
#undef SD_FIELD_OFF
#undef SD_FIELD_SIZE

static const sd_reg_t *sd_reg_lookup(uint16_t sub_off)
{
    for (size_t i = 0; i < sizeof(sd_regs) / sizeof(sd_regs[0]); ++i) {
        if (sd_regs[i].sub_off == sub_off) return &sd_regs[i];
    }
    return NULL;
}

static bool sd_dispatch_read(sound_hda_dev_t *hda, sound_hda_stream_t *s,
                             uint16_t sub_off, void *data, uint8_t size)
{
    const sd_reg_t *r = sd_reg_lookup(sub_off);
    if (r == NULL) return false;
    uint32_t v = 0;
    switch (r->kind) {
        case SD_REG_RW_FIELD:
        case SD_REG_RO_FIELD:
            v = sd_field_load(s, r->field_off, r->field_size);
            break;
        case SD_REG_RO_FN:
        case SD_REG_ACTION:
            v = r->read_fn(hda, s);
            break;
    }
    mmio_store(data, size, v);
    return true;
}

static bool sd_dispatch_write(sound_hda_dev_t *hda, sound_hda_stream_t *s,
                              uint16_t sub_off, const void *data, uint8_t size)
{
    const sd_reg_t *r = sd_reg_lookup(sub_off);
    if (r == NULL) return false;
    uint32_t v = mmio_load(data, size);
    switch (r->kind) {
        case SD_REG_RW_FIELD:
            sd_field_store(s, r->field_off, r->field_size, v);
            break;
        case SD_REG_RO_FIELD:
        case SD_REG_RO_FN:
            // Read-only — writes ignored (no error per §3.1.2).
            break;
        case SD_REG_ACTION:
            r->write_fn(hda, s, v);
            break;
    }
    return true;
}

static void sound_hda_remove(rvvm_mmio_dev_t* dev)
{
    // Halt the stream worker before the PCI function is torn down.
    // Without this, a worker still walking the BDL will call
    // pci_send_irq() / pci_get_dma_ptr() on a freed pci_func and crash.
    //
    // running=0 asks the worker to exit. Paired with the running check
    // inside the inner BDL loop in sound_hda_stream_drain, the worker
    // bails within one backend write — it does not run the pacing
    // sleep or IRQ dispatch on an entry observed after shutdown, so
    // no pci_func access outlives this call.
    //
    // worker_alive is cleared by the worker when it actually returns —
    // polling it is a reliable join, since thread_create_task is
    // fire-and-forget and RVVM has no matching thread handle here.
    //
    // We wait unbounded: hanging on teardown is recoverable (the host
    // notices and kills the process), but returning while the worker
    // is still live and using hda->pci_func is a use-after-free the
    // moment the caller frees the PCI state. Log a one-shot warning
    // after 5 s as a diagnostic breadcrumb for a wedged backend.
    sound_hda_dev_t *hda = dev->data;
    if (hda == NULL) return;

    // Ask every stream worker to stop.
    for (size_t i = 0; i < HDA_STREAMS_TOTAL; ++i) {
        atomic_store_uint32_relax(&hda->streams[i].running, 0);
    }
    // Unblock any worker stuck inside subsystem.write before we start
    // waiting. Without this, blocking backends (ALSA PCM mid-xrun, IPC
    // sinks, any callback that doesn't poll running itself) stall
    // teardown for as long as the host takes to drain — which for
    // PipeWire under load can be seconds. Non-blocking backends leave
    // abort NULL; the running check in sound_hda_stream_drain is enough.
    if (hda->subsystem.abort != NULL) {
        hda->subsystem.abort(&hda->subsystem);
    }
    // Join every stream worker. Wait unbounded: hanging on teardown is
    // recoverable (the host notices and kills the process), but
    // returning while a worker still uses hda->pci_func is a
    // use-after-free the moment the caller frees the PCI state.
    uint32_t waited_ms = 0;
    for (;;) {
        bool any_alive = false;
        for (size_t i = 0; i < HDA_STREAMS_TOTAL; ++i) {
            if (atomic_load_uint32_relax(&hda->streams[i].worker_alive)) {
                any_alive = true;
                break;
            }
        }
        if (!any_alive) break;
        sleep_ms(5);
        waited_ms += 5;
        if (waited_ms == 5000) {
            DO_ONCE(rvvm_warn("sound_hda_remove: stream worker still alive"
                              " after 5 s; backend may be blocking"));
        }
    }
}

static rvvm_mmio_type_t sound_hda_type = {
    .name = "intel_hda",
    .remove = sound_hda_remove,
};

// Compose INTSTS (HDA spec §3.3.15): bits 0..29 are per-stream SIS flags
// (set when SDnSTS has any of BCIS/FIFOE/DESE latched), bit 30 CIS, bit
// 31 GIS = OR of everything else. The SIS bit number equals the stream's
// descriptor index per §3.3.14.
//
// Earlier this hardcoded `1 << 29` (no real stream maps there) and
// Linux's azx_interrupt silently discarded every IRQ. Iterating
// `streams[]` keeps the read correct as new descriptors come online.
static uint32_t sound_hda_compute_intsts(sound_hda_dev_t *hda)
{
    uint32_t sis = 0;
    for (size_t i = 0; i < HDA_STREAMS_TOTAL; ++i) {
        if (hda->streams[i].status & 0x1Cu) {
            sis |= 1u << hda->streams[i].intsts_bit;
        }
    }
    if (sis) sis |= (1u << 31);  // GIS mirrors any pending SIS
    return sis;
}

// Wall clock (HDA spec §3.3.16): 32-bit counter at 24 MHz. Used by
// Linux's azx_position_ok as a sanity gate against bogus IRQs; returning
// 0 here makes the driver reject every period interrupt and writers
// stall in wait_for_avail. With a real monotonic 24 MHz counter the gate
// passes once per period and snd_pcm_period_elapsed wakes writers.
static uint32_t sound_hda_wallclk(void)
{
    return (uint32_t)rvtimer_clocksource(24000000ULL);
}

// Resolve an MMIO offset in the [0x80, 0x80 + N*0x20) range to a
// (stream, sub_off) pair. Returns NULL on miss (offset is global or out
// of range).
static sound_hda_stream_t *hda_resolve_stream(sound_hda_dev_t *hda,
                                              size_t offset, uint16_t *sub_off)
{
    if (offset < HDA_STREAM_BASE || offset >= HDA_STREAM_REGION_END) return NULL;
    size_t   rel = offset - HDA_STREAM_BASE;
    uint32_t idx = rel / HDA_STREAM_STRIDE;
    *sub_off = rel % HDA_STREAM_STRIDE;
    return &hda->streams[idx];
}

static bool sound_hda_mmio_read(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    sound_hda_dev_t *hda = dev->data;
    spin_lock(&hda->lock);
    bool ok = true;
    uint32_t v = 0;

    switch (offset) {
        case SOUND_HDA_GCAP:        v = SOUND_HDA_PARAM_GCAP; break;
        case SOUND_HDA_VS:          v = SOUND_HDA_PARAM_V;    break;
        case SOUND_HDA_OUTPAY:      v = 0x3C;                 break;
        case SOUND_HDA_INPAY:       v = 0x1D;                 break;
        case SOUND_HDA_GLOBAL_CTRL:
            // GCTL (HDA spec §3.3.7). Bit 0 CRST is RWS — readback
            // reflects what was last written so guest reset polls see
            // both CRST=0 (in-reset) and CRST=1 (out-of-reset)
            // transitions. Bit 8 UNSOL is force-on: we accept
            // unsolicited responses unconditionally (the codec doesn't
            // currently emit any, so this is purely advertised
            // capability). Other bits round-trip from the stored value.
            v = hda->gctl | (1u << 8);
            break;
        case SOUND_HDA_INTR_CTRL:   v = hda->intr_ctrl;             break;
        case SOUND_HDA_INTSTS:      v = sound_hda_compute_intsts(hda); break;
        case SOUND_HDA_WALL_CLOCK:  v = sound_hda_wallclk();        break;
        case SOUND_HDA_CORB_WP:     v = hda->corb_wp;               break;
        case SOUND_HDA_CORB_RP:     v = hda->corb_rp & 0xFFu;       break;
        case SOUND_HDA_CORB_SIZE:
            v = (SOUND_HDA_PARAM_CORBSZCAP << 4) | SOUND_HDA_PARAM_CORBSIZE;
            break;
        case SOUND_HDA_RIRB_WP:     v = hda->rirb_wp & 0xFFu;       break;
        case SOUND_HDA_RIRB_STATUS: v = hda->rirb_status & 0xFFu;   break;
        case SOUND_HDA_RIRB_SIZE:
            v = (SOUND_HDA_PARAM_RIRBSZCAP << 4) | SOUND_HDA_PARAM_RIRBSIZE;
            break;
        default: {
            // Stream descriptor block — dispatch through the SD register
            // table. Same dispatch services every input, output, and
            // bidirectional descriptor (HDA spec §3.3.34: shared layout).
            uint16_t sub_off;
            sound_hda_stream_t *s = hda_resolve_stream(hda, offset, &sub_off);
            if (s != NULL && sd_dispatch_read(hda, s, sub_off, data, size)) {
                spin_unlock(&hda->lock);
                return true;
            }
            ok = false;
            break;
        }
    }

    if (ok) mmio_store(data, size, v);
    spin_unlock(&hda->lock);
    return ok;
}

static void sound_hda_write_rirb(sound_hda_dev_t *hda, uint32_t cad, uint32_t response)
{
    ++hda->rirb_wp;
    hda->rirb_wp %= hda->rirb_size;

    // Each RIRB entry is 8 bytes (HDA spec §3.3.27 / §4.4.2.1: response
    // dword + response_ex dword). Map the entry, then store the two
    // dwords at indices 0 and 1.
    //
    // Earlier this used `rirb_lo + rirb_wp*4` for the base (treating WP
    // as a dword index) and then indexed `rirb[rirb_wp]` and
    // `rirb[rirb_wp+1]` (treating WP again, this time as a dword
    // offset on top of the already-shifted base) — two compounding
    // errors that landed at `rirb_lo + rirb_wp*8` *only* because the
    // host returned a pointer to a mapping wider than the 4 bytes
    // requested. Strict bounds checking, or a RIRB allocated with a
    // page boundary mid-ring, broke it.
    uint32_t *rirb = pci_get_dma_ptr(
        hda->pci_func,
        (rvvm_addr_t)hda->rirb_lo + hda->rirb_wp * 8,
        8
    );
    if (rirb == NULL) return;

    rirb[0] = response;
    rirb[1] = cad;       // response_ex (codec address in bits 3:0)
    pci_send_irq(hda->pci_func, 0);
}

// NID = 0
static uint32_t sound_hda_codec_root_cmd(uint32_t payload)
{
    switch (payload) {
        case CODEC_PARAM_VENDOR_ID:
            // HDA spec §7.3.4.1: bits 31:16 = Vendor ID, 15:0 = Device
            // ID. Returning just the device ID left vendor=0 — Linux's
            // generic codec path still bound, but any driver that
            // matches on vendor (or logs it for diagnostics) saw 0x0000.
            return ((uint32_t)SOUND_VENDOR_ID_CMEDIA << 16) | SOUND_DEVICE_ID_CMEDIA;

        case CODEC_PARAM_REVISION_ID:
            return 0xFFFF;

        case CODEC_PARAM_SUB_NODE_COUNT:
            return 0x00010001; // 1 Subnode, StartNid = 1

        default:
            return 0;
    }
}

// NID = 1
static uint32_t sound_hda_codec_fg_output_cmd(uint32_t payload)
{
    switch (payload) {
        case CODEC_PARAM_SUB_NODE_COUNT:
            return 0x00020002; // 2 Subnode, StartNid = 2

        case CODEC_PARAM_FUNC_GROUP_TYPE:
            return CODEC_PARAM_FUNC_GROUP_TYPE_AUDIO;

        case CODEC_PARAM_SUPP_PCM_SIZE_RATES:
            // Earlier this was locked to 48 kHz only because the worker
            // had no per-stream rate awareness and paced to a fixed
            // bytes/sec — 4× slow playback at 192 kHz, etc. The pacing
            // now derives from stream->fmt (see sound_hda_stream_drain
            // at the SAMPLE_RATE_BYTES_PER_SEC computation), so the
            // worker adapts to whatever the guest actually selects.
            //
            // Locking the codec was overly strict for the common ALSA
            // configuration: HDA-Intel's `default` PCM resolves to raw
            // hw access (no `plug` plugin), so the application's
            // hw_params must match what the codec advertises. With
            // 48 kHz only, anything else (mpg123 at 44.1, speaker-test
            // at any non-48k) silently fails to open and produces no
            // sound — only apps that happen to write 48 kHz mono
            // (aplay of a 48 kHz WAV) work.
            //
            // Advertise 44.1 / 48 / 88.2 / 96 kHz @ 16-bit. Covers
            // MP3 (44.1), WAV/system sounds (48), and hi-res (88.2/96)
            // without offering rates the worker would have to fake.
            // Worker downmixes any inadvertent stereo and pacing
            // adapts via stream->fmt; nothing else needs to know.
            return HDA_PCM_SIZE_16
                 | HDA_RATE_BIT(44100) | HDA_RATE_BIT(48000)
                 | HDA_RATE_BIT(88200) | HDA_RATE_BIT(96000);

        case CODEC_PARAM_SUPP_STREAM_FMTS:
            return CODEC_PARAM_SUPP_STREAM_FMTS_PCM;

        case CODEC_PARAM_SUPP_POWER_STATES:
            return 0xF; // D3, D2, D1, D0

        default:
            return 0;
    }
}

// NID = 2
static uint32_t sound_hda_codec_output_cmd(uint32_t payload)
{
    switch (payload) {
        case CODEC_PARAM_VENDOR_ID:
            // HDA spec §7.3.4.1: bits 31:16 = Vendor ID, 15:0 = Device
            // ID. Returning just the device ID left vendor=0 — Linux's
            // generic codec path still bound, but any driver that
            // matches on vendor (or logs it for diagnostics) saw 0x0000.
            return ((uint32_t)SOUND_VENDOR_ID_CMEDIA << 16) | SOUND_DEVICE_ID_CMEDIA;

        case CODEC_PARAM_REVISION_ID:
            return 0xFFFF;

        case CODEC_PARAM_SUB_NODE_COUNT:
            return 0x00010001; // 1 Subnode, StartNid = 1

        case CODEC_PARAM_AUDIO_WIDGET_CAPS:
            // No STEREO bit: tell the guest driver this widget is mono
            // only. Prevents Linux from configuring a stereo stream
            // (which it will by default on stereo-capable widgets, even
            // for mono input, and then silently downmixes the input
            // duplicating mono → L+R — so the emulator byte rate
            // doubles and pacing diverges).
            return CODEC_PARAM_AUDIO_WIDGET_CAPS_OUTPUT
                 | CODEC_PARAM_AUDIO_WIDGET_CAPS_FORMAT_OVR
                 | CODEC_PARAM_AUDIO_WIDGET_CAPS_AMP_OVR
                 | CODEC_PARAM_AUDIO_WIDGET_CAPS_AMP_OUT;

        case CODEC_PARAM_SUPP_PCM_SIZE_RATES:
            // Match the converter widget's advertised rates (see
            // CODEC_PARAM_SUPP_PCM_SIZE_RATES at the root node).
            return HDA_PCM_SIZE_16
                 | HDA_RATE_BIT(44100) | HDA_RATE_BIT(48000)
                 | HDA_RATE_BIT(88200) | HDA_RATE_BIT(96000);

        case CODEC_PARAM_SUPP_STREAM_FMTS:
            return CODEC_PARAM_SUPP_STREAM_FMTS_PCM;

        case CODEC_PARAM_OUTPUT_AMP_CAPS:
            return CODEC_PARAM_OUTPUT_AMP_CAPS_MUTE_CAP
                 | CODEC_PARAM_OUTPUT_AMP_CAPS_STEPSIZE
                 | CODEC_PARAM_OUTPUT_AMP_CAPS_NUMSTEPS
                 | CODEC_PARAM_OUTPUT_AMP_CAPS_OFFSET;

        default:
            return 0;
    }
}

// NID = 3
static uint32_t sound_hda_codec_pin_output_cmd(uint32_t payload)
{
    switch (payload) {
        case CODEC_PARAM_AUDIO_WIDGET_CAPS:
            // Mono pin to match the output widget; see the output-widget
            // caps comment above.
            return CODEC_PARAM_AUDIO_WIDGET_CAPS_PIN
                 | CODEC_PARAM_AUDIO_WIDGET_CAPS_CONN_LIST;

        case CODEC_PARAM_PIN_CAPS:
            return CODEC_PARAM_PIN_CAPS_OUTPUT
                 | CODEC_PARAM_PIN_CAPS_PRESENSE_DETECT;

        case CODEC_PARAM_CONN_LIST_LEN:
            return 1;

        default:
            return 0;
    }
}

#define NODE_ID_ROOT       0 // Root node
#define NODE_ID_FG_OUTPUT  1 // Output function group
#define NODE_ID_OUTPUT     2 // Output
#define NODE_ID_PIN_OUTPUT 3 // Pin output

static uint32_t sound_hda_codec_stream_cmd(sound_hda_stream_t *stream, uint32_t nid, uint32_t verb, uint32_t payload)
{
    uint32_t response = 0;

    switch (verb) {
        case VERB_GET_CONV_FMT:
            response = stream->fmt;
            break;
        case VERB_SET_CONV_FMT:
            stream->fmt = payload;
            break;
        case VERB_GET_AMP_GAIN_MUTE: {
            // HDA spec §7.3.3.7 Get payload (Figure 62): bit 13 selects
            // left (1) vs right (0) channel; bit 15 selects output (1)
            // vs input (0). Our codec advertises output amps only, so
            // bit 15 doesn't change which side we read.
            //
            // Earlier this used `payload & VERB_GET_AMP_GAIN_MUTE_RIGHT`
            // (== `& 0`) which is always false, so right-channel reads
            // returned 0 unconditionally. ALSA's mixer state for the
            // right channel was a permanent 0 / mute regardless of what
            // the guest had set.
            bool left = (payload & VERB_GET_AMP_GAIN_MUTE_LEFT) != 0;
            uint8_t mute = left ? stream->left_mute : stream->right_mute;
            uint8_t gain = left ? stream->left_gain : stream->right_gain;
            // Get response (Figure 63): bit 7 mute, bits 6:0 gain.
            response = ((uint32_t)(mute & 1u) << 7) | (gain & 0x7Fu);
            break;
        }
        case VERB_SET_AMP_GAIN_MUTE: {
            // HDA spec §7.3.3.7 Set payload (Figure 64): bit 15 Set
            // Output, 14 Set Input, 13 Set Left, 12 Set Right, 7 Mute,
            // 6:0 Gain. Earlier this stored fixed mask constants
            // (`0x80` mute, `0x07` gain) regardless of payload — every
            // guest write left mute=on and gain=7 forever; ALSA volume
            // controls were dead.
            //
            // Codec advertises output amps only — Set Input bit is a
            // no-op for us. Spec also says: "if neither Set Left nor
            // Set Right is set, the command is effectively a no-op."
            if (payload & VERB_SET_AMP_GAIN_MUTE_OUTPUT) {
                uint8_t mute = (payload & VERB_SET_AMP_GAIN_MUTE_MUTE) ? 1u : 0u;
                uint8_t gain = payload & VERB_SET_AMP_GAIN_MUTE_GAIN_MASK;
                if (payload & VERB_SET_AMP_GAIN_MUTE_LEFT) {
                    stream->left_mute = mute;
                    stream->left_gain = gain;
                }
                if (payload & VERB_SET_AMP_GAIN_MUTE_RIGHT) {
                    stream->right_mute = mute;
                    stream->right_gain = gain;
                }
            }
            break;
        }
        case VERB_GET_CONV_STREAM_CHAN:
            response = (stream->stream << 4) | stream->channel;
            break;
        case VERB_SET_CONV_STREAM_CHAN:
            stream->channel = payload & 0x0F;
            stream->stream = (payload >> 4) & 0x0F;
            break;
        case VERB_GET_PIN_WIDGET_CTRL:
            switch (nid) {
            case NODE_ID_PIN_OUTPUT:
                response = VERB_GET_PIN_WIDGET_CTRL_OUT_ENABLE;
                break;
            }
            break;
        default:
            break;
    }

    return response;
}

#define VERB_PARAM_12BIT_SHIFT      8
#define VERB_PARAM_4BIT_SHIFT      16
#define VERB_PARAM_CAD_SHIFT       28
#define VERB_PARAM_NID_SHIFT       20

static void sound_hda_codec_cmd(sound_hda_dev_t *hda, uint32_t cmd)
{
    uint8_t cad = 0;
    uint8_t nid = 0;
    uint16_t verb = 0;
    uint16_t payload = 0;
    uint32_t response = 0;

    // 4.4.1.5 Other CORB Programming Notes
    // Zero is valid CORB command.
    if (cmd == 0)
        return;

    cad = (cmd >> VERB_PARAM_CAD_SHIFT) & 0x0F;
    nid = (cmd >> VERB_PARAM_NID_SHIFT) & 0xFF;

    if ((cmd & 0x70000) == 0x70000) {
        verb = (cmd >> VERB_PARAM_12BIT_SHIFT) & 0x0FFF;
        payload = cmd & 0xFF;
    } else {
        verb = (cmd >> VERB_PARAM_4BIT_SHIFT) & 0x0F;
        payload = cmd & 0xFFFF;
    }

    switch (verb) {
        case VERB_GET_PARAMETER:
            // We define NID count and start indices in CODEC_PARAM_SUB_NODE_COUNT
            switch (nid) {
            case NODE_ID_ROOT:
                response = sound_hda_codec_root_cmd(payload);
                break;
            case NODE_ID_FG_OUTPUT:
                response = sound_hda_codec_fg_output_cmd(payload);
                break;
            case NODE_ID_OUTPUT:
                response = sound_hda_codec_output_cmd(payload);
                break;
            case NODE_ID_PIN_OUTPUT:
                response = sound_hda_codec_pin_output_cmd(payload);
                break;
            }

            break;
        case VERB_GET_SUBSYSTEM_ID:
            response = (SOUND_DEVICE_ID_CMEDIA << 16) | 1;
            break;
        case VERB_SET_POWER_STATE:
            hda->power_state = payload;
            response = 0;
            break;
        case VERB_GET_POWER_STATE:
            //         PS-Set              PS-Act
            response = hda->power_state | (hda->power_state << 4);
            break;
        case VERB_GET_PIN_WIDGET_CTRL:
            response = VERB_GET_PIN_WIDGET_CTRL_OUT_ENABLE;
            break;
        case VERB_GET_PIN_SENSE:
            response = VERB_GET_PIN_SENSE_PRESENSE_PLUGGED;
            break;
        case VERB_GET_CONN_LIST_ENTRY:
            response = NODE_ID_OUTPUT;
            break;
        case VERB_GET_CONFIG_DEFAULT:
            response = VERB_GET_CONFIG_DEFAULT_CONNECTIVITY_JACK
                     | VERB_GET_CONFIG_DEFAULT_DEVICE_LINE_OUT
                     | VERB_GET_CONFIG_DEFAULT_ASSOCIATION_DEFAULT
                     | VERB_GET_CONFIG_DEFAULT_COLOR_ORANGE;
            break;
        case VERB_FUNCTION_RESET:
            // HDA spec §7.3.3.33 verb 0x7FF — reset the function group
            // and all its widgets to power-on values. Configuration
            // Defaults must NOT be reset (we don't store any per-instance
            // overrides for them anyway). Response is 0.
            //
            // Targets the Audio Function Group (NID 1); for our topology
            // that's a single output converter (NID 2) plus pin output
            // (NID 3). Reset the converter's codec-side state — the pin
            // widget has no mutable state we track.
            if (nid == NODE_ID_FG_OUTPUT || nid == NODE_ID_OUTPUT) {
                sound_hda_stream_t *s = hda_output_stream(hda);
                s->fmt        = 0;
                s->channel    = 0;
                s->stream     = 0;
                s->left_gain  = 0;
                s->right_gain = 0;
                s->left_mute  = 0;
                s->right_mute = 0;
            }
            response = 0;
            break;
        default:
            switch (nid) {
            case NODE_ID_OUTPUT:
                response = sound_hda_codec_stream_cmd(hda_output_stream(hda), nid, verb, payload);
                break;
            }
            break;
    }

    sound_hda_write_rirb(hda, cad, response);
}

// Drain the stream once: read current fmt/BDL, pace PCM to the backend
// until the guest clears running (or an unrecoverable condition bails).
// Separate from the worker-lifetime loop below so early-bail paths can
// just return without touching worker_alive.
//
// Operates on a single stream descriptor — generic across input/output/
// bidir streams. Today only output streams have a write_fn backend wired
// in init, so the drain on input streams burns its pacing loop and
// silently advances LPIB; the guest sees an idle but functional stream.
static void sound_hda_stream_drain(sound_hda_stream_t *stream)
{
    sound_hda_dev_t *hda = stream->hda;

    // Pace the worker to the stream's configured bytes-per-second.
    // Without pacing, non-blocking backends (ring buffers, null sinks)
    // let this loop blast through the BDL as fast as the CPU allows:
    // LPIB advances instantly, the guest HDA driver sees its DMA
    // "consume" data faster than it can refill, and aplay trips ALSA's
    // position-consistency assertion ("pcm_plugin.c Assertion
    // status->appl_ptr == *pcm->appl.ptr failed"). Blocking backends
    // (ALSA's snd_pcm_writei) dodge this accidentally because writei
    // blocks when the host buffer fills.
    //
    // The rate is derived from stream->fmt per HDA spec 7.3.3.10:
    //   bit  14      : base rate        (0=48000, 1=44100)
    //   bits 13:11   : rate multiplier  (N+1: 1×, 2×, 3×, 4×)
    //   bits 10:8    : rate divisor     (N+1: /1 .. /8)
    //   bits 6:4     : bits/sample      (0=8, 1=16, 2=20, 3=24, 4=32)
    //   bits 3:0     : channels minus 1 (0=1ch, 1=2ch, …)
    //
    // A previous iteration hardcoded 192 kHz mono then later 48 kHz
    // mono — both broke when the guest driver chose a different stream
    // format (Linux HDA likes to configure stereo streams even for mono
    // content, doubling the true byte rate). Deriving from fmt is the
    // only way to be robust across guest driver choices.
    uint16_t fmt              = stream->fmt;
    uint32_t channels         = (fmt & 0xF) + 1;
    uint32_t bytes_per_sample = hda_fmt_container_bytes[(fmt >> 4) & 7];
    uint32_t mult_code        = (fmt >> 11) & 7;
    uint32_t div              = ((fmt >> 8) & 7) + 1;
    uint32_t base_hz          = (fmt & (1u << 14)) ? 44100 : 48000;
    // Reject formats the spec marks reserved or that we don't render:
    //   - bit 15 TYPE=1 (Non-PCM): codec advertises PCM only
    //   - BITS code 5..7: container size 0 in the lookup table
    //   - MULT code 4..7: spec 3.3.41 / 3.7.1 reserved
    // Folded into the same bail-and-let-guest-retry path as fmt=0 below.
    bool fmt_valid = (fmt & (1u << 15)) == 0
                  && bytes_per_sample != 0
                  && mult_code <= 3;
    uint64_t sample_rate_hz = (uint64_t)base_hz * (mult_code + 1) / div;
    uint32_t bytes_per_frame = channels * bytes_per_sample;
    uint64_t SAMPLE_RATE_BYTES_PER_SEC = sample_rate_hz * bytes_per_frame;
    if (!fmt_valid || SAMPLE_RATE_BYTES_PER_SEC == 0) {
        // Guest wrote run=1 before configuring the format, or programmed
        // an encoding the spec marks reserved. Bail like the NULL-dma
        // case — driver will retry properly.
        atomic_store_uint32_relax(&stream->running, 0);
        return;
    }
    uint64_t       paced_start_ns  = 0;
    uint64_t       paced_bytes_out = 0;

    uint32_t total = stream->bdl_lvi + 1;
    // Map the BDL itself, not the audio data it points to. Each BDL
    // entry is 16 bytes (HDA spec §3.6.3: 8-byte address + 4-byte
    // length + 4-byte flags), so the total BDL byte size is
    // (lvi+1)*16. Earlier this passed `stream->bdl_len` (which is the
    // SDnCBL audio data total — often 64–256 KB) — pci_get_dma_ptr's
    // forgiving range handling let it work in practice, but a strict
    // bounds check or a BDL allocated near the edge of its mapping
    // would land us reading past the end of the BDL into adjacent
    // (or unmapped) guest memory.
    uint64_t bdl_bytes = (uint64_t)total * 16;
    uint64_t *dma = pci_get_dma_ptr(hda->pci_func, stream->bdl_lo, bdl_bytes);

    // If the guest set up the stream control register without a valid BDL
    // (bdl_lo == 0 or invalid), pci_get_dma_ptr returns NULL. Bail out
    // cleanly — the guest's ALSA driver will eventually retry with a
    // proper BDL when userspace opens another PCM handle.
    if (dma == NULL) {
        atomic_store_uint32_relax(&stream->running, 0);
        return;
    }

    while (atomic_load_uint32_relax(&stream->running)) {
        for (uint32_t i = 0; i < total; ++i) {
            uint64_t *bdle = &dma[i * 2];
            uint64_t addr = bdle[0];
            uint32_t len  = bdle[1] & 0xFFFFFFFF;
            uint8_t  ioc  = bdle[1] >> 32 & 1;

            // Dispatch PCM to the configured host-side backend. If no backend
            // was installed at init time (neither a compile-time USE_ALSA nor
            // a caller-supplied write_fn via sound_hda_init_ex), the HDA
            // device enumerates on the PCI bus but this path is a no-op —
            // the guest sees a working device and the LPIB counter advances
            // so its driver doesn't stall, but PCM data is silently dropped.
            //
            // Backend contract: always receives MONO 16-bit LE at the
            // stream's configured rate. If the guest driver configured a
            // multi-channel stream (Linux's HDA code likes stereo even
            // when the codec widget advertises mono capability), we
            // downmix here by averaging channels. Keeps backends simple
            // (they don't need to know channel count) and the pacing
            // math below stays 1:1 with the bytes they see.
            if (hda->subsystem.write != NULL) {
                void *pcm = pci_get_dma_ptr(hda->pci_func, addr, len);
                if (pcm == NULL) {
                    // Bad guest BDL entry (zero addr, unmapped region, or
                    // zero len). Don't feed NULL to the backend — ALSA's
                    // snd_pcm_writei(NULL, n) is UB and ringbuf writes
                    // memcpy from NULL. Pacing still advances below so
                    // LPIB keeps moving and the guest can recover.
                } else if (channels == 1) {
                    hda->subsystem.write(&hda->subsystem, pcm, len);
                } else if (bytes_per_sample == 2) {
                    // Common case: 16-bit multi-channel → 16-bit mono.
                    // Stack-allocate — BDL entries are small (256-4096 B).
                    size_t frame_bytes_in = (size_t)bytes_per_frame;
                    size_t frames         = len / frame_bytes_in;
                    int16_t *src = (int16_t*)pcm;
                    int16_t  mono_buf[4096];
                    size_t bytes_emitted = 0;
                    while (bytes_emitted < frames * 2) {
                        size_t chunk_frames = frames - (bytes_emitted / 2);
                        if (chunk_frames > 4096) chunk_frames = 4096;
                        for (size_t f = 0; f < chunk_frames; f++) {
                            int32_t sum = 0;
                            size_t base = (bytes_emitted / 2 + f) * channels;
                            for (uint32_t c = 0; c < channels; c++) {
                                sum += src[base + c];
                            }
                            mono_buf[f] = (int16_t)(sum / (int32_t)channels);
                        }
                        hda->subsystem.write(&hda->subsystem, mono_buf, chunk_frames * 2);
                        bytes_emitted += chunk_frames * 2;
                    }
                } else {
                    // Rare: non-16-bit multi-channel. Pass raw; backends
                    // that care can parse stream->fmt from the caller.
                    hda->subsystem.write(&hda->subsystem, pcm, len);
                }
            } else {
                UNUSED(addr);
            }

            // Shutdown check between backend write and pacing / IRQ
            // dispatch. The outer `while (running)` only gates the
            // BDL pass; once started, a full pass is up to (lvi+1)
            // entries × ~43 ms of pacing — several seconds the
            // device might already be torn down behind us. Bailing
            // here ensures no pci_send_irq / pci_get_dma_ptr call
            // outlives sound_hda_remove(), which is how #208 got
            // a freed pci_func under the IRQ dispatch.
            if (!atomic_load_uint32_relax(&stream->running))
                return;

            // Wall-clock pacing. Compute the ideal elapsed time for the
            // bytes we've emitted so far and sleep the difference.
            paced_bytes_out += len;
            uint64_t now_ns = rvtimer_clocksource(1000000000ULL);
            if (paced_start_ns == 0) {
                paced_start_ns = now_ns;
            } else {
                uint64_t expected_ns = paced_bytes_out * 1000000000ULL
                                     / SAMPLE_RATE_BYTES_PER_SEC;
                uint64_t elapsed_ns  = now_ns - paced_start_ns;
                if (expected_ns > elapsed_ns) {
                    sleep_ns(expected_ns - elapsed_ns);
                } else if (elapsed_ns > expected_ns + 100000000ULL) {
                    // Fell more than 100 ms behind — rebase so we don't
                    // try to "catch up" by blasting. Happens on machine
                    // resume from pause, or very long Java-side GCs.
                    paced_start_ns  = now_ns;
                    paced_bytes_out = 0;
                }
            }

            stream->lpib += len;
            // If stream longer than BDL length, reset LPIB.
            if (stream->lpib >= stream->bdl_len)
                stream->lpib = 0;
            // When LPIB goes over BDL length, we are done.
            if (stream->bdl_len > 0 && stream->lpib > stream->bdl_len)
                atomic_store_uint32_relax(&stream->running, 0);
            if (ioc) {
                // Latch BCIS (SDnSTS bit 2) unconditionally so INTSTS
                // reflects the real event — the guest clears it via RW1C
                // on OSD0STS (sound_hda_mmio_write). HDA spec 3.3.38
                // mandates latching here; the interrupt-enable gates below
                // control only whether the PCI IRQ line is asserted, not
                // whether the status bit latches. Linux's HDA ISR
                // (snd_hdac_bus_handle_stream_irq) reads SD_STS and only
                // calls snd_pcm_period_elapsed when SD_INT_COMPLETE
                // (== BCIS) is set — without this latch, hw_ptr would
                // stop advancing from the driver's POV and writers blocked
                // in wait_for_avail would never wake.
                //
                // Serialize the latch against the MMIO-side reader/clearer
                // (sound_hda_mmio_read/_write hold hda->lock while touching
                // stream->status via the SD register table). Without the
                // lock this is a torn-byte race that can drop a newly-set
                // BCIS under an in-flight clear and lose the period IRQ.
                spin_lock(&hda->lock);
                stream->status |= 0x04;
                // Gate the PCI IRQ raise on the guest-published enables
                // (HDA spec §3.3.14 / §3.3.36):
                //
                //   IOCE  — SDnCTL bit 2, per-stream "raise IRQ on BDL IOC"
                //   SIE   — INTCTL bit N, per-stream interrupt enable
                //           (N = stream's descriptor index in ISS/OSS/BSS
                //           order — cached as stream->intsts_bit at init)
                //   GIE   — INTCTL bit 31, controller-global interrupt enable
                //
                // All three must be set for the stream's IOC event to drive
                // the PCI INTx/MSI line. Raising while the guest has
                // interrupts disabled (stream close, suspend, etc.) produces
                // a spurious IRQ cascade that preempts the guest vCPU while
                // its ALSA teardown path holds hda->lock via MMIO — a
                // self-contended freeze.
                uint8_t  ioce = stream->ioce;
                uint32_t ic   = hda->intr_ctrl;
                spin_unlock(&hda->lock);
                bool gie = (ic & (1u << 31)) != 0;
                bool sie = (ic & (1u << stream->intsts_bit)) != 0;
                if (ioce && gie && sie) {
                    pci_send_irq(hda->pci_func, 0);
                }
            }
        }
    }
}

static void *sound_hda_stream_worker(void *arg)
{
    sound_hda_stream_t *stream = arg;

    // Worker lifetime is published via worker_alive. Loop handles a narrow
    // missed-wakeup window: guest writes run=1 after we read running=0 but
    // before we clear worker_alive — its spawn CAS sees worker_alive=1
    // and skips, leaving no worker for a running stream. We catch that by
    // re-reading running after the clear, and re-claim the slot to drain
    // again. If a concurrent spawn already won the CAS, they own the next
    // drain — we exit. Either way, exactly one worker runs at a time per
    // stream descriptor.
    for (;;) {
        sound_hda_stream_drain(stream);

        atomic_store_uint32_relax(&stream->worker_alive, 0);

        if (!atomic_load_uint32_relax(&stream->running))
            return NULL;
        if (!atomic_cas_uint32(&stream->worker_alive, 0, 1))
            return NULL;
        // Re-claimed the slot; drain again with freshly-read stream state.
    }
}

// SDnCTL action — invoked by the SD register dispatch on every guest
// write to offset 0x00 of any stream descriptor. Same body for input,
// output, and bidir streams; only the worker spawn is output-specific
// today (input-stream worker would consume from a host source instead).
//
// HDA spec §3.3.35: writing SRST=1 enters reset; the controller must
// report SRST=1 in subsequent reads so software's reset-entry poll
// succeeds, then writing SRST=0 exits reset. Linux's
// snd_hdac_stream_reset() polls up to 300×3μs for each transition.
//
// SRST=1 also resets the stream's per-stream registers per §3.3.35:
// "While in reset, the corresponding stream's registers and associated
// stream FIFO are reset." We clear LPIB and status on the 0→1 edge;
// BDL/FMT/CBL/LVI stay because Linux always rewrites them after
// observing SRST=0.
static void sound_hda_stream_ctl_action(sound_hda_dev_t *hda,
                                        sound_hda_stream_t *stream,
                                        uint32_t cmd)
{
    uint8_t srst    = (cmd >> 0)  & 1u;
    uint8_t run     = (cmd >> 1)  & 1u;
    uint8_t ioce    = (cmd >> 2)  & 1u;
    uint8_t feie    = (cmd >> 3)  & 1u;
    uint8_t deie    = (cmd >> 4)  & 1u;
    uint8_t stripe  = (cmd >> 16) & 0x3u;
    uint8_t tp      = (cmd >> 18) & 1u;
    uint8_t strm    = (cmd >> 20) & 0xFu;

    if (srst && !stream->srst) {
        // Edge: SRST 0→1. Reset per-stream state that real HW would clear.
        stream->lpib   = 0;
        stream->status = 0;
    }
    stream->srst     = srst;
    stream->ioce     = ioce;
    stream->feie     = feie;
    stream->deie     = deie;
    stream->stripe   = stripe;
    stream->tp       = tp;
    stream->ctl_strm = strm;

    // Publish guest intent regardless of direction so MMIO RMW reads of
    // SDnCTL round-trip the RUN bit correctly. The worker spawn is gated
    // on direction: only output streams have a backend wired today, so
    // spawning a drain worker on an input stream would feed BDL bytes
    // (which the guest hasn't written yet) into the output sink. Input /
    // bidir streams stay no-ops on RUN — the guest sees RUN=1 reflected
    // back, then eventually times out waiting for capture data, which is
    // the same observable behaviour as a real codec without microphone
    // input wired up.
    atomic_store_uint32_relax(&stream->running, run ? 1u : 0u);
    if (run && stream->dir == HDA_STREAM_DIR_OUTPUT) {
        // Ordering: running=1 published above, THEN gate the spawn on
        // worker_alive. A worker exiting its while-loop right now will
        // clear worker_alive after its last running-check; our subsequent
        // CAS either wins the slot (worker already gone) or loses it
        // (worker still alive), and the worker's post-clear re-check of
        // running catches the case where we lost the CAS but running=1
        // still needs a worker.
        if (atomic_cas_uint32(&stream->worker_alive, 0, 1)) {
            thread_create_task(sound_hda_stream_worker, stream);
        }
    }
    UNUSED(hda);
}

// GCTL CRST=0 → controller reset (HDA spec §3.3.7). All state machines,
// FIFOs, and MMIO registers clear except WAKEEN, STATESTS, and CRST
// itself. Software is responsible for clearing CORB/RIRB RUN bits and
// stream RUN bits before asserting CRST=0 — but a guest that ignores
// that requirement (or a fresh probe after a previous session left
// streams running) shouldn't leave us with stale state. Halt every
// stream worker via the running flag, zero ring-buffer pointers, and
// reset per-stream MMIO state. UNSOL bit (8) follows CRST.
static void sound_hda_controller_reset(sound_hda_dev_t *hda)
{
    for (size_t i = 0; i < HDA_STREAMS_TOTAL; ++i) {
        sound_hda_stream_t *s = &hda->streams[i];
        atomic_store_uint32_relax(&s->running, 0);
        s->lpib     = 0;
        s->status   = 0;
        s->srst     = 0;
        s->ioce     = 0;
        s->feie     = 0;
        s->deie     = 0;
        s->stripe   = 0;
        s->tp       = 0;
        s->ctl_strm = 0;
        s->bdl_lo   = 0;
        s->bdl_hi   = 0;
        s->bdl_len  = 0;
        s->bdl_lvi  = 0;
        s->fmt      = 0;
    }
    hda->corb_rp = 0;
    hda->corb_wp = 0;
    hda->rirb_rp = 0;
    hda->rirb_wp = 0;
    hda->intr_ctrl = 0;
    // WAKEEN, STATESTS, power_state, codec subsystem registers are
    // explicitly preserved per spec ("only cleared on power-on reset").
    // We don't model wake bits today, so the preservation is moot.
    //
    // We don't wait for workers to drain — they bail on their next
    // running check (within one BDL entry), and the guest's reset poll
    // loop reads CRST=0 immediately. A worker still finishing a backend
    // write while we return is safe because the BDL pointers are now
    // zeroed: any new dma fetch will fail and the worker exits.
}

// CORB write-pointer write (HDA spec §3.3.20). The full DMA model would
// have a CORB engine consume entries from RP+1..WP at its own pace; we
// instead synchronously fetch the entry at WP and dispatch it inline.
// Functionally OK for Linux which bumps WP after every entry, but a
// guest that batches multiple commands before bumping WP loses all but
// the last — a known limitation.
static void sound_hda_corb_wp_write(sound_hda_dev_t *hda, uint32_t v)
{
    hda->corb_wp = v & 0x7Fu;
    uint32_t *cmd = pci_get_dma_ptr(hda->pci_func,
                                    (rvvm_addr_t)hda->corb_lo + hda->corb_wp * 4, 4);
    if (cmd) sound_hda_codec_cmd(hda, *cmd);
}

// CORBSIZE / RIRBSIZE write — encoding (bits 1:0) selects ring depth.
static const uint32_t hda_corb_sizes[4] = { 8,  64, 1024, 0 };
static const uint32_t hda_rirb_sizes[4] = { 16, 128, 2048, 0 };

static bool sound_hda_mmio_write(rvvm_mmio_dev_t* dev, void* data, size_t offset, uint8_t size)
{
    sound_hda_dev_t *hda = dev->data;
    spin_lock(&hda->lock);
    bool ok = true;
    uint32_t v = mmio_load(data, size);

    switch (offset) {
        case SOUND_HDA_GLOBAL_CTRL: {
            // CRST 1→0: enter reset. Halt streams + zero ring/state.
            // CRST 0→1: leave reset (no extra action; readback now
            // returns 1 once the new gctl is stored).
            uint32_t prev = hda->gctl;
            hda->gctl = v;
            if ((prev & 1u) && !(v & 1u)) {
                sound_hda_controller_reset(hda);
            }
            break;
        }
        case SOUND_HDA_INTR_CTRL:   hda->intr_ctrl = v; break;
        case SOUND_HDA_CORB_LO:     hda->corb_lo   = v; break;
        case SOUND_HDA_CORB_HI:     hda->corb_hi   = v; break;
        case SOUND_HDA_CORB_WP:     sound_hda_corb_wp_write(hda, v); break;
        case SOUND_HDA_CORB_RP:
            // Bit 15 self-clears RP per §3.3.21.
            hda->corb_rp = (v & 0x8000u) ? 0u : (v & 0x7Fu);
            break;
        case SOUND_HDA_CORB_SIZE: {
            uint32_t sz = hda_corb_sizes[v & 0x3u];
            if (sz) hda->corb_size = sz;
            break;
        }
        case SOUND_HDA_RIRB_LO:     hda->rirb_lo   = v; break;
        case SOUND_HDA_RIRB_HI:     hda->rirb_hi   = v; break;
        case SOUND_HDA_RIRB_WP:
            hda->rirb_wp = (v & 0x8000u) ? 0u : (v & 0xFFu);
            break;
        case SOUND_HDA_RIRB_INTR_CNT: hda->rirb_cnt = v; break;
        case SOUND_HDA_RIRB_STATUS:
            // RW1C against bits 0/1.
            hda->rirb_status &= ~(v & 0x3u);
            break;
        case SOUND_HDA_RIRB_SIZE: {
            uint32_t sz = hda_rirb_sizes[v & 0x3u];
            if (sz) hda->rirb_size = sz;
            break;
        }
        default: {
            // Stream descriptor block — same dispatch as the read path.
            uint16_t sub_off;
            sound_hda_stream_t *s = hda_resolve_stream(hda, offset, &sub_off);
            if (s != NULL && sd_dispatch_write(hda, s, sub_off, data, size)) {
                spin_unlock(&hda->lock);
                return true;
            }
            ok = false;
            break;
        }
    }

    spin_unlock(&hda->lock);
    return ok;
}

/*
 * Bridge struct stored in sound_subsystem_t.sound_data when a caller-supplied
 * backend is installed via sound_hda_init_ex. Adapts the public two-argument
 * callback to the subsystem's three-argument shape. Freed... never. The HDA
 * device itself is leaked on machine destruction today (see sound_hda_remove),
 * so this follows the same lifecycle.
 */
typedef struct {
    sound_hda_backend_write_fn user_fn;
    sound_hda_backend_abort_fn abort_fn;
    void                       *user_data;
} sound_hda_backend_bridge_t;

static void sound_hda_backend_bridge_write(sound_subsystem_t *sub, void *data, size_t size)
{
    sound_hda_backend_bridge_t *bridge = (sound_hda_backend_bridge_t *)sub->sound_data;
    if (bridge != NULL && bridge->user_fn != NULL) {
        bridge->user_fn(bridge->user_data, data, size);
    }
}

static void sound_hda_backend_bridge_abort(sound_subsystem_t *sub)
{
    sound_hda_backend_bridge_t *bridge = (sound_hda_backend_bridge_t *)sub->sound_data;
    if (bridge != NULL && bridge->abort_fn != NULL) {
        bridge->abort_fn(bridge->user_data);
    }
}

PUBLIC pci_dev_t *sound_hda_init_ex(pci_bus_t *pci_bus,
                                    sound_hda_backend_write_fn write_fn,
                                    sound_hda_backend_abort_fn abort_fn,
                                    void *user_data)
{
    sound_hda_dev_t *sound_hda = safe_new_obj(sound_hda_dev_t);

    // Initialize per-stream identity. Descriptor index = SIE/SIS bit per
    // HDA spec §3.3.14, sequential ISS → OSS → BSS. Today: input slot at
    // index 0, output slot at index 1 (NO_IN=1, NO_OUT=1, NO_BSS=0).
    for (size_t i = 0; i < HDA_STREAMS_TOTAL; ++i) {
        sound_hda_stream_t *s = &sound_hda->streams[i];
        s->hda        = sound_hda;
        s->index      = (uint8_t)i;
        s->intsts_bit = (uint8_t)i;
        if (i < SOUND_HDA_PARAM_NO_IN) {
            s->dir = HDA_STREAM_DIR_INPUT;
        } else if (i < SOUND_HDA_PARAM_NO_IN + SOUND_HDA_PARAM_NO_OUT) {
            s->dir = HDA_STREAM_DIR_OUTPUT;
        } else {
            s->dir = HDA_STREAM_DIR_BIDIR;
        }
    }

    pci_func_desc_t sound_hda_desc = {
        .vendor_id  = SOUND_VENDOR_ID_CMEDIA,
        .device_id  = SOUND_DEVICE_ID_CMEDIA,
        .class_code = SOUND_CLASS_CODE_CMEDIA,
        .prog_if    = 0x00,
        .irq_pin    = PCI_IRQ_PIN_INTA,
        .bar[0] = {
            .size        = 0x4000,
            .min_op_size = 1,
            .max_op_size = 4,
            .read        = sound_hda_mmio_read,
            .write       = sound_hda_mmio_write,
            .data        = sound_hda,
            .type        = &sound_hda_type
        },
    };

    pci_dev_t *pci_dev = pci_attach_func(pci_bus, &sound_hda_desc);
    if (pci_dev)
        sound_hda->pci_func = pci_get_device_func(pci_dev, 0);

    // Backend selection priority:
    //   1. Caller-supplied write_fn (via sound_hda_init_ex) — skip the
    //      compile-time default. Used by embedders that want to route
    //      audio somewhere other than the host's native audio stack
    //      (managed runtimes, IPC channels, WAV capture fixtures, etc.).
    //   2. Compile-time USE_ALSA — the traditional Linux host path.
    //   3. Neither — PCI device enumerates but the stream worker drops PCM.
    if (write_fn != NULL) {
        sound_hda_backend_bridge_t *bridge = safe_new_obj(sound_hda_backend_bridge_t);
        bridge->user_fn = write_fn;
        bridge->abort_fn = abort_fn;
        bridge->user_data = user_data;
        sound_hda->subsystem.sound_data = bridge;
        sound_hda->subsystem.write = sound_hda_backend_bridge_write;
        // Only publish the bridge's abort indirection when the caller
        // actually supplied one — otherwise sound_hda_remove would
        // invoke a no-op indirection every teardown.
        if (abort_fn != NULL) {
            sound_hda->subsystem.abort = sound_hda_backend_bridge_abort;
        }
    } else {
#ifdef USE_ALSA
        if (!alsa_sound_init(&sound_hda->subsystem))
            return NULL;
#endif
    }

    return pci_dev;
}

PUBLIC pci_dev_t *sound_hda_init_auto_ex(rvvm_machine_t *machine,
                                         sound_hda_backend_write_fn write_fn,
                                         sound_hda_backend_abort_fn abort_fn,
                                         void *user_data)
{
    return sound_hda_init_ex(rvvm_get_pci_bus(machine), write_fn, abort_fn, user_data);
}

PUBLIC pci_dev_t *sound_hda_init(pci_bus_t *pci_bus)
{
    return sound_hda_init_ex(pci_bus, NULL, NULL, NULL);
}

PUBLIC pci_dev_t *sound_hda_init_auto(rvvm_machine_t *machine)
{
    return sound_hda_init(rvvm_get_pci_bus(machine));
}

POP_OPTIMIZATION_SIZE
