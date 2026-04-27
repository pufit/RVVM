/*
alsa.c - ALSA sound backend
Copyright (C) 2025  David Korenchuk <github.com/epoll-reactor-2>

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifdef USE_ALSA

#include "atomics.h"
#include "compiler.h"
#include "dlib.h"
#include "sound-hda.h"
#include "spinlock.h"
#include "threading.h"
#include "utils.h"

#include <string.h>


#if CHECK_INCLUDE(alsa/asoundlib.h, 1)
#include <alsa/asoundlib.h>
#endif

#define ASOUND_DLIB_SYM(sym) static __typeof__(sym) *MACRO_CONCAT(sym, _dlib) = NULL;

ASOUND_DLIB_SYM(snd_pcm_open)
ASOUND_DLIB_SYM(snd_pcm_hw_params_malloc)
ASOUND_DLIB_SYM(snd_pcm_hw_params_any)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_access)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_format)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_channels)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_rate_near)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_period_size_near)
ASOUND_DLIB_SYM(snd_pcm_hw_params_set_buffer_size_near)
ASOUND_DLIB_SYM(snd_pcm_hw_params)
ASOUND_DLIB_SYM(snd_pcm_writei)
ASOUND_DLIB_SYM(snd_pcm_readi)
ASOUND_DLIB_SYM(snd_pcm_start)
ASOUND_DLIB_SYM(snd_pcm_prepare)
ASOUND_DLIB_SYM(snd_pcm_drain)
ASOUND_DLIB_SYM(snd_pcm_drop)
ASOUND_DLIB_SYM(snd_pcm_close)
ASOUND_DLIB_SYM(snd_pcm_hw_params_free)

#define snd_pcm_open snd_pcm_open_dlib
#define snd_pcm_hw_params_malloc snd_pcm_hw_params_malloc_dlib
#define snd_pcm_hw_params_any snd_pcm_hw_params_any_dlib
#define snd_pcm_hw_params_set_access snd_pcm_hw_params_set_access_dlib
#define snd_pcm_hw_params_set_format snd_pcm_hw_params_set_format_dlib
#define snd_pcm_hw_params_set_channels snd_pcm_hw_params_set_channels_dlib
#define snd_pcm_hw_params_set_rate_near snd_pcm_hw_params_set_rate_near_dlib
#define snd_pcm_hw_params_set_period_size_near snd_pcm_hw_params_set_period_size_near_dlib
#define snd_pcm_hw_params_set_buffer_size_near snd_pcm_hw_params_set_buffer_size_near_dlib
#define snd_pcm_hw_params snd_pcm_hw_params_dlib
#define snd_pcm_writei snd_pcm_writei_dlib
#define snd_pcm_readi snd_pcm_readi_dlib
#define snd_pcm_start snd_pcm_start_dlib
#define snd_pcm_prepare snd_pcm_prepare_dlib
#define snd_pcm_drain snd_pcm_drain_dlib
#define snd_pcm_drop snd_pcm_drop_dlib
#define snd_pcm_close snd_pcm_close_dlib
#define snd_pcm_hw_params_free snd_pcm_hw_params_free_dlib

// Capture jitter ring. ALSA capture returns whatever's ready in
// period-sized chunks; the HDA input drain pulls in BDL-sized chunks
// (typically 4-8 KiB at 48 kHz mono = 40-80 ms). Sizes don't line up,
// and PipeWire's quantum drifts against HDA's wall-clock pacing — so
// we run a dedicated pump thread that keeps draining ALSA into this
// ring, and the HDA worker reads from the ring. 64 KiB ≈ 680 ms of
// mono S16LE @ 48 kHz, which is plenty of headroom against any
// realistic scheduling jitter without adding noticeable user latency.
#define ALSA_CAPTURE_RING_BYTES 65536u

typedef struct {
    snd_pcm_t *pcm_playback;
    snd_pcm_t *pcm_capture;     // NULL if capture device couldn't be opened

    // Set by alsa_sound_abort() when the HDA device is being removed.
    // alsa_sound_write() checks this on every iteration so a blocked
    // writei that abort has unblocked (via snd_pcm_drop) does not loop
    // back through snd_pcm_prepare + retry and stay stuck. The capture
    // pump thread also checks it as its outer-loop exit condition.
    uint32_t aborted;

    // Capture pump thread handle. Joined in alsa_sound_abort.
    rvvm_thread_t *capture_thread;

    // Lock-protected jitter ring (single-producer pump thread,
    // single-consumer HDA input drain).
    spinlock_t capture_lock;
    uint8_t    capture_buf[ALSA_CAPTURE_RING_BYTES];
    uint32_t   capture_head;    // bytes pushed (producer)
    uint32_t   capture_tail;    // bytes popped (consumer)
} alsa_subsystem_t;

static uint32_t alsa_ring_used(const alsa_subsystem_t *a)
{
    return a->capture_head - a->capture_tail;
}

// Push bytes into the capture ring. Drops oldest on overflow — latency
// over completeness, same policy as the UART/parport bridges.
static void alsa_ring_push(alsa_subsystem_t *a, const uint8_t *src, size_t n)
{
    spin_lock(&a->capture_lock);
    if (alsa_ring_used(a) + n > ALSA_CAPTURE_RING_BYTES) {
        uint32_t overflow = (alsa_ring_used(a) + n) - ALSA_CAPTURE_RING_BYTES;
        a->capture_tail += overflow;
    }
    for (size_t i = 0; i < n; i++) {
        a->capture_buf[(a->capture_head + i) % ALSA_CAPTURE_RING_BYTES] = src[i];
    }
    a->capture_head += n;
    spin_unlock(&a->capture_lock);
}

// Pop up to `n` bytes from the capture ring. Returns the count actually
// returned; caller pads the rest with silence.
static size_t alsa_ring_pop(alsa_subsystem_t *a, uint8_t *dst, size_t n)
{
    spin_lock(&a->capture_lock);
    size_t avail = alsa_ring_used(a);
    size_t got = avail < n ? avail : n;
    for (size_t i = 0; i < got; i++) {
        dst[i] = a->capture_buf[(a->capture_tail + i) % ALSA_CAPTURE_RING_BYTES];
    }
    a->capture_tail += got;
    spin_unlock(&a->capture_lock);
    return got;
}

// Capture pump thread: drains ALSA capture continuously into the jitter
// ring. Period-sized reads (10 ms) keep the host-side latency low; the
// ring smooths against the HDA worker's BDL-sized fetches.
static void *alsa_capture_pump(void *arg)
{
    alsa_subsystem_t *alsa = arg;
    int16_t  buf[480];   // 480 mono frames = 10 ms @ 48 kHz
    int xrun_retries = 4;

    while (!atomic_load_uint32_relax(&alsa->aborted)) {
        snd_pcm_sframes_t n = snd_pcm_readi(alsa->pcm_capture, buf, 480);
        if (n < 0) {
            if (n == -EAGAIN || n == -EINTR) continue;
            if (n == -EPIPE || n == -ESTRPIPE) {
                // Capture xrun = overrun (data dropped). prepare() +
                // start() puts the stream back in the running state so
                // readi resumes returning frames; without start() the
                // post-prepare stream is "prepared" not "running" and
                // readi just blocks indefinitely.
                if (--xrun_retries < 0) return NULL;
                snd_pcm_prepare(alsa->pcm_capture);
                snd_pcm_start(alsa->pcm_capture);
                continue;
            }
            // Unrecoverable (ENODEV, EBADFD, ...) — exit the thread.
            return NULL;
        }
        alsa_ring_push(alsa, (uint8_t*)buf, (size_t)n * 2);
    }
    return NULL;
}

static size_t alsa_sound_read(sound_subsystem_t *subsystem, void *data, size_t size)
{
    alsa_subsystem_t *alsa = subsystem->sound_data;
    if (alsa->pcm_capture == NULL) return 0;
    if (atomic_load_uint32_relax(&alsa->aborted)) return 0;
    return alsa_ring_pop(alsa, (uint8_t*)data, size);
}

static void alsa_sound_write(sound_subsystem_t *subsystem, void *data, size_t size)
{
    alsa_subsystem_t *alsa = subsystem->sound_data;

    // The previous version called snd_pcm_writei exactly once and
    // dropped whatever didn't land: on xrun (-EPIPE) it prepared the
    // PCM but skipped re-writing the chunk; on a positive short return
    // it ignored the tail; on -EINTR it also dropped. Each lost chunk
    // is an audible click/crackle.
    //
    // Loop until every frame is delivered. size is in bytes and we
    // always feed mono 16-bit, so one frame = 2 bytes. Bail on
    // unrecoverable errors after a handful of retries so a disconnected
    // device doesn't spin us forever.
    int16_t *buf = (int16_t*)data;
    snd_pcm_uframes_t remaining = size / 2;
    int xrun_retries = 4;

    while (remaining > 0) {
        // Abort requested by teardown; drop the rest of the chunk so the
        // stream worker unblocks promptly. Checked before writei so a
        // post-drop retry (snd_pcm_drop forces the next writei to return
        // -EPIPE) does not slip back into prepare+retry.
        if (atomic_load_uint32_relax(&alsa->aborted)) return;

        snd_pcm_sframes_t n = snd_pcm_writei(alsa->pcm_playback, buf, remaining);
        if (n < 0) {
            if (n == -EAGAIN || n == -EINTR) {
                // Spurious short-wake; try again with the same data.
                continue;
            }
            if (n == -EPIPE || n == -ESTRPIPE) {
                // Xrun / suspend. Reset the PCM and retry the chunk.
                // -ESTRPIPE means suspended — resume would be ideal but
                // prepare is enough to get us back to a writable state.
                if (--xrun_retries < 0) return;
                snd_pcm_prepare(alsa->pcm_playback);
                continue;
            }
            // Unrecoverable (EBADFD, ENODEV, ...) — give up this chunk.
            return;
        }
        buf       += n;     // int16_t* step = one sample per element
        remaining -= n;
    }
}

static void alsa_sound_abort(sound_subsystem_t *subsystem)
{
    alsa_subsystem_t *alsa = subsystem->sound_data;
    // Publish the flag before unblocking writei. snd_pcm_drop causes any
    // in-flight writei / readi to return -EPIPE; the write loop and the
    // capture pump both see the flag and bail instead of looping
    // through snd_pcm_prepare.
    atomic_store_uint32_relax(&alsa->aborted, 1);
    snd_pcm_drop(alsa->pcm_playback);
    if (alsa->pcm_capture) snd_pcm_drop(alsa->pcm_capture);
    if (alsa->capture_thread) {
        rvvm_thread_join(alsa->capture_thread);
        alsa->capture_thread = NULL;
    }
}

static bool alsa_load_symbols(void)
{
    dlib_ctx_t *libasound = dlib_open("asound", DLIB_NAME_PROBE);
    if (libasound == NULL) {
        rvvm_warn("Cannot load libasound.so");
        return 0;
    }

    bool avail = 1;

#define ASOUND_DLIB_RESOLVE(lib, sym) \
do { \
    sym = dlib_resolve(lib, #sym); \
    avail = avail && !!sym; \
} while (0)

    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_open);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_malloc);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_any);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_access);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_format);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_channels);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_rate_near);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_period_size_near);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_set_buffer_size_near);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_writei);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_readi);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_start);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_prepare);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_drain);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_drop);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_close);
    ASOUND_DLIB_RESOLVE(libasound, snd_pcm_hw_params_free);

    dlib_close(libasound);

    return avail;
}

bool alsa_sound_init(sound_subsystem_t *sound)
{
    bool libasound_avail = true;
    DO_ONCE(libasound_avail = alsa_load_symbols());
    if (!libasound_avail) {
        rvvm_error("Could not load libasound.so");
        return false;
    }

    alsa_subsystem_t *subsystem = safe_new_obj(alsa_subsystem_t);
    if (snd_pcm_open(&subsystem->pcm_playback, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        rvvm_warn("Failed to open ALSA device");
        return false;
    }

    snd_pcm_t *pcm_device = subsystem->pcm_playback;
    snd_pcm_hw_params_t *params = NULL;
    // Match the HDA codec's advertised format (48 kHz mono 16-bit in
    // sound-hda.c's CODEC_PARAM_SUPP_PCM_SIZE_RATES). Opening the host
    // PCM at 192 kHz while the codec feeds 48 kHz streams plays audio
    // 4× fast → two octaves up.
    unsigned int sample_rate = 48000;
    int channels = 1;
    // Host PCM latency. The guest runs its own ALSA buffer on top of the
    // emulated HDA DMA BDL — that's what user-space apps size via
    // snd_pcm_hw_params on the guest. These numbers only control the
    // host-side PipeWire/ALSA ring the emulator feeds into.
    //
    // 128 frames / 2.67 ms was too tight: a single host scheduling hiccup
    // xruns PipeWire and mpg123's write returns short (3400 of 4608) as
    // back-pressure propagates. 960-frame periods (20 ms) with a 4×
    // buffer (80 ms total) give PipeWire enough slack to ride out jitter
    // without noticeable user-facing latency.
    snd_pcm_uframes_t period_frames = 960;
    snd_pcm_uframes_t buffer_frames = 3840;
    int dir = 0;

    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(pcm_device, params);

    snd_pcm_hw_params_set_access(pcm_device, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_device, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_device, params, channels);
    snd_pcm_hw_params_set_rate_near(pcm_device, params, &sample_rate, &dir);
    snd_pcm_hw_params_set_period_size_near(pcm_device, params, &period_frames, &dir);
    snd_pcm_hw_params_set_buffer_size_near(pcm_device, params, &buffer_frames);
    snd_pcm_hw_params(pcm_device, params);

    // Prime the host PCM with two periods of silence before any guest
    // audio shows up. PipeWire (and modern PulseAudio) finishes
    // wiring up the DSP graph for a freshly-opened snd_pcm_t lazily —
    // the first writei into the stream pays for graph construction,
    // node activation, and reservation of the period grid. Without
    // priming, the very first guest period lands in a partially-set-
    // up sink and the leading 5-20 ms is dropped or fed into a
    // not-yet-running mixer, producing the audible "ta" transient at
    // stream start that users have reported.
    //
    // Two periods is the minimum that reliably absorbs the wake-up
    // cost across PipeWire and PulseAudio; one period works on
    // PipeWire alone, three is overkill and adds startup latency.
    int16_t silence[960 * 2 /* periods × period_frames × 1 channel */] = {0};
    snd_pcm_writei(pcm_device, silence, sizeof(silence) / sizeof(int16_t));

    subsystem->pcm_playback = pcm_device;
    snd_pcm_hw_params_free(params);

    // Capture path. Open the host's default capture source (PipeWire's
    // default source on modern desktops, the system mic / headset / etc.
    // depending on user routing). Capture is best-effort: if it fails to
    // open we skip mic support and keep playback functional, since not
    // every host has a usable mic and we don't want to break HDA over
    // it.
    if (snd_pcm_open(&subsystem->pcm_capture, "default", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        rvvm_warn("Failed to open ALSA capture device — microphone disabled");
        subsystem->pcm_capture = NULL;
    } else {
        snd_pcm_hw_params_t *cap_params = NULL;
        unsigned int cap_rate = 48000;
        unsigned int cap_channels = 1;            // mono only for now
        snd_pcm_uframes_t cap_period = 480;       // 10 ms
        snd_pcm_uframes_t cap_buffer = 1920;      // 40 ms — small for low latency
        int cap_dir = 0;

        snd_pcm_hw_params_malloc(&cap_params);
        snd_pcm_hw_params_any(subsystem->pcm_capture, cap_params);
        snd_pcm_hw_params_set_access(subsystem->pcm_capture, cap_params,
                                     SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(subsystem->pcm_capture, cap_params,
                                     SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(subsystem->pcm_capture, cap_params, cap_channels);
        snd_pcm_hw_params_set_rate_near(subsystem->pcm_capture, cap_params,
                                        &cap_rate, &cap_dir);
        snd_pcm_hw_params_set_period_size_near(subsystem->pcm_capture, cap_params,
                                               &cap_period, &cap_dir);
        snd_pcm_hw_params_set_buffer_size_near(subsystem->pcm_capture, cap_params,
                                               &cap_buffer);
        snd_pcm_hw_params(subsystem->pcm_capture, cap_params);
        snd_pcm_hw_params_free(cap_params);

        // ALSA capture opens in "prepared" state, not "running". Without
        // an explicit start(), readi() blocks indefinitely waiting for a
        // stream that's never going to produce frames on its own.
        if (snd_pcm_start(subsystem->pcm_capture) < 0) {
            rvvm_warn("Failed to start ALSA capture stream");
            snd_pcm_close(subsystem->pcm_capture);
            subsystem->pcm_capture = NULL;
        } else {
            subsystem->capture_thread = rvvm_thread_create(alsa_capture_pump, subsystem);
        }
    }

    sound->sound_data = subsystem;
    sound->write = alsa_sound_write;
    sound->read  = alsa_sound_read;
    sound->abort = alsa_sound_abort;

    return true;
}

#endif /* USE_ALSA */
