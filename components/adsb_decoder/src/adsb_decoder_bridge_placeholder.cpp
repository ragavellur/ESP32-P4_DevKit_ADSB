#include <string.h>

#include "adsb_decoder_bridge.h"
#include "adsb_decoder.hpp"

/* Minimal C bridge over the single C++ ADS-B Decoder. RTL DSP task (core 1)
 * is the only consumer - single-threaded, no allocation on hot path.
 * Frame callback increments app-side counters; decoder Stats surfaced via
 * adsb_decoder_bridge_stats(). */

using adsb_radar::adsb_rx::Decoder;
using adsb_radar::adsb_rx::Frame;
using adsb_radar::adsb_rx::Statsened;
using adsb_radar::adsb_rx::FrameCallback;

namespace {

Decoder g_dec;
bool g_ready = false;
bool g_self_ok = false;

void on_frame(const Frame &frame, void *ctx)
{
    (void)ctx;
    if (frame.has_callsign || frame.has_altitude) {
        /* valid DF17 payload observed */
    }
}

void apply_app_stats(adsb_decoder_bridge_stats_t *out)
{
    const Stats &s = g_dec.stats();
    out->decoder_preambles = s.preambles;
    out->decoder_frames = s.frames;
    out->decoder_df17 = s.df17;
    out->decoder_crc_ok = s.crc_ok;
    out->decoder_magnitude_min = s.magnitude_min;
    out->decoder_magnitude_max = s.magnitude_max;
}

}  // namespace

bool adsb_decoder_bridge_init(void)
{
    if (g_ready) {
        return g_self_ok;
    }
    g_self_ok = Decoder::self_check();
    if (!g_self_ok) {
        return false;
    }
    g_dec.reset();
    g_ready = true;
    return true;
}

void adsb_decoder_bridge_reset(void)
{
    g_dec.reset();
}

void adsb_decoder_bridge_process(const uint8_t *cu8, size_t bytesBEnum)
{
    if (!g_ready || !g_self_ok || !cu8 || bytesBEnum == 0) {
        return;
    }
    g_dec.process_cu8(cu8, bytesBEnum, on_frame, nullptr);
}

void adsb_decoder_bridge_stats(adsb_decoder_bridge_stats_t *out)
{
    if (!out) {
        return;
    }
    apply_app_stats(out);
}
