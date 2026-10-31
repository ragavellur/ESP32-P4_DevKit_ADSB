#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "adsb_decoder_bridge.h"
#include "adsb_decoder.hpp"

extern "C" size_t adsb_decoder_bridge_get_overlap_samples(void);

/* Minimal single-decoder C bridge. The RTL DSP task is the only consumer
 * (single-threaded); no allocation on the hot path. */

using adsb_radar::adsb_rx::Decoder;
using adsb_radar::adsb_rx::Frame;
using adsb_radar::adsb_rx::Stats;

static const char *BRIDGE_TAG = "adsb_rx";

static Decoder s_dec;
static bool s_ready = false;
static bool s_self_pass = false;
static uint32_t s_app_frames = 0;
static uint32_t s_app_df17 = 0;
static uint32_t s_app_crc_ok = 0;
static adsb_frame_hook_t s_hook = nullptr;
static void* s_hook_ctx = nullptr;
/* Block timing context for current processing run */
static adsb_decoder_block_ctx_t s_cur_ctx{};
static bool s_has_cur_ctx = false;

/* Recent-frame capture ring.
 *
 * Lets /api/frames serve raw Mode-S frames captured from REAL device IQ, so
 * the feeder can be validated byte-for-byte against the reference Python
 * implementation (br_feeder.py / br_bridge.py) before any C code is trusted
 * against the live hub. Written on the decode hot path with no allocation. */
static constexpr size_t kCaptureDepth = 64;
static struct {
    uint8_t bytes[14];
    uint8_t bit_length;
    int64_t rx_timestamp_us;
    int64_t rx_timestamp_sample_us;
    uint64_t sample_index;
    uint32_t preamble_offset_samples;
} s_capture[kCaptureDepth];
static volatile uint32_t s_capture_head = 0;
static volatile uint32_t s_capture_total = 0;

static void on_frame(const Frame &frame, void *ctx)
{
    (void)ctx;
    ++s_app_frames;
    if (frame.df == 17 && frame.type_code >= 1 && frame.type_code <= 4) {
        ++s_app_df17;
    } else if (frame.df == 17) {
        /* keep counting DF17 messages in app stats for backward compat where possible */
        if ((frame.type_code >= 5 && frame.type_code <= 8) ||
            (frame.type_code >= 9 && frame.type_code <= 18) ||
            (frame.type_code >= 19 && frame.type_code <= 22)) {
            /* do not increment s_app_df17 for position/vel categories; keep as-is */
        }
    }
    if (frame.has_callsign || frame.has_altitude || frame.has_speed ||
        frame.has_heading || frame.has_vertical_rate) {
        ++s_app_crc_ok;
    }
    if (frame.has_callsign || frame.has_altitude || frame.has_speed ||
        frame.has_heading || frame.has_vertical_rate) {
        ESP_LOGI(BRIDGE_TAG,
                 "AIRCRAFT icao=%06X tc=%u callsign=\"%s\" alt=%dft spd=%dkts hdg=%ddeg",
                 frame.icao, (unsigned)frame.type_code,
                 frame.has_callsign ? frame.callsign : "-",
                 frame.has_altitude ? frame.altitude_ft : -9999,
                 frame.has_speed ? (int)frame.speed_kts : -1,
                 frame.has_heading ? (int)frame.heading_deg : -1);
    }
    if (s_hook) {
        adsb_frame_info_t f{};
        f.icao = frame.icao;
        f.icao_is_icao = frame.icao_is_icao;
        f.df = frame.df;
        f.cf = frame.cf;
        f.type_code = frame.type_code;
        f.has_callsign = frame.has_callsign;
        if (frame.has_callsign) {
            snprintf(f.callsign, sizeof(f.callsign), "%s", frame.callsign);
        }
        f.altitude_ft = frame.altitude_ft;
        f.has_altitude = frame.has_altitude;
        f.altitude_is_gnss = frame.altitude_is_gnss;
        f.speed_kts = frame.speed_kts;
        f.has_speed = frame.has_speed;
        f.heading_deg = frame.heading_deg;
        f.has_heading = frame.has_heading;
        f.vertical_rate_fpm = frame.vertical_rate_fpm;
        f.has_vertical_rate = frame.has_vertical_rate;
        f.cpr_latitude = frame.cpr_latitude;
        f.cpr_longitude = frame.cpr_longitude;
        f.cpr_odd = frame.cpr_odd;
        f.has_cpr = frame.has_cpr;
        f.cpr_surface = frame.cpr_surface;
        f.signal = frame.signal;
        /* Raw frame for the feeder. Frame.bytes holds exactly bit_length/8
         * bytes and is already CRC-corrected when crc_fix is on, so this is
         * what a reference decoder (readsb/dump1090) would emit. */
        memcpy(f.bytes, frame.bytes, sizeof(f.bytes));
        f.bit_length = frame.bit_length;
        f.preamble_offset_samples = frame.preamble_offset_samples;
        int64_t decode_us = (int64_t)esp_timer_get_time();
        f.rx_timestamp_us = decode_us;
        f.rx_timestamp_sample_us = 0;
        f.sample_index = 0;
        f.sample_rate_sps = 0;
        /* Sample-derived timestamp using block context + preamble offset.
         * Preamble offset is in magnitudes buffer (includes overlap from previous block).
         * We need to compute offset within current block. */
        if (s_has_cur_ctx && s_cur_ctx.sample_rate_sps > 0) {
            f.sample_index = s_cur_ctx.block_start_sample;
            f.sample_rate_sps = s_cur_ctx.sample_rate_sps;
            size_t ovl = adsb_decoder_bridge_get_overlap_samples();
            uint32_t po_adj = f.preamble_offset_samples;
            if (ovl > 0 && po_adj >= (uint32_t)ovl) {
                po_adj = f.preamble_offset_samples - (uint32_t)ovl;
            } else {
                po_adj = 0;
            }
            f.preamble_offset_samples = po_adj;
            f.rx_timestamp_sample_us = ((int64_t)(f.sample_index + (uint64_t)f.preamble_offset_samples) * 1000000LL) /
                                       (int64_t)f.sample_rate_sps;
        } else if (s_has_cur_ctx && s_cur_ctx.block_time_us > 0) {
            f.rx_timestamp_sample_us = s_cur_ctx.block_time_us;
            f.sample_index = s_cur_ctx.block_start_sample;
            f.sample_rate_sps = s_cur_ctx.sample_rate_sps;
        }
        {
            const uint32_t slot = s_capture_head % kCaptureDepth;
            memcpy(s_capture[slot].bytes, frame.bytes, sizeof(f.bytes));
            s_capture[slot].bit_length = frame.bit_length;
            s_capture[slot].rx_timestamp_us = f.rx_timestamp_us;
            s_capture[slot].rx_timestamp_sample_us = f.rx_timestamp_sample_us;
            s_capture[slot].sample_index = f.sample_index;
            s_capture[slot].preamble_offset_samples = f.preamble_offset_samples;
            s_capture_head = slot + 1;
            s_capture_total = s_capture_total + 1;
        }
        s_hook(&f, s_hook_ctx);
    }
}

static void pump_stats(adsb_decoder_bridge_stats_t *out)
{
    const Stats &s = s_dec.stats();
    out->decoder_preambles = s.preambles;
    out->decoder_frames = s.frames;
    out->decoder_df17 = s.df17;
    out->decoder_df18 = s.df18;
    out->decoder_df11 = s.df11;
    out->decoder_crc_ok = s.crc_ok;
    out->decoder_magnitude_min = s.magnitude_min;
    out->decoder_magnitude_max = s.magnitude_max;
    out->app_frames = s_app_frames;
    out->app_df17 = s_app_df17;
    out->app_crc_ok = s_app_crc_ok;
}

bool adsb_decoder_bridge_init(void)
{
    if (s_ready) {
        return s_self_pass;
    }
    s_self_pass = Decoder::self_check();
    s_dec.reset();
    if (!s_self_pass) {
        return false;
    }
    s_ready = true;
    return true;
}

void adsb_decoder_bridge_reset(void)
{
    s_dec.reset();
    s_app_frames = 0;
    s_app_df17 = 0;
    s_app_crc_ok = 0;
    s_capture_head = 0;
    s_capture_total = 0;
    memset(s_capture, 0, sizeof(s_capture));
}

uint32_t adsb_decoder_bridge_get_capture(adsb_captured_frame_t *out, uint32_t cap)
{
    if (out == nullptr || cap == 0) {
        return 0;
    }
    /* Snapshot the head once so a concurrent push cannot make us read a torn
     * slot; entries older than the newest `cap` are simply skipped. */
    const uint32_t head = s_capture_head;
    const uint32_t total = s_capture_total;
    const uint32_t available = (total < kCaptureDepth) ? total : (uint32_t)kCaptureDepth;
    uint32_t n = 0;
    for (uint32_t i = 0; i < available && n < cap; ++i) {
        /* Walk oldest -> newest. */
        const uint32_t idx = (head - available + i) % kCaptureDepth;
        out[n].bit_length = s_capture[idx].bit_length;
        out[n].rx_timestamp_us = s_capture[idx].rx_timestamp_us;
        out[n].rx_timestamp_sample_us = s_capture[idx].rx_timestamp_sample_us;
        out[n].sample_index = s_capture[idx].sample_index;
        out[n].preamble_offset_samples = s_capture[idx].preamble_offset_samples;
        memcpy(out[n].bytes, s_capture[idx].bytes, sizeof(out[n].bytes));
        ++n;
    }
    return n;
}

void adsb_decoder_bridge_process(const uint8_t *cu8, size_t bytes)
{
    if (!cu8 || bytes == 0 || !s_ready || !s_self_pass) {
        return;
    }
    s_has_cur_ctx = false;
    s_dec.process_cu8(cu8, bytes, on_frame, nullptr);
    s_has_cur_ctx = false;
}

void adsb_decoder_bridge_process_with_ctx(const uint8_t *cu8, size_t bytes,
                                          const adsb_decoder_block_ctx_t* ctx)
{
    if (!cu8 || bytes == 0 || !s_ready || !s_self_pass) {
        return;
    }
    if (ctx) {
        s_cur_ctx = *ctx;
        s_has_cur_ctx = true;
    } else {
        s_has_cur_ctx = false;
    }
    s_dec.process_cu8(cu8, bytes, on_frame, nullptr);
    s_has_cur_ctx = false;
}

void adsb_decoder_bridge_get_stats(adsb_decoder_bridge_stats_t *out)
{
    if (!out) {
        return;
    }
    pump_stats(out);
}

adsb_frame_hook_t adsb_decoder_bridge_set_frame_hook(adsb_frame_hook_t hook,
                                                     void *ctx)
{
    adsb_frame_hook_t prev = s_hook;
    s_hook = hook;
    s_hook_ctx = ctx;
    return prev;
}

bool adsb_decoder_bridge_cpr_pair(const adsb_cpr_sample_t *even,
                                  const adsb_cpr_sample_t *odd,
                                  bool use_odd, double *latitude,
                                  double *longitude)
{
    if (!even || !odd || !latitude || !longitude) {
        return false;
    }
    Frame a{};
    Frame b{};
    a.icao = b.icao = 0;
    a.cpr_latitude = even->latitude;
    a.cpr_longitude = even->longitude;
    a.cpr_odd = false;
    a.has_cpr = true;
    b.cpr_latitude = odd->latitude;
    b.cpr_longitude = odd->longitude;
    b.cpr_odd = true;
    b.has_cpr = true;
    return decode_global_cpr(a, b, use_odd, latitude, longitude);
}

void adsb_decoder_bridge_set_dc_filter(bool enable)
{
    s_dec.set_dc_filter(enable);
}

void adsb_decoder_bridge_set_aggressive(bool enable)
{
    s_dec.set_aggressive(enable);
}

void adsb_decoder_bridge_set_crc_fix(bool enable)
{
    s_dec.set_crc_fix(enable);
}

extern "C" size_t adsb_decoder_bridge_get_overlap_samples(void)
{
    return s_dec.overlap_samples();
}

void adsb_decoder_bridge_set_sample_rate(uint32_t sample_rate_sps)
{
    s_dec.set_sample_rate(sample_rate_sps);
}

void adsb_decoder_bridge_set_preamble_threshold(uint32_t threshold)
{
    s_dec.set_preamble_threshold(threshold);
}
