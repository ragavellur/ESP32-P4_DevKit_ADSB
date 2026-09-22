#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "adsb_decoder_bridge.h"
#include "adsb_decoder.hpp"

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

static void on_frame(const Frame &frame, void *ctx)
{
    (void)ctx;
    ++s_app_frames;
    if (frame.type_code >= 1 && frame.type_code <= 4) {
        ++s_app_df17;
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
        f.type_code = frame.type_code;
        f.has_callsign = frame.has_callsign;
        if (frame.has_callsign) {
            snprintf(f.callsign, sizeof(f.callsign), "%s", frame.callsign);
        }
        f.altitude_ft = frame.altitude_ft;
        f.has_altitude = frame.has_altitude;
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
        f.signal = frame.signal;
        s_hook(&f, s_hook_ctx);
    }
}

static void pump_stats(adsb_decoder_bridge_stats_t *out)
{
    const Stats &s = s_dec.stats();
    out->decoder_preambles = s.preambles;
    out->decoder_frames = s.frames;
    out->decoder_df17 = s.df17;
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
}

void adsb_decoder_bridge_process(const uint8_t *cu8, size_t bytes)
{
    if (!cu8 || bytes == 0 || !s_ready || !s_self_pass) {
        return;
    }
    s_dec.process_cu8(cu8, bytes, on_frame, nullptr);
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
