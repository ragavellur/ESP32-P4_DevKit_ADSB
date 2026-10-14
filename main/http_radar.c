#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "adsb_decoder_bridge.h"
#include "adsb_state.h"
#include "esp_rtl_sdr.h"
#include "http_radar.h"
#include "rtl_pipeline.h"

static const char *TAG = "http_radar";

/* Embedded resource (main/radar_data/radar.html, via EMBED_TXTFILES). */
extern const char radar_html_start[] asm("_binary_radar_html_start");
extern const char radar_html_end[]   asm("_binary_radar_html_end");

/* Receiver coordinates from NVS ("radar" ns, "lat"/"lon" i32 x 1e7), default Pune. */
#define RECEIVER_LAT_DEFAULT 18.480718
#define RECEIVER_LON_DEFAULT 73.898235

static double s_receiver_lat = RECEIVER_LAT_DEFAULT;
static double s_receiver_lon = RECEIVER_LON_DEFAULT;

/* JSON working buffer in PSRAM (64 tracks x ~180 bytes, plus head). */
#define JSON_BUF_CAP (64 * 200)

static void nvs_load_receiver(void)
{
    nvs_handle_t h;
    int32_t lat = 0, lon = 0;
    if (nvs_open("radar", NVS_READONLY, &h) != ESP_OK) return;
    /* Accept any stored value including 0.0 (equator/prime meridian). */
    if (nvs_get_i32(h, "lat", &lat) == ESP_OK &&
        nvs_get_i32(h, "lon", &lon) == ESP_OK) {
        s_receiver_lat = lat / 1e7;
        s_receiver_lon = lon / 1e7;
    }
    nvs_close(h);
}

/* store receiver coords so /api/status stays correct across reboots */
void http_radar_set_receiver(double lat, double lon)
{
    s_receiver_lat = lat;
    s_receiver_lon = lon;
    nvs_handle_t h;
    if (nvs_open("radar", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "lat", (int32_t)(lat * 1e7));
    nvs_set_i32(h, "lon", (int32_t)(lon * 1e7));
    nvs_commit(h);
    nvs_close(h);
}

void http_radar_get_receiver(double *lat, double *lon)
{
    if (lat) *lat = s_receiver_lat;
    if (lon) *lon = s_receiver_lon;
}

/* Keep dump1090-ish shape so existing clients can parse it. */
static void put_aircraft_json(char *buf, size_t cap, uint32_t *out_count)
{
    adsb_aircraft_t tracks[ADS_STATE_MAX_TRACKS];
    const uint32_t n = rtl_pipeline_fill_aircraft(tracks, ADS_STATE_MAX_TRACKS);
    rtl_pipeline_status_t st;
    rtl_pipeline_get_status(&st);

    int off = snprintf(buf, cap,
        "{\"now\":%llu.0,\"receiver\":{\"lat\":%.6f,\"lon\":%.6f},"
        "\"messages\":\"%lu\",\"tracks\":\"%lu\","
        "\"aircraft\":[",
        (unsigned long long)(esp_timer_get_time() / 1000),
        s_receiver_lat, s_receiver_lon,
        (unsigned long)st.crc_ok_cnt, (unsigned long)n);

    for (uint32_t i = 0; i < n && off > 0 && off < (int)cap; ++i) {
        const adsb_aircraft_t *t = &tracks[i];
        int w = snprintf(buf + off, cap - (size_t)off,
            "%s{\"hex\":\"%06lx\",\"flight\":\"%s\",\"alt_baro\":%d,"
            "\"gs\":%d,\"heading\":%d,\"baro_rate\":%d,\"signal\":%d",
            i ? "," : "",
            (unsigned long)t->icao,
            t->flight[0] ? t->flight : "",
            t->alt_baro,
            t->gs,
            t->heading,
            t->baro_rate,
            t->signal);
        off += w;
        if (w < 0 || off >= (int)cap) break;

        if (t->has_position) {
            w = snprintf(buf + off, cap - (size_t)off,
                         ",\"lat\":%.6f,\"lon\":%.6f", t->lat, t->lon);
            off += w;
            if (w < 0 || off >= (int)cap) break;
        }
        w = snprintf(buf + off, cap - (size_t)off, "}" );
        off += w;
        if (w < 0 || off >= (int)cap) break;
    }
    if (off > 0 && off < (int)cap)
        off += snprintf(buf + off, cap - (size_t)off, "]}");
    if (off < 0 || off >= (int)cap) off = (int)cap - 1;
    buf[off] = '\0';
    if (out_count) *out_count = n;
}

static void put_status_json(char *buf, size_t cap)
{
    rtl_pipeline_status_t st;
    rtl_pipeline_get_status(&st);
    const char *phase = "idle";
    switch (st.phase) {
    case RTL_PIPELINE_OPTAINING:  phase = "optaining";   break;
    case RTL_PIPELINE_STREAMING:  phase = "streaming";   break;
    case RTL_PIPELINE_DEVICE_LOST:phase = "device-lost"; break;
    case RTL_PIPELINE_DEFUNCT:    phase = "defunct";     break;
    default: break;
    }
    const char *dev = st.device_present ? "true" : "false";
    const char *dec = st.decode_ok ? "true" : "false";
    snprintf(buf, cap,
        "{\"now\":%llu,\"phase\":\"%s\",\"device_present\":%s,"
        "\"decode_ok\":%s,\"effective_sps\":\"%lu\",\"programmed_sps\":\"%lu\","
        "\"overruns\":\"%lu\",\"drops\":\"%lu\",\"ring_free\":\"%lu\","
        "\"crc_ok\":\"%lu\",\"tracks\":\"%lu\","
        "\"gain_tenth_db\":%d,\"gain_mode\":%u,\"rf_state\":%u,"
        "\"sample_min\":%d,\"sample_max\":%d,"
        "\"uptime_ms\":%llu,"
        "\"receiver\":{\"lat\":%.6f,\"lon\":%.6f}}",
        (unsigned long long)(esp_timer_get_time() / 1000),
        phase, dev, dec,
        (unsigned long)st.effective_sps, (unsigned long)st.programmed_sps,
        (unsigned long)st.overrun_cnt, (unsigned long)st.consumer_drops,
        (unsigned long)st.ring_free_slots,
        (unsigned long)st.crc_ok_cnt, (unsigned long)st.track_count,
        (int)st.gain_tenth_db, (unsigned)st.gain_mode, (unsigned)st.rf_state,
        (int)st.sample_min, (int)st.sample_max,
        (unsigned long long)(esp_timer_get_time() / 1000),
        s_receiver_lat, s_receiver_lon);
}

/* Rich decode diagnostics: everything needed to judge reception health from
 * the browser alone, so the device can run headless (no USB serial). */
static esp_err_t h_debug(httpd_req_t *req)
{
    rtl_pipeline_debug_t d;
    rtl_pipeline_get_debug(&d);
    rtl_pipeline_status_t st;
    rtl_pipeline_get_status(&st);

    /* Convenience derived rates (per second) from the 60 s window. */
    const float win_crc_ps  = d.win60_seconds ? ((float)d.win60_crc / (float)d.win60_seconds) : 0.0f;
    const float win_pre_ps  = d.win60_seconds ? ((float)d.win60_preamble / (float)d.win60_seconds) : 0.0f;
    const float win_bytes_ps= d.win60_seconds ? ((float)d.win60_bytes / (float)d.win60_seconds) : 0.0f;
    /* Preamble -> CRC yield tells you whether the receiver sees real signal. */
    const float crc_yield   = d.win60_preamble ? ((float)d.win60_crc * 100.0f / (float)d.win60_preamble) : 0.0f;

    char *buf = heap_caps_malloc(JSON_BUF_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }
    snprintf(buf, JSON_BUF_CAP,
        "{\"now\":%llu,\"phase\":\"%s\",\"device_present\":%s,\"decode_ok\":%s,"
        "\"uptime_ms\":%llu,"
        "\"totals\":{\"preambles\":%lu,\"frames\":%lu,\"crc_ok\":%lu,"
        "\"df17\":%lu,\"iq_bytes\":%lu,\"tracks\":%lu},"
        "\"window60\":{\"seconds\":%lu,\"preambles\":%lu,\"frames\":%lu,"
        "\"crc_ok\":%lu,\"df17\":%lu,\"iq_bytes\":%lu,"
        "\"preamble_per_s\":%.2f,\"crc_per_s\":%.2f,\"iq_bytes_per_s\":%.0f,"
        "\"crc_yield_pct\":%.2f},"
        "\"config\":{\"programmed_sps\":%lu,\"preamble_threshold\":%lu,"
        "\"gain_tenth_db\":%d,\"gain_mode\":%u,\"dc_filter\":%s,\"crc_fix\":%s,"
        "\"effective_sps\":%lu},"
        "\"signal\":{\"mag_min\":%u,\"mag_max\":%u,\"sample_min\":%u,"
        "\"sample_max\":%u},"
        "\"health\":{\"usb\":%u,\"rf\":%u,\"overall\":%u,"
        "\"efficiency\":%.3f,\"advice\":\"%s\"},"
        "\"pipe\":{\"overruns\":%lu,\"drops\":%lu,\"ring_free\":%lu}}",
        (unsigned long long)(esp_timer_get_time() / 1000),
        st.phase == RTL_PIPELINE_STREAMING ? "streaming" :
        st.phase == RTL_PIPELINE_OPTAINING ? "obtaining" :
        st.phase == RTL_PIPELINE_DEVICE_LOST ? "device-lost" :
        st.phase == RTL_PIPELINE_DEFUNCT ? "defunct" : "idle",
        st.device_present ? "true" : "false",
        st.decode_ok ? "true" : "false",
        (unsigned long long)(esp_timer_get_time() / 1000),
        (unsigned long)d.preambles_total, (unsigned long)d.frames_total,
        (unsigned long)d.crc_total, (unsigned long)d.df17_total,
        (unsigned long)d.bytes_total, (unsigned long)st.track_count,
        (unsigned long)d.win60_seconds, (unsigned long)d.win60_preamble,
        (unsigned long)d.win60_frames, (unsigned long)d.win60_crc,
        (unsigned long)d.win60_df17, (unsigned long)d.win60_bytes,
        win_pre_ps, win_crc_ps, win_bytes_ps, crc_yield,
        (unsigned long)d.programmed_sps, (unsigned long)d.preamble_threshold,
        (int)d.gain_tenth_db, (unsigned)d.gain_mode,
        d.dc_filter ? "true" : "false", d.crc_fix ? "true" : "false",
        (unsigned long)st.effective_sps,
        (unsigned)d.mag_min, (unsigned)d.mag_max,
        (unsigned)d.sample_min, (unsigned)d.sample_max,
        (unsigned)d.health_usb, (unsigned)d.health_rf, (unsigned)d.health_overall,
        (double)d.health_efficiency, d.advice,
        (unsigned long)st.overrun_cnt, (unsigned long)st.consumer_drops,
        (unsigned long)st.ring_free_slots);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_send(req, buf, strlen(buf));
    heap_caps_free(buf);
    return ret;
}

static esp_err_t h_radar(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const size_t len = (size_t)(radar_html_end - radar_html_start);
    return httpd_resp_send(req, radar_html_start, len);
}

static esp_err_t h_aircraft_json(httpd_req_t *req)
{
    char *buf = heap_caps_malloc(JSON_BUF_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "out of memory");
        return ESP_FAIL;
    }
    uint32_t count = 0;
    put_aircraft_json(buf, JSON_BUF_CAP, &count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t e = httpd_resp_send(req, buf, strlen(buf));
    heap_caps_free(buf);
    return e;
}

static esp_err_t h_status_json(httpd_req_t *req)
{
    char buf[768];
    put_status_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ----- Receiver location API ----- */

static esp_err_t h_location_get(httpd_req_t *req)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"lat\":%.6f,\"lon\":%.6f}",
             s_receiver_lat, s_receiver_lon);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

static esp_err_t h_location_post(httpd_req_t *req)
{
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > 256) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content length");
        return ESP_FAIL;
    }

    char buf[257];
    int received = httpd_req_recv(req, buf, content_len);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "failed to read body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    char *p = strstr(buf, "\"lat\"");
    char *q = strstr(buf, "\"lon\"");
    if (!p || !q) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "lat and lon required");
        return ESP_FAIL;
    }
    p = strchr(p, ':');
    q = strchr(q, ':');
    if (!p || !q) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "malformed lat/lon");
        return ESP_FAIL;
    }

    double lat = strtod(p + 1, NULL);
    double lon = strtod(q + 1, NULL);
    if (!isfinite(lat) || !isfinite(lon) ||
        lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "lat must be -90..90, lon -180..180");
        return ESP_FAIL;
    }

    http_radar_set_receiver(lat, lon);
    ESP_LOGI(TAG, "receiver location set: %.6f, %.6f", lat, lon);

    char resp[96];
    snprintf(resp, sizeof(resp), "{\"lat\":%.6f,\"lon\":%.6f}", lat, lon);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, strlen(resp));
}

/* ----- Settings API ----- */

#define SETTINGS_NS "radar"
#define SETTINGS_KEY_GAIN "gain_tenth_db"
#define SETTINGS_KEY_DC_FILTER "dc_filter"
#define SETTINGS_KEY_ADAPTIVE_GAIN "adaptive_gain"
#define SETTINGS_KEY_GAIN_MODE "gain_mode"
#define SETTINGS_KEY_AGGRESSIVE "aggressive"
#define SETTINGS_KEY_CRC_FIX "crc_fix"
#define SETTINGS_KEY_SAMPLE_RATE "sample_rate_sps"

#define DEFAULT_GAIN_TENTH_DB 496
#define DEFAULT_DC_FILTER false
#define DEFAULT_ADAPTIVE_GAIN false
#define DEFAULT_GAIN_MODE 0  /* 0=manual, 1=adaptive */
#define DEFAULT_AGGRESSIVE false
#define DEFAULT_CRC_FIX false
#define DEFAULT_SAMPLE_RATE_SPS 2400000u

static void settings_json_response(char *buf, size_t cap,
                                   int gain, bool dc_filter, bool adaptive_gain, int gain_mode, bool aggressive, bool crc_fix, uint32_t sample_rate_sps)
{
    const char *mode_str = (gain_mode == 1) ? "adaptive" : "manual";
    snprintf(buf, cap,
        "{"
        "\"gain\":%d,"
        "\"dc_filter\":%s,"
        "\"adaptive_gain\":%s,"
        "\"gain_mode\":\"%s\","
        "\"aggressive\":%s,"
        "\"crc_fix\":%s,"
        "\"sample_rate_sps\":%" PRIu32 ","
        "\"defaults\":{"
            "\"gain\":%d,"
            "\"dc_filter\":%s,"
            "\"adaptive_gain\":%s,"
            "\"gain_mode\":\"%s\","
            "\"aggressive\":%s,"
            "\"crc_fix\":%s,"
            "\"sample_rate_sps\":%" PRIu32
        "}"
        "}",
        gain,
        dc_filter ? "true" : "false",
        adaptive_gain ? "true" : "false",
        mode_str,
        aggressive ? "true" : "false",
        crc_fix ? "true" : "false",
        (unsigned long)sample_rate_sps,
        DEFAULT_GAIN_TENTH_DB,
        DEFAULT_DC_FILTER ? "true" : "false",
        DEFAULT_ADAPTIVE_GAIN ? "true" : "false",
        (DEFAULT_GAIN_MODE == 1) ? "adaptive" : "manual",
        DEFAULT_AGGRESSIVE ? "true" : "false",
        DEFAULT_CRC_FIX ? "true" : "false",
        (unsigned long)DEFAULT_SAMPLE_RATE_SPS);
}

static esp_err_t h_settings_get(httpd_req_t *req)
{
    /* Read current settings from NVS (or defaults) */
    nvs_handle_t h;
    int32_t gain = DEFAULT_GAIN_TENTH_DB;
    uint8_t dc_filter = DEFAULT_DC_FILTER;
    uint8_t adaptive_gain = DEFAULT_ADAPTIVE_GAIN;
    uint8_t gain_mode = DEFAULT_GAIN_MODE;
    uint8_t aggressive = DEFAULT_AGGRESSIVE;
    uint8_t crc_fix = DEFAULT_CRC_FIX;
    uint32_t sample_rate_sps = DEFAULT_SAMPLE_RATE_SPS;

    if (nvs_open(SETTINGS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, SETTINGS_KEY_GAIN, &gain);
        nvs_get_u8(h, SETTINGS_KEY_DC_FILTER, &dc_filter);
        nvs_get_u8(h, SETTINGS_KEY_ADAPTIVE_GAIN, &adaptive_gain);
        nvs_get_u8(h, SETTINGS_KEY_GAIN_MODE, &gain_mode);
        nvs_get_u8(h, SETTINGS_KEY_AGGRESSIVE, &aggressive);
        nvs_get_u8(h, SETTINGS_KEY_CRC_FIX, &crc_fix);
        nvs_get_u32(h, SETTINGS_KEY_SAMPLE_RATE, &sample_rate_sps);
        nvs_close(h);
    }

    char buf[512];
    settings_json_response(buf, sizeof(buf), gain, dc_filter != 0, adaptive_gain != 0, gain_mode, aggressive != 0, crc_fix != 0, sample_rate_sps);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* Extract the scalar value for a JSON key, bounded to that value only.
 * Copies the value token into out with surrounding spaces and any trailing
 * ',', '}', ']' stripped. Returns true if the key was found.
 *
 * This avoids the classic bug where a substring search for "true"/"false"
 * spills past the field and matches a *different* key's value. */
static bool json_get_value(const char *buf, const char *key, char *out, size_t out_sz)
{
    if (!buf || !key || !out || out_sz == 0) return false;

    char pattern[64];
    int n = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pattern)) return false;

    const char *p = strstr(buf, pattern);
    if (!p) return false;

    p = strchr(p + n, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;

    size_t i = 0;
    while (*p && *p != ',' && *p != '}' && *p != ']' && i + 1 < out_sz) {
        out[i++] = *p++;
    }
    if (i == 0) return false;
    while (i > 0 && (out[i - 1] == ' ' || out[i - 1] == '\t')) i--;
    out[i] = '\0';
    return i > 0;
}

static esp_err_t h_settings_post(httpd_req_t *req)
{
    /* Parse JSON body */
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content length");
        return ESP_FAIL;
    }

    char *buf = malloc(content_len + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, buf, content_len);
    if (received <= 0) {
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "failed to read body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    /* Parse our known fields with value-bounded lookup (see json_get_value). */
    int gain = -1;
    int dc_filter = -1;
    int adaptive_gain = -1;
    int gain_mode = -1;
    int aggressive = -1;
    int crc_fix = -1;
    int sample_rate_sps = -1;

    char val[64];

    if (json_get_value(buf, "gain", val, sizeof(val))) {
        gain = atoi(val);
    }
    if (json_get_value(buf, "dc_filter", val, sizeof(val))) {
        if (strcmp(val, "true") == 0) dc_filter = 1;
        else if (strcmp(val, "false") == 0) dc_filter = 0;
    }
    if (json_get_value(buf, "adaptive_gain", val, sizeof(val))) {
        if (strcmp(val, "true") == 0) adaptive_gain = 1;
        else if (strcmp(val, "false") == 0) adaptive_gain = 0;
    }
    if (json_get_value(buf, "gain_mode", val, sizeof(val))) {
        if (strcmp(val, "adaptive") == 0) gain_mode = 1;
        else if (strcmp(val, "manual") == 0) gain_mode = 0;
        else gain_mode = (atoi(val) != 0) ? 1 : 0;
    }
    if (json_get_value(buf, "aggressive", val, sizeof(val))) {
        if (strcmp(val, "true") == 0) aggressive = 1;
        else if (strcmp(val, "false") == 0) aggressive = 0;
    }
    if (json_get_value(buf, "crc_fix", val, sizeof(val))) {
        if (strcmp(val, "true") == 0) crc_fix = 1;
        else if (strcmp(val, "false") == 0) crc_fix = 0;
    }
    if (json_get_value(buf, "sample_rate_sps", val, sizeof(val))) {
        sample_rate_sps = atoi(val);
    }
    free(buf);

    /* Validate */
    if (gain != -1 && (gain < 0 || gain > 496)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "gain must be 0-496");
        return ESP_FAIL;
    }
    if (gain_mode != -1 && (gain_mode < 0 || gain_mode > 1)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "gain_mode must be 0 (manual) or 1 (adaptive)");
        return ESP_FAIL;
    }
    if (sample_rate_sps != -1 && sample_rate_sps != 2400000) {
        /* 2.048 MSPS is rejected: verified on this R820T2 it decodes 0
         * messages on any decoder (readsb included); only 2.4 MSPS works. */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "sample_rate_sps must be 2400000 (2.048 MSPS does not decode ADS-B)");
        return ESP_FAIL;
    }

    /* Apply and save */
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NS, NVS_READWRITE, &h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS open failed");
        return ESP_FAIL;
    }

    if (gain != -1) {
        nvs_set_i32(h, SETTINGS_KEY_GAIN, gain);
    }
    if (dc_filter != -1) {
        nvs_set_u8(h, SETTINGS_KEY_DC_FILTER, dc_filter);
    }
    if (adaptive_gain != -1) {
        nvs_set_u8(h, SETTINGS_KEY_ADAPTIVE_GAIN, adaptive_gain);
    }
    if (gain_mode != -1) {
        nvs_set_u8(h, SETTINGS_KEY_GAIN_MODE, gain_mode);
    }
    if (aggressive != -1) {
        nvs_set_u8(h, SETTINGS_KEY_AGGRESSIVE, aggressive);
    }
    if (crc_fix != -1) {
        nvs_set_u8(h, SETTINGS_KEY_CRC_FIX, crc_fix);
    }
    if (sample_rate_sps != -1) {
        nvs_set_u32(h, SETTINGS_KEY_SAMPLE_RATE, (uint32_t)sample_rate_sps);
    }
    nvs_commit(h);
    nvs_close(h);

    /* Update in-memory settings via pipeline API */
    int cur_gain;
    bool cur_adaptive;
    bool cur_dc, cur_aggressive, cur_crc_fix;
    gain_mode_t cur_mode;
    uint32_t cur_sample_rate;
    extern void rtl_pipeline_get_settings(int*, bool*, bool*, gain_mode_t*, bool*, bool*, uint32_t*);
    rtl_pipeline_get_settings(&cur_gain, &cur_dc, &cur_adaptive, &cur_mode, &cur_aggressive, &cur_crc_fix, &cur_sample_rate);

    int new_gain = (gain != -1) ? gain : cur_gain;
    bool new_dc_filter = (dc_filter != -1) ? dc_filter : cur_dc;
    bool new_adaptive_gain = (adaptive_gain != -1) ? adaptive_gain : cur_adaptive;
    gain_mode_t new_gain_mode = (gain_mode != -1) ? (gain_mode_t)gain_mode : cur_mode;
    bool new_aggressive = (aggressive != -1) ? aggressive : cur_aggressive;
    bool new_crc_fix = (crc_fix != -1) ? crc_fix : cur_crc_fix;
    uint32_t new_sample_rate_sps = (sample_rate_sps != -1) ? (uint32_t)sample_rate_sps : cur_sample_rate;

    extern void rtl_pipeline_update_settings(int, bool, bool, gain_mode_t, bool, bool, uint32_t);
    rtl_pipeline_update_settings(new_gain, new_dc_filter, new_adaptive_gain,
                                  new_gain_mode, new_aggressive, new_crc_fix, new_sample_rate_sps);

    /* Apply settings immediately via pipeline API */
    extern esp_err_t rtl_pipeline_apply_settings(void);
    esp_err_t ret = rtl_pipeline_apply_settings();
    if (ret == ESP_ERR_INVALID_STATE) {
        /* Sample rate changed - need to restart pipeline */
        ESP_LOGI(TAG, "Sample rate changed, restarting pipeline...");
        extern void rtl_pipeline_restart(void);
        rtl_pipeline_restart();
    }

    /* Return updated settings */
    char resp[512];
    int resp_gain = (gain != -1) ? gain : DEFAULT_GAIN_TENTH_DB;
    bool resp_dc_filter = (dc_filter != -1) ? dc_filter : DEFAULT_DC_FILTER;
    bool resp_adaptive_gain = (adaptive_gain != -1) ? adaptive_gain : DEFAULT_ADAPTIVE_GAIN;
    int resp_mode = (gain_mode != -1) ? gain_mode : DEFAULT_GAIN_MODE;
    bool resp_aggressive = (aggressive != -1) ? aggressive : DEFAULT_AGGRESSIVE;
    bool resp_crc_fix = (crc_fix != -1) ? crc_fix : DEFAULT_CRC_FIX;
    uint32_t resp_sample_rate_sps = (sample_rate_sps != -1) ? (uint32_t)sample_rate_sps : DEFAULT_SAMPLE_RATE_SPS;
    settings_json_response(resp, sizeof(resp), resp_gain, resp_dc_filter, resp_adaptive_gain, resp_mode, resp_aggressive, resp_crc_fix, resp_sample_rate_sps);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t h_settings_reset(httpd_req_t *req)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NS, NVS_READWRITE, &h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS open failed");
        return ESP_FAIL;
    }
    nvs_erase_key(h, SETTINGS_KEY_GAIN);
    nvs_erase_key(h, SETTINGS_KEY_DC_FILTER);
    nvs_erase_key(h, SETTINGS_KEY_ADAPTIVE_GAIN);
    nvs_erase_key(h, SETTINGS_KEY_GAIN_MODE);
    nvs_erase_key(h, SETTINGS_KEY_AGGRESSIVE);
    nvs_erase_key(h, SETTINGS_KEY_CRC_FIX);
    nvs_erase_key(h, SETTINGS_KEY_SAMPLE_RATE);
    nvs_commit(h);
    nvs_close(h);

    /* Apply defaults immediately */
    adsb_decoder_bridge_set_dc_filter(DEFAULT_DC_FILTER);
    adsb_decoder_bridge_set_aggressive(DEFAULT_AGGRESSIVE);
    adsb_decoder_bridge_set_crc_fix(DEFAULT_CRC_FIX);
    /* Gain will be picked up by status task on next cycle */

    char resp[512];
    settings_json_response(resp, sizeof(resp),
                           DEFAULT_GAIN_TENTH_DB, DEFAULT_DC_FILTER,
                           DEFAULT_ADAPTIVE_GAIN, DEFAULT_GAIN_MODE,
                           DEFAULT_AGGRESSIVE, DEFAULT_CRC_FIX, DEFAULT_SAMPLE_RATE_SPS);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, strlen(resp));
}

esp_err_t http_radar_start(void)
{
    nvs_load_receiver();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;
    cfg.core_id = 0;  /* Pin httpd to core 0 */

    httpd_handle_t server = NULL;
    esp_err_t e = httpd_start(&server, &cfg);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(e));
        return e;
    }

    const httpd_uri_t r_radar = {
        .uri = "/radar", .method = HTTP_GET, .handler = h_radar, .user_ctx = NULL
    };
    const httpd_uri_t r_root = {
        .uri = "/", .method = HTTP_GET, .handler = h_radar, .user_ctx = NULL
    };
    const httpd_uri_t r_feed = {
        .uri = "/data/aircraft.json", .method = HTTP_GET,
        .handler = h_aircraft_json, .user_ctx = NULL
    };
    const httpd_uri_t r_status = {
        .uri = "/api/status", .method = HTTP_GET,
        .handler = h_status_json, .user_ctx = NULL
    };
    const httpd_uri_t r_debug = {
        .uri = "/api/debug", .method = HTTP_GET,
        .handler = h_debug, .user_ctx = NULL
    };
    const httpd_uri_t r_loc_get = {
        .uri = "/api/location", .method = HTTP_GET,
        .handler = h_location_get, .user_ctx = NULL
    };
    const httpd_uri_t r_loc_post = {
        .uri = "/api/location", .method = HTTP_POST,
        .handler = h_location_post, .user_ctx = NULL
    };
    const httpd_uri_t r_settings_get = {
        .uri = "/api/settings", .method = HTTP_GET,
        .handler = h_settings_get, .user_ctx = NULL
    };
    const httpd_uri_t r_settings_post = {
        .uri = "/api/settings", .method = HTTP_POST,
        .handler = h_settings_post, .user_ctx = NULL
    };
    const httpd_uri_t r_settings_reset = {
        .uri = "/api/settings/reset", .method = HTTP_POST,
        .handler = h_settings_reset, .user_ctx = NULL
    };
    if (httpd_register_uri_handler(server, &r_radar)  != ESP_OK ||
        httpd_register_uri_handler(server, &r_root)   != ESP_OK ||
        httpd_register_uri_handler(server, &r_feed)   != ESP_OK ||
        httpd_register_uri_handler(server, &r_status) != ESP_OK ||
        httpd_register_uri_handler(server, &r_debug)  != ESP_OK ||
        httpd_register_uri_handler(server, &r_loc_get)  != ESP_OK ||
        httpd_register_uri_handler(server, &r_loc_post) != ESP_OK ||
        httpd_register_uri_handler(server, &r_settings_get)  != ESP_OK ||
        httpd_register_uri_handler(server, &r_settings_post) != ESP_OK ||
        httpd_register_uri_handler(server, &r_settings_reset) != ESP_OK) {
        ESP_LOGE(TAG, "uri registration failed");
        httpd_stop(server);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "server up: / /radar /data/aircraft.json /api/status /api/debug /api/settings /api/location");
    return ESP_OK;
}