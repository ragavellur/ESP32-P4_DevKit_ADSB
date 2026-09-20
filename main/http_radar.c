#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "adsb_state.h"
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
    if (nvs_get_i32(h, "lat", &lat) == ESP_OK && lat != 0)
        s_receiver_lat = lat / 1e7;
    if (nvs_get_i32(h, "lon", &lon) == ESP_OK && lon != 0)
        s_receiver_lon = lon / 1e7;
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
        "\"uptime_ms\":%llu,"
        "\"receiver\":{\"lat\":%.6f,\"lon\":%.6f}}",
        (unsigned long long)(esp_timer_get_time() / 1000),
        phase, dev, dec,
        (unsigned long)st.effective_sps, (unsigned long)st.programmed_sps,
        (unsigned long)st.overrun_cnt, (unsigned long)st.consumer_drops,
        (unsigned long)st.ring_free_slots,
        (unsigned long)st.crc_ok_cnt, (unsigned long)st.track_count,
        (unsigned long long)(esp_timer_get_time() / 1000),
        s_receiver_lat, s_receiver_lon);
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

esp_err_t http_radar_start(void)
{
    nvs_load_receiver();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;

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
    if (httpd_register_uri_handler(server, &r_radar)  != ESP_OK ||
        httpd_register_uri_handler(server, &r_root)   != ESP_OK ||
        httpd_register_uri_handler(server, &r_feed)   != ESP_OK ||
        httpd_register_uri_handler(server, &r_status) != ESP_OK) {
        ESP_LOGE(TAG, "uri registration failed");
        httpd_stop(server);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "server up: / /radar /data/aircraft.json /api/status");
    return ESP_OK;
}