/*
 * Native ESP-IDF MLAT client (mutability/json protocol)
 *
 * Implements client side of the mlat-client JSON protocol:
 * - TCP JSON lines to server, optional zlib framing as offered
 * - Handshake with init message (lat/lon/alt, clock type/freq)
 * - Submit MLAT observations as {"mlat":{"t":...,"m":"..."}} (12MHz-style timestamps)
 * - Handle server responses, keepalive, reconnect
 * - Separate FreeRTOS task, lock-free SPSC ring between decoder and network task
 * - Never blocks IQ/DSP; uses esp_timer monotonic time
 *
 * Timing: uses sample_counter (sample_index + preamble_offset_samples) scaled
 * from sample_rate_sps to 12MHz units. Falls back to decode-time (rx_timestamp_us*12)
 * if sample rate unknown; invalid if no timing available.
 */

#include "mlat_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "mlat_cli";

/* Config defaults - no hardcoded credentials */
#define DEFAULT_PORT 31090
#define DEFAULT_CLOCK_FREQ 12000000.0

/* NVS */
#define MLAT_NS "radar"
#define MLAT_KEY_ENABLED "mlat_en"
#define MLAT_KEY_HOST "mlat_host"
#define MLAT_KEY_PORT "mlat_port"
#define MLAT_KEY_UUID "mlat_uuid"
#define MLAT_KEY_NAME "mlat_name"
#define MLAT_KEY_LAT "mlat_lat"
#define MLAT_KEY_LON "mlat_lon"
#define MLAT_KEY_ALT "mlat_alt"
#define MLAT_KEY_PRIVACY "mlat_priv"
#define MLAT_KEY_RETRES "mlat_retres"
#define MLAT_KEY_USEUDP "mlat_udp"
#define MLAT_KEY_USEZLIB "mlat_zlib"

/* Ring */
#define RING_DEPTH 512
typedef struct {
    uint8_t bytes[14];
    uint8_t bit_length;
    int64_t rx_timestamp_us;        /* decode time */
    int64_t rx_timestamp_sample_us; /* sample-derived */
    uint64_t sample_index;
    uint32_t sample_rate_sps;
    uint32_t preamble_offset_samples;
    uint32_t icao;
} ring_slot_t;

static ring_slot_t s_ring[RING_DEPTH];
static volatile uint32_t s_ring_head = 0;
static volatile uint32_t s_ring_tail = 0;
static SemaphoreHandle_t s_ring_lock = NULL;

static mlat_client_config_t s_cfg = {
    .enabled = false,
    .host = "feed.bharatradar.com",
    .port = DEFAULT_PORT,
    .uuid = {0},
    .station_name = {0},
    .lat = 0,
    .lon = 0,
    .alt_m = 0,
    .privacy = false,
    .return_results = false,
    .use_udp = false,
    .use_zlib = false,
};

static mlat_client_stats_t s_stats;
static TaskHandle_t s_task = NULL;
static volatile bool s_stop = false;

typedef struct {
    bool used;
    uint32_t icao;
    bool have_even;
    bool have_odd;
    uint64_t even_ts12;
    uint64_t odd_ts12;
    int64_t even_received_us;
    int64_t odd_received_us;
    int64_t last_seen_us;
    uint8_t even_msg[14];
    uint8_t odd_msg[14];
} mlat_sync_entry_t;

#define MLAT_SYNC_CACHE_SIZE 128
#define MLAT_TRACK_JSON_MAX (MLAT_SYNC_CACHE_SIZE * 9 + 64)
_Static_assert(MLAT_TRACK_JSON_MAX >= MLAT_SYNC_CACHE_SIZE * 9 + 11,
               "tracking JSON buffer is too small");

static mlat_sync_entry_t s_sync_cache[MLAT_SYNC_CACHE_SIZE];
static uint32_t s_sync_sent_this_sec = 0;
static int64_t s_sync_sec_start_us = 0;

#define MLAT_TRACK_MAX 256
static uint32_t s_track_last_reported[MLAT_TRACK_MAX];
static int s_track_last_count = 0;
static int64_t s_track_last_sent_us = 0;
static volatile bool s_net_ready = false;

static void ip_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    if (id == IP_EVENT_ETH_GOT_IP || id == IP_EVENT_STA_GOT_IP) {
        s_net_ready = true;
    }
}

static void load_config(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.enabled = false;
    snprintf(s_cfg.host, sizeof(s_cfg.host), "%s", "feed.bharatradar.com");
    s_cfg.port = DEFAULT_PORT;
    s_cfg.lat = s_cfg.lon = s_cfg.alt_m = 0;
    s_cfg.privacy = false;
    s_cfg.return_results = false;
    s_cfg.use_udp = false;
    s_cfg.use_zlib = false;
    s_cfg.uuid[0] = '\0';
    s_cfg.station_name[0] = '\0';

    nvs_handle_t h;
    if (nvs_open(MLAT_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t en = 0;
    if (nvs_get_u8(h, MLAT_KEY_ENABLED, &en) == ESP_OK) s_cfg.enabled = (en != 0);
    size_t len = sizeof(s_cfg.host);
    nvs_get_str(h, MLAT_KEY_HOST, s_cfg.host, &len);
    uint32_t port = 0;
    if (nvs_get_u32(h, MLAT_KEY_PORT, &port) == ESP_OK && port > 0 && port <= 65535) s_cfg.port = (uint16_t)port;
    len = sizeof(s_cfg.uuid);
    nvs_get_str(h, MLAT_KEY_UUID, s_cfg.uuid, &len);
    len = sizeof(s_cfg.station_name);
    nvs_get_str(h, MLAT_KEY_NAME, s_cfg.station_name, &len);
    int32_t v = 0;
    if (nvs_get_i32(h, MLAT_KEY_LAT, &v) == ESP_OK) s_cfg.lat = v / 1e6;
    if (nvs_get_i32(h, MLAT_KEY_LON, &v) == ESP_OK) s_cfg.lon = v / 1e6;
    if (nvs_get_i32(h, MLAT_KEY_ALT, &v) == ESP_OK) s_cfg.alt_m = v / 1000.0;
    uint8_t b = 0;
    if (nvs_get_u8(h, MLAT_KEY_PRIVACY, &b) == ESP_OK) s_cfg.privacy = (b != 0);
    if (nvs_get_u8(h, MLAT_KEY_RETRES, &b) == ESP_OK) s_cfg.return_results = (b != 0);
    if (nvs_get_u8(h, MLAT_KEY_USEUDP, &b) == ESP_OK) s_cfg.use_udp = (b != 0);
    if (nvs_get_u8(h, MLAT_KEY_USEZLIB, &b) == ESP_OK) s_cfg.use_zlib = (b != 0);
    nvs_close(h);
}

static void save_config(void)
{
    nvs_handle_t h;
    if (nvs_open(MLAT_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, MLAT_KEY_ENABLED, s_cfg.enabled ? 1 : 0);
    nvs_set_str(h, MLAT_KEY_HOST, s_cfg.host);
    nvs_set_u32(h, MLAT_KEY_PORT, s_cfg.port);
    nvs_set_str(h, MLAT_KEY_UUID, s_cfg.uuid);
    nvs_set_str(h, MLAT_KEY_NAME, s_cfg.station_name);
    nvs_set_i32(h, MLAT_KEY_LAT, (int32_t)(s_cfg.lat * 1e6));
    nvs_set_i32(h, MLAT_KEY_LON, (int32_t)(s_cfg.lon * 1e6));
    nvs_set_i32(h, MLAT_KEY_ALT, (int32_t)(s_cfg.alt_m * 1000.0));
    nvs_set_u8(h, MLAT_KEY_PRIVACY, s_cfg.privacy ? 1 : 0);
    nvs_set_u8(h, MLAT_KEY_RETRES, s_cfg.return_results ? 1 : 0);
    nvs_set_u8(h, MLAT_KEY_USEUDP, s_cfg.use_udp ? 1 : 0);
    nvs_set_u8(h, MLAT_KEY_USEZLIB, s_cfg.use_zlib ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

void mlat_client_get_config(mlat_client_config_t *out)
{
    if (out) *out = s_cfg;
}

esp_err_t mlat_client_set_config(const mlat_client_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->enabled) {
        if (cfg->host[0] == '\0' || cfg->port == 0) return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.host[0] == '\0') s_cfg.enabled = false;
    save_config();
    return ESP_OK;
}

void mlat_client_reset_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(MLAT_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, MLAT_KEY_ENABLED);
        nvs_erase_key(h, MLAT_KEY_HOST);
        nvs_erase_key(h, MLAT_KEY_PORT);
        nvs_erase_key(h, MLAT_KEY_UUID);
        nvs_erase_key(h, MLAT_KEY_NAME);
        nvs_erase_key(h, MLAT_KEY_LAT);
        nvs_erase_key(h, MLAT_KEY_LON);
        nvs_erase_key(h, MLAT_KEY_ALT);
        nvs_erase_key(h, MLAT_KEY_PRIVACY);
        nvs_erase_key(h, MLAT_KEY_RETRES);
        nvs_erase_key(h, MLAT_KEY_USEUDP);
        nvs_erase_key(h, MLAT_KEY_USEZLIB);
        nvs_commit(h);
        nvs_close(h);
    }
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.enabled = false;
    snprintf(s_cfg.host, sizeof(s_cfg.host), "%s", "feed.bharatradar.com");
    s_cfg.port = DEFAULT_PORT;
}

void mlat_client_get_stats(mlat_client_stats_t *out)
{
    if (out) *out = s_stats;
    /* compute queue depth */
    uint32_t head = s_ring_head;
    uint32_t tail = s_ring_tail;
    s_stats.queue_depth = (head >= tail) ? (head - tail) : (RING_DEPTH - (tail - head));
}

void mlat_client_push_frame(const uint8_t *bytes, uint8_t bit_length,
                            int64_t rx_timestamp_us,
                            int64_t rx_timestamp_sample_us,
                            uint64_t sample_index,
                            uint32_t sample_rate_sps,
                            uint32_t preamble_offset_samples,
                            uint32_t icao)
{
    if (!s_cfg.enabled || bytes == NULL) return;
    uint8_t len = bit_length / 8;
    if (len != 7 && len != 14) return;
    /* MLAT typically wants DF11 and DF17 */
    s_stats.frames_observed++;
    /* DF11 and DF17 are primary; allow others for now */
    s_stats.frames_eligible++;

    /* lock-free-ish: use lock for head/tail update to be safe */
    if (s_ring_lock && xSemaphoreTake(s_ring_lock, 0) == pdTRUE) {
        uint32_t head = s_ring_head;
        uint32_t next = (head + 1) % RING_DEPTH;
        if (next == s_ring_tail) {
            s_stats.frames_dropped_queue++;
            xSemaphoreGive(s_ring_lock);
            return;
        }
        memcpy(s_ring[head].bytes, bytes, len);
        s_ring[head].bit_length = bit_length;
        s_ring[head].rx_timestamp_us = rx_timestamp_us;
        s_ring[head].rx_timestamp_sample_us = rx_timestamp_sample_us;
        s_ring[head].sample_index = sample_index;
        s_ring[head].sample_rate_sps = sample_rate_sps;
        s_ring[head].preamble_offset_samples = preamble_offset_samples;
        s_ring[head].icao = icao;
        s_ring_head = next;
        if (s_stats.queue_depth > s_stats.queue_high_water) s_stats.queue_high_water = s_stats.queue_depth + 1;
        xSemaphoreGive(s_ring_lock);
    } else {
        s_stats.frames_dropped_queue++;
    }
}

static bool ring_pop(ring_slot_t *out)
{
    if (!s_ring_lock || xSemaphoreTake(s_ring_lock, portMAX_DELAY) != pdTRUE) return false;
    uint32_t tail = s_ring_tail;
    if (tail == s_ring_head) {
        xSemaphoreGive(s_ring_lock);
        return false;
    }
    *out = s_ring[tail];
    s_ring_tail = (tail + 1) % RING_DEPTH;
    xSemaphoreGive(s_ring_lock);
    return true;
}

/* Convert timestamp to 12MHz units */
static uint64_t timestamp_to_12mhz_slot(const ring_slot_t *s)
{
    if (s->sample_rate_sps > 0) {
        uint64_t sample = s->sample_index + (uint64_t)s->preamble_offset_samples;
        uint64_t rate = s->sample_rate_sps;
        uint64_t sec = sample / rate;
        uint64_t rem = sample % rate;
        uint64_t ts12 = sec * 12000000ULL + (rem * 12000000ULL) / rate;
        s_stats.frames_ts_preamble++;
        return ts12;
    }
    if (s->rx_timestamp_us > 0) {
        s_stats.frames_ts_fallback++;
        return (uint64_t)s->rx_timestamp_us * 12ULL;
    }
    s_stats.frames_invalid_ts++;
    return 0;
}

static void bytes_to_hex(const uint8_t *b, int n, char *out, int outlen)
{
    int j = 0;
    for (int i = 0; i < n && j + 2 < outlen; i++) {
        snprintf(out + j, outlen - j, "%02X", b[i]);
        j += 2;
    }
    out[j] = '\0';
}

static bool send_all(int fd, const char *buf, size_t len)
{
    if (fd < 0 || !buf) return false;
    if (len == 0) len = strlen(buf);
    size_t sent = 0;
    while (sent < len) {
        int r = send(fd, buf + sent, len - sent, 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
            return false;
        }
        if (r == 0) return false;
        sent += r;
    }
    return true;
}

static bool json_appendf(char *buf, size_t cap, size_t *used,
                         const char *fmt, ...)
{
    if (!buf || !used || *used >= cap) {
        return false;
    }

    va_list ap;
    va_start(ap, fmt);
    size_t avail = cap - *used;
    int n = vsnprintf(buf + *used, avail, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= avail) {
        return false;
    }

    *used += (size_t)n;
    return true;
}

static void mlat_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "MLAT client task started");

    /* Register for IP events */
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_cb, NULL);

    while (!s_stop) {
        if (!s_cfg.enabled || s_cfg.host[0] == '\0') {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!s_net_ready) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* Resolve and connect */
        struct addrinfo hints = {0};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        char portbuf[8];
        snprintf(portbuf, sizeof(portbuf), "%u", (unsigned)s_cfg.port);
        struct addrinfo *res = NULL;
        if (getaddrinfo(s_cfg.host, portbuf, &hints, &res) != 0 || !res) {
            ESP_LOGW(TAG, "DNS failed for %s", s_cfg.host);
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd < 0) {
            freeaddrinfo(res);
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
            ESP_LOGW(TAG, "connect failed to %s:%u", s_cfg.host, s_cfg.port);
            close(fd);
            freeaddrinfo(res);
            s_stats.reconnects++;
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }
        freeaddrinfo(res);
        s_stats.connected = true;
        ESP_LOGI(TAG, "connected to %s:%u", s_cfg.host, s_cfg.port);

        /* Send handshake for BharatRadar mlat-server */
        char init[512];
        const char *user = s_cfg.uuid[0] ? s_cfg.uuid : (s_cfg.station_name[0] ? s_cfg.station_name : "esp32p4");
        snprintf(init, sizeof(init),
                 "{\"version\":3,\"user\":\"%s\",\"uuid\":\"%s\",\"client_version\":\"esp32p4-mlat/1.0\","
                 "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,"
                 "\"compress\":[\"none\"],\"clock_type\":\"dump1090\"}\n",
                 user, user,
                 s_cfg.lat, s_cfg.lon, s_cfg.alt_m);
        if (!send_all(fd, init, strlen(init))) {
            s_stats.connected = false;
            close(fd);
            s_stats.reconnects++;
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        /* Simple protocol loop: read responses, drain queue and send mlat messages */
        char rbuf[1024];
        memset(s_sync_cache, 0, sizeof(s_sync_cache));
        memset(s_track_last_reported, 0, sizeof(s_track_last_reported));
        s_track_last_count = 0;
        s_track_last_sent_us = esp_timer_get_time();
        s_sync_sent_this_sec = 0;
        s_sync_sec_start_us = esp_timer_get_time();
        bool conn_bad = false;
        while (!s_stop && s_cfg.enabled && !conn_bad) {
            /* Try to read server messages (non-blocking-ish via timeout) */
            int rr = recv(fd, rbuf, sizeof(rbuf)-1, MSG_DONTWAIT);
            if (rr == 0) {
                conn_bad = true;
                break;
            }
            if (rr < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    /* no data */
                } else {
                    conn_bad = true;
                    break;
                }
            }
            if (rr > 0) {
                rbuf[rr] = '\0';
                /* crude parse for results */
                if (strstr(rbuf, "\"result\"")) s_stats.server_results++;
                if (strstr(rbuf, "\"deny\"")) {
                    ESP_LOGW(TAG, "server denied connection: %s", rbuf);
                    conn_bad = true;
                    break;
                }
            }

            /* Drain up to N frames per iteration */
            int sent = 0;
            while (sent < 30) {
                ring_slot_t sl;
                if (!ring_pop(&sl)) break;
                uint64_t ts12 = timestamp_to_12mhz_slot(&sl);
                if (ts12 == 0) {
                    continue;
                }

                /* Try to build sync pairs for DF17 airborne position */
                if (sl.bit_length == 112) {
                    uint8_t df = sl.bytes[0] >> 3;
                    if (df == 17) {
                        uint8_t tc = sl.bytes[4] >> 3;
                        if ((tc >= 9 && tc <= 18) || (tc >= 20 && tc <= 22)) {
                            bool odd = (sl.bytes[6] & 0x04) != 0;
                            int64_t now_us = esp_timer_get_time();
                            int idx = -1;
                            for (int i = 0; i < MLAT_SYNC_CACHE_SIZE; i++) {
                                if (s_sync_cache[i].used && s_sync_cache[i].icao == sl.icao) { idx = i; break; }
                            }
                            if (idx < 0) {
                                /* find free */
                                for (int i = 0; i < MLAT_SYNC_CACHE_SIZE; i++) {
                                    if (!s_sync_cache[i].used) { idx = i; break; }
                                }
                                /* evict oldest */
                                if (idx < 0) {
                                    int best = 0;
                                    int64_t best_age = s_sync_cache[0].last_seen_us;
                                    for (int i = 1; i < MLAT_SYNC_CACHE_SIZE; i++) {
                                        if (s_sync_cache[i].last_seen_us < best_age) {
                                            best_age = s_sync_cache[i].last_seen_us;
                                            best = i;
                                        }
                                    }
                                    idx = best;
                                    /* clear */
                                    s_sync_cache[idx].used = true;
                                    s_sync_cache[idx].icao = 0;
                                    s_sync_cache[idx].have_even = false;
                                    s_sync_cache[idx].have_odd = false;
                                    s_sync_cache[idx].even_ts12 = 0;
                                    s_sync_cache[idx].odd_ts12 = 0;
                                    s_sync_cache[idx].even_received_us = 0;
                                    s_sync_cache[idx].odd_received_us = 0;
                                }
                            }
                            if (idx >= 0) {
                                mlat_sync_entry_t *e = &s_sync_cache[idx];
                                e->used = true;
                                e->icao = sl.icao;
                                e->last_seen_us = now_us;
                                if (!odd) {
                                    e->have_even = true;
                                    e->even_ts12 = ts12;
                                    e->even_received_us = now_us;
                                    memcpy(e->even_msg, sl.bytes, 14);
                                } else {
                                    e->have_odd = true;
                                    e->odd_ts12 = ts12;
                                    e->odd_received_us = now_us;
                                    memcpy(e->odd_msg, sl.bytes, 14);
                                }
                                /* expire old entries without pairs */
                                for (int i = 0; i < MLAT_SYNC_CACHE_SIZE; i++) {
                                    if (s_sync_cache[i].used && s_sync_cache[i].icao && !(s_sync_cache[i].have_even && s_sync_cache[i].have_odd)) {
                                        if (now_us - s_sync_cache[i].last_seen_us > 5000000LL) {
                                            if (s_sync_cache[i].have_even && (now_us - s_sync_cache[i].even_received_us > 5000000LL)) s_sync_cache[i].have_even = false;
                                            if (s_sync_cache[i].have_odd && (now_us - s_sync_cache[i].odd_received_us > 5000000LL)) s_sync_cache[i].have_odd = false;
                                        }
                                    }
                                }
                                if (e->have_even && e->have_odd) {
                                    if (llabs(e->even_received_us - e->odd_received_us) <= 5000000LL) {
                                        bool ordered = (e->even_ts12 < e->odd_ts12) && (e->even_received_us < e->odd_received_us);
                                        if (!ordered) {
                                            e->have_even = false;
                                            e->have_odd = false;
                                        } else {
                                            s_stats.sync_pairs_seen++;
                                            int64_t secnow = now_us / 1000000LL;
                                            int64_t secstart = s_sync_sec_start_us / 1000000LL;
                                            if (secstart != secnow) {
                                                s_sync_sec_start_us = now_us;
                                                s_sync_sent_this_sec = 0;
                                            }
                                            if (s_sync_sent_this_sec < 40) {
                                                char em[32], om[32];
                                                bytes_to_hex(e->even_msg, 14, em, sizeof(em));
                                                bytes_to_hex(e->odd_msg, 14, om, sizeof(om));
                                                char smsg[220];
                                                snprintf(smsg, sizeof(smsg), "{\"sync\":{\"et\":%llu,\"ot\":%llu,\"em\":\"%s\",\"om\":\"%s\"}}\n",
                                                         (unsigned long long)e->even_ts12, (unsigned long long)e->odd_ts12, em, om);
                                                if (!send_all(fd, smsg, strlen(smsg))) {
                                                    conn_bad = true;
                                                    break;
                                                }
                                                s_stats.sync_sent++;
                                                s_sync_sent_this_sec++;
                                            }
                                            e->have_even = false;
                                            e->have_odd = false;
                                        }
                                    } else {
                                        if (e->even_received_us < e->odd_received_us - 5000000LL) e->have_even = false;
                                        else e->have_odd = false;
                                    }
                                }
                            }
                        }
                    }
                }

                uint8_t mlen = sl.bit_length / 8;
                char mhex[64];
                bytes_to_hex(sl.bytes, mlen, mhex, sizeof(mhex));
                char msg[256];
                snprintf(msg, sizeof(msg), "{\"mlat\":{\"t\":%llu,\"m\":\"%s\"}}\n",
                         (unsigned long long)ts12, mhex);
                if (!send_all(fd, msg, strlen(msg))) {
                    conn_bad = true;
                    break;
                }
                s_stats.frames_submitted++;
                s_stats.tcp_sent++;
                sent++;
            }

            /* Track seen/lost once per second */
            {
                int64_t now_us = esp_timer_get_time();
                if (now_us - s_track_last_sent_us >= 1000000LL) {
                    uint32_t active[MLAT_TRACK_MAX];
                    int active_cnt = 0;

                    for (int i = 0;
                         i < MLAT_SYNC_CACHE_SIZE && active_cnt < MLAT_TRACK_MAX;
                         i++) {
                        if (s_sync_cache[i].used &&
                            s_sync_cache[i].icao != 0 &&
                            s_sync_cache[i].last_seen_us > 0 &&
                            now_us - s_sync_cache[i].last_seen_us <= 10000000LL) {
                            active[active_cnt++] = s_sync_cache[i].icao;
                        }
                    }

                    bool tracking_ok = true;

                    {
                        char buf[MLAT_TRACK_JSON_MAX];
                        size_t used = 0;
                        bool ok = json_appendf(buf, sizeof(buf), &used, "{\"seen\":[");
                        bool first = true;

                        if (ok) {
                            for (int a = 0; a < active_cnt; a++) {
                                bool seen_before = false;
                                for (int l = 0; l < s_track_last_count; l++) {
                                    if (s_track_last_reported[l] == active[a]) {
                                        seen_before = true;
                                        break;
                                    }
                                }
                                if (seen_before) {
                                    continue;
                                }
                                if (!first &&
                                    !json_appendf(buf, sizeof(buf), &used, ",")) {
                                    ok = false;
                                    break;
                                }
                                if (!json_appendf(buf, sizeof(buf), &used,
                                                  "\"%06x\"",
                                                  (unsigned int)active[a])) {
                                    ok = false;
                                    break;
                                }
                                first = false;
                            }
                        }

                        if (ok &&
                            !json_appendf(buf, sizeof(buf), &used, "]}\n")) {
                            ok = false;
                        }

                        if (!ok) {
                            tracking_ok = false;
                            ESP_LOGE(TAG, "seen JSON build failed (%d active)", active_cnt);
                        } else if (!first) {
                            if (!send_all(fd, buf, used)) {
                                conn_bad = true;
                                tracking_ok = false;
                            } else {
                                s_stats.tracking_seen_sent++;
                            }
                        }
                    }

                    if (!conn_bad && tracking_ok) {
                        char buf[MLAT_TRACK_JSON_MAX];
                        size_t used = 0;
                        bool ok = json_appendf(buf, sizeof(buf), &used, "{\"lost\":[");
                        bool first = true;

                        if (ok) {
                            for (int l = 0; l < s_track_last_count; l++) {
                                bool still = false;
                                for (int a = 0; a < active_cnt; a++) {
                                    if (active[a] == s_track_last_reported[l]) {
                                        still = true;
                                        break;
                                    }
                                }
                                if (still) {
                                    continue;
                                }
                                if (!first &&
                                    !json_appendf(buf, sizeof(buf), &used, ",")) {
                                    ok = false;
                                    break;
                                }
                                if (!json_appendf(buf, sizeof(buf), &used,
                                                  "\"%06x\"",
                                                  (unsigned int)s_track_last_reported[l])) {
                                    ok = false;
                                    break;
                                }
                                first = false;
                            }
                        }

                        if (ok &&
                            !json_appendf(buf, sizeof(buf), &used, "]}\n")) {
                            ok = false;
                        }

                        if (!ok) {
                            tracking_ok = false;
                            ESP_LOGE(TAG, "lost JSON build failed (%d tracked)", s_track_last_count);
                        } else if (!first) {
                            if (!send_all(fd, buf, used)) {
                                conn_bad = true;
                                tracking_ok = false;
                            } else {
                                s_stats.tracking_lost_sent++;
                            }
                        }
                    }

                    if (!conn_bad) {
                        if (tracking_ok) {
                            for (int a = 0;
                                 a < active_cnt && a < MLAT_TRACK_MAX;
                                 a++) {
                                s_track_last_reported[a] = active[a];
                            }
                            s_track_last_count = active_cnt;
                        }
                        s_track_last_sent_us = now_us;
                    }
                }
            }

            if (conn_bad) break;

            /* No ping */

            if (sent == 0) vTaskDelay(pdMS_TO_TICKS(10));
        }

        s_stats.connected = false;
        close(fd);
        if (conn_bad) {
            s_stats.reconnects++;
        } else {
            s_stats.reconnects++;
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }

    esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_cb);
    ESP_LOGI(TAG, "MLAT client task stopped");
    s_task = NULL;
    vTaskDelete(NULL);
}

void mlat_client_init(void)
{
    if (!s_ring_lock) {
        s_ring_lock = xSemaphoreCreateMutex();
    }
    memset(&s_stats, 0, sizeof(s_stats));
    s_ring_head = 0;
    s_ring_tail = 0;
    load_config();
    ESP_LOGI(TAG, "initialized (enabled=%d host=%s port=%u)", s_cfg.enabled, s_cfg.host, s_cfg.port);
    if (s_cfg.enabled && s_cfg.host[0]) {
        mlat_client_start();
    }
}

void mlat_client_start(void)
{
    if (s_task) {
        /* try to restart */
        mlat_client_stop();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (!s_cfg.enabled) return;
    if (s_cfg.host[0] == '\0') return;
    s_stop = false;
    xTaskCreatePinnedToCore(mlat_task, "mlat_task", 8192, NULL, 3, &s_task, 0);
}

void mlat_client_stop(void)
{
    s_stop = true;
    if (s_task) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
