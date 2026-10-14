/* BharatRadar ADS-B feeder (BEAST binary over TCP).
 *
 * A direct port of the proven reference implementation (br_feeder.py), which
 * was validated end-to-end against the live hub using raw frames captured from
 * this very receiver: 32k+ frames accepted, aircraft visible on the public map.
 *
 * Design constraints:
 *   - The decode hook runs synchronously on adsb_dsp (core 1, prio 6) and must
 *     never block. beast_feeder_push() only writes into a lock-free SPSC ring
 *     and drops on overflow.
 *   - All socket I/O lives on a core-0 task at prio 3, below the decode task,
 *     per LEARNING.md L24 (never co-locate saturating DSP with its servicing
 *     housekeeping on the same core/priority band).
 */

#include "beast_feeder.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <lwip/sockets.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "beast_feed";

/* BEAST wire constants (mirrors br_feeder.py). */
#define BEAST_ESC        0x1A
#define BEAST_SHORT      0x32 /* 7-byte Mode-S   */
#define BEAST_LONG       0x33 /* 14-byte ADS-B    */
#define BEAST_RSSI_NONE  0xFF

#define DEFAULT_HOST "feed.bharatradar.com"
#define DEFAULT_PORT 30004

/* NVS keys. NVS key names are capped at 15 chars. */
#define FEEDER_KEY_ENABLED "feed_en"
#define FEEDER_KEY_UUID    "feed_uuid"
#define FEEDER_KEY_NAME    "feed_name"
#define FEEDER_KEY_HOST    "feed_host"
#define FEEDER_KEY_PORT    "feed_port"

/* ---- Config -------------------------------------------------------------- */

static beast_feeder_config_t s_cfg = {
    .enabled = false,
    .uuid = "",
    .feeder_name = "",
    .host = DEFAULT_HOST,
    .port = DEFAULT_PORT,
};

static void feeder_config_defaults(void)
{
    s_cfg.enabled = false;
    s_cfg.uuid[0] = '\0';
    s_cfg.feeder_name[0] = '\0';
    snprintf(s_cfg.host, sizeof(s_cfg.host), "%s", DEFAULT_HOST);
    s_cfg.port = DEFAULT_PORT;
}

/* readsb rejects UUIDs shorter than 28 chars; we require 28..39. */
static bool uuid_is_valid(const char *uuid)
{
    const size_t n = strlen(uuid);
    return n >= 28 && n < BEAST_FEEDER_UUID_MAX;
}

static void feeder_load_config(void)
{
    feeder_config_defaults();

    nvs_handle_t h;
    if (nvs_open("radar", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t en = 0;
    if (nvs_get_u8(h, FEEDER_KEY_ENABLED, &en) == ESP_OK) {
        s_cfg.enabled = (en != 0);
    }
    size_t len = sizeof(s_cfg.uuid);
    if (nvs_get_str(h, FEEDER_KEY_UUID, s_cfg.uuid, &len) != ESP_OK) {
        s_cfg.uuid[0] = '\0';
    }
    if (!uuid_is_valid(s_cfg.uuid)) {
        /* An absent or malformed stored UUID must not enable feeding. */
        s_cfg.uuid[0] = '\0';
        s_cfg.enabled = false;
    }
    len = sizeof(s_cfg.feeder_name);
    if (nvs_get_str(h, FEEDER_KEY_NAME, s_cfg.feeder_name, &len) != ESP_OK) {
        s_cfg.feeder_name[0] = '\0';
    }
    len = sizeof(s_cfg.host);
    if (nvs_get_str(h, FEEDER_KEY_HOST, s_cfg.host, &len) != ESP_OK) {
        snprintf(s_cfg.host, sizeof(s_cfg.host), "%s", DEFAULT_HOST);
    }
    if (s_cfg.host[0] == '\0') {
        snprintf(s_cfg.host, sizeof(s_cfg.host), "%s", DEFAULT_HOST);
    }
    uint32_t port = 0;
    if (nvs_get_u32(h, FEEDER_KEY_PORT, &port) == ESP_OK && port > 0 && port <= 65535) {
        s_cfg.port = (uint16_t)port;
    }
    nvs_close(h);
}

static void feeder_save_config(void)
{
    nvs_handle_t h;
    if (nvs_open("radar", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, FEEDER_KEY_ENABLED, s_cfg.enabled ? 1 : 0);
    nvs_set_str(h, FEEDER_KEY_UUID, s_cfg.uuid);
    nvs_set_str(h, FEEDER_KEY_NAME, s_cfg.feeder_name);
    nvs_set_str(h, FEEDER_KEY_HOST, s_cfg.host);
    nvs_set_u32(h, FEEDER_KEY_PORT, (uint32_t)s_cfg.port);
    nvs_commit(h);
    nvs_close(h);
}

void beast_feeder_get_config(beast_feeder_config_t *out)
{
    if (out) *out = s_cfg;
}

esp_err_t beast_feeder_set_config(const beast_feeder_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Enabling requires a usable UUID; other fields are validated up front so
     * we never persist a half-valid config. */
    if (cfg->enabled && !uuid_is_valid(cfg->uuid)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->enabled && (cfg->host[0] == '\0' || cfg->port == 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    char uuid[BEAST_FEEDER_UUID_MAX];
    snprintf(uuid, sizeof(uuid), "%s", cfg->uuid);
    if (!uuid_is_valid(uuid)) {
        uuid[0] = '\0';
    }

    s_cfg.enabled = cfg->enabled;
    snprintf(s_cfg.uuid, sizeof(s_cfg.uuid), "%s", uuid);
    snprintf(s_cfg.feeder_name, sizeof(s_cfg.feeder_name), "%s", cfg->feeder_name);
    snprintf(s_cfg.host, sizeof(s_cfg.host), "%s",
             cfg->host[0] ? cfg->host : DEFAULT_HOST);
    s_cfg.port = cfg->port ? cfg->port : DEFAULT_PORT;

    feeder_save_config();
    ESP_LOGI(TAG, "config: enabled=%d uuid=%s host=%s port=%u",
             (int)s_cfg.enabled, s_cfg.uuid[0] ? s_cfg.uuid : "(unset)", s_cfg.host,
             (unsigned)s_cfg.port);
    return ESP_OK;
}

void beast_feeder_reset_settings(void)
{
    nvs_handle_t h;
    if (nvs_open("radar", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, FEEDER_KEY_ENABLED);
    nvs_erase_key(h, FEEDER_KEY_UUID);
    nvs_erase_key(h, FEEDER_KEY_NAME);
    nvs_erase_key(h, FEEDER_KEY_HOST);
    nvs_erase_key(h, FEEDER_KEY_PORT);
    nvs_commit(h);
    nvs_close(h);
    feeder_config_defaults();
}

/* ---- Lock-free SPSC ring ------------------------------------------------- */

#define RING_DEPTH 256 /* power of two */
typedef struct {
    uint8_t  bytes[14];
    uint8_t  bit_length;
    int64_t  rx_timestamp_us;
} ring_slot_t;

static ring_slot_t s_ring[RING_DEPTH];
static volatile uint32_t s_ring_head = 0; /* producer: decode task (core 1) */
static volatile uint32_t s_ring_tail = 0; /* consumer: feeder task (core 0)  */
static uint32_t s_frames_dropped = 0;

void beast_feeder_push(const uint8_t *bytes, uint8_t bit_length, int64_t rx_timestamp_us)
{
    if (!s_cfg.enabled || bytes == NULL) {
        return;
    }
    const uint8_t len = (uint8_t)(bit_length / 8u);
    if (len != 7 && len != 14) {
        return; /* only DF11/DF17 reach the feeder */
    }
    const uint32_t head = s_ring_head;
    const uint32_t next = (head + 1u) % RING_DEPTH;
    /* Full when tail is one slot behind head. Drop rather than stall decode. */
    if (next == s_ring_tail) {
        s_frames_dropped++;
        return;
    }
    memcpy(s_ring[head].bytes, bytes, len);
    s_ring[head].bit_length = bit_length;
    s_ring[head].rx_timestamp_us = rx_timestamp_us;
    s_ring_head = next;
}

static bool ring_pop(ring_slot_t *out)
{
    const uint32_t tail = s_ring_tail;
    if (tail == s_ring_head) {
        return false;
    }
    *out = s_ring[tail];
    s_ring_tail = (tail + 1u) % RING_DEPTH;
    return true;
}

static uint32_t ring_pending(void)
{
    return (s_ring_head - s_ring_tail) % RING_DEPTH;
}

/* ---- BEAST encoding ------------------------------------------------------ */

/* Double every 0x1A byte (stereo bit-stuffing). Returns escaped length. */
static size_t beast_escape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t j = 0;
    for (size_t i = 0; i < n; ++i) {
        out[j++] = in[i];
        if (in[i] == BEAST_ESC) {
            out[j++] = BEAST_ESC;
        }
    }
    return j;
}

/* Encode one raw Mode-S message as a full BEAST frame (leading 0x1A included).
 * Mirrors beast_frame() in br_feeder.py exactly. */
static size_t beast_encode(const ring_slot_t *s, uint8_t *out)
{
    const uint8_t len = (uint8_t)(s->bit_length / 8u);
    const uint8_t ftype = (len == 7) ? BEAST_SHORT : BEAST_LONG;

    /* 12 MHz counter from the esp_timer microsecond stamp. Hub only uses the
     * relative delta for rate estimation, so the epoch does not matter. */
    uint64_t ts = (uint64_t)s->rx_timestamp_us * 12u;

    uint8_t body[1 + 6 + 1 + 14];
    size_t bi = 0;
    body[bi++] = ftype;
    body[bi++] = (uint8_t)(ts >> 40) & 0xFF;
    body[bi++] = (uint8_t)(ts >> 32) & 0xFF;
    body[bi++] = (uint8_t)(ts >> 24) & 0xFF;
    body[bi++] = (uint8_t)(ts >> 16) & 0xFF;
    body[bi++] = (uint8_t)(ts >> 8) & 0xFF;
    body[bi++] = (uint8_t)(ts) & 0xFF;
    body[bi++] = BEAST_RSSI_NONE;
    memcpy(body + bi, s->bytes, len);
    bi += len;

    out[0] = BEAST_ESC;
    const size_t n = beast_escape(body, bi, out + 1);
    return n + 1;
}

/* ---- Feeder task --------------------------------------------------------- */

static volatile bool s_connected = false;
static volatile uint32_t s_frames_sent = 0;
static volatile uint32_t s_reconnects = 0;
static volatile int s_last_error = 0;

/* Set once the stack has an IP.
 *
 * CRITICAL: lwIP's getaddrinfo() posts to the tcpip thread and asserts
 * (`tcpip_send_msg_wait_sem ... Invalid mbox`) if the thread is not running
 * yet. Calling it before Ethernet is up therefore panics the chip on boot and,
 * because the feeder retries, produces an unbreakable reboot loop. Never
 * resolve a hostname until this flag is set. */
static volatile bool s_net_ready = false;

static void feeder_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    if (id == IP_EVENT_ETH_GOT_IP || id == IP_EVENT_STA_GOT_IP) {
        s_net_ready = true;
    }
}

static int feeder_connect(void)
{
    struct addrinfo hints = {0};
    hints.ai_family = AF_INET; /* hub is IPv4; AAAA-only would stall us */
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = NULL;
    const char *port_s = NULL;
    char portbuf[8];
    snprintf(portbuf, sizeof(portbuf), "%u", (unsigned)s_cfg.port);
    port_s = portbuf;

    int err = getaddrinfo(s_cfg.host, port_s, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGW(TAG, "dns failed for %s: gai=%d", s_cfg.host, err);
        s_last_error = err;
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
        freeaddrinfo(res);
        s_last_error = errno;
        return -1;
    }
    if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
        s_last_error = errno;
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);

    /* Handshake: 0x1A 0xE4 <ASCII UUID>. We deliberately omit the optional
     * 'W O' ping request: the hub works with UUID-only and we have no need for
     * pongs (verified against the live hub). */
    uint8_t hs[2 + BEAST_FEEDER_UUID_MAX];
    hs[0] = BEAST_ESC;
    hs[1] = 0xE4;
    const size_t ulen = strlen(s_cfg.uuid);
    memcpy(hs + 2, s_cfg.uuid, ulen);
    if (send(fd, hs, 2 + ulen, 0) < 0) {
        s_last_error = errno;
        close(fd);
        return -1;
    }

    s_connected = true;
    s_last_error = 0;
    ESP_LOGI(TAG, "connected %s:%u uuid=%s", s_cfg.host, (unsigned)s_cfg.port, s_cfg.uuid);
    return fd;
}

static void feeder_disconnect(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
    s_connected = false;
}

static void feeder_task(void *arg)
{
    (void)arg;
    int fd = -1;
    int backoff_ms = 1000;

    for (;;) {
        if (!s_cfg.enabled) {
            if (fd >= 0) {
                feeder_disconnect(fd);
                fd = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        /* Hard gate: no hostname resolution until lwIP has an IP. */
        if (!s_net_ready) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (fd < 0) {
            fd = feeder_connect();
            if (fd < 0) {
                s_reconnects++;
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                if (backoff_ms < 30000) backoff_ms *= 2;
                continue;
            }
            backoff_ms = 1000;
        }

        ring_slot_t slot;
        if (!ring_pop(&slot)) {
            /* Nothing to send: poll the socket briefly so we notice a hub-side
             * close, then idle without spinning. */
            char rx[64];
            int r = recv(fd, rx, sizeof(rx), MSG_DONTWAIT);
            if (r == 0) { /* hub closed */
                ESP_LOGW(TAG, "hub closed connection");
                feeder_disconnect(fd);
                fd = -1;
                s_reconnects++;
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint8_t frame[1 + 2 * 22]; /* leading 0x1A + worst-case escaped body */
        const size_t n = beast_encode(&slot, frame);
        int sent = send(fd, frame, n, 0);
        if (sent < 0) {
            s_last_error = errno;
            feeder_disconnect(fd);
            fd = -1;
            s_reconnects++;
            continue;
        }
        s_frames_sent++;
    }
}

esp_err_t beast_feeder_start(void)
{
    feeder_load_config();
    ESP_LOGI(TAG, "start: enabled=%d uuid=%s host=%s port=%u",
             (int)s_cfg.enabled, s_cfg.uuid[0] ? s_cfg.uuid : "(unset)", s_cfg.host,
             (unsigned)s_cfg.port);

    /* Must be called after esp_event_loop_create_default() (eth_netif_start). */
    esp_err_t ev = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, feeder_ip_event, NULL, NULL);
    if (ev != ESP_OK) {
        ESP_LOGW(TAG, "IP event register failed (%s); feeder waits on link",
                 esp_err_to_name(ev));
    }

    /* Core 0, prio 3: strictly below adsb_dsp (core 1, prio 6). */
    BaseType_t ok = xTaskCreatePinnedToCore(feeder_task, "beast_feed", 6144, NULL, 3,
                                            NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

void beast_feeder_get_status(beast_feeder_status_t *out)
{
    if (!out) return;
    out->enabled = s_cfg.enabled;
    out->connected = s_connected;
    out->frames_queued = ring_pending();
    out->frames_sent = s_frames_sent;
    out->frames_dropped = s_frames_dropped;
    out->reconnects = s_reconnects;
    out->last_error = s_last_error;
}