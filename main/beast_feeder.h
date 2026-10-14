#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* BharatRadar ADS-B feeder (BEAST binary protocol over TCP).
 *
 * Streams raw Mode-S frames from the decoder to a BharatRadar hub so
 * aircraft decoded on this device also appear on the public map.
 *
 * Protocol (verified end-to-end with the reference Python feeder against the
 * live hub using real device frames):
 *   connect:  0x1A 0xE4 <ASCII UUID>
 *   optional: 0x1A 'W' 'O' to request pings (we do not require pongs)
 *   frame:    0x1A <type> <6B BE 12MHz timestamp> <1B RSSI> <payload>
 *     type 0x32 + 7B  DF11, 0x33 + 14B DF17
 *   escaping: every literal 0x1A byte in the above is doubled.
 *   RSSI unknown -> 0xFF. Location is NOT sent over BEAST.
 *
 * Threading: the decode hook (prio 6, core 1) only ever enqueues into a small
 * lock-free ring and returns immediately; all socket I/O happens on a
 * dedicated core-0 task below the decode priority (learned from LEARNING.md L24:
 * never co-locate saturating DSP with the housekeeping that services it). */

#define BEAST_FEEDER_UUID_MAX 40 /* 36-char UUID + NUL */

/** Start the feeder task. Safe to call before an IP is up: the task waits for
 *  network readiness and reconnects with backoff. No-op if !enabled. */
esp_err_t beast_feeder_start(void);

/** Enqueue a raw frame (hot path, called from the decode hook). Non-blocking;
 *  drops the frame if the ring is full rather than stalling the decoder. */
void beast_feeder_push(const uint8_t *bytes, uint8_t bit_length, int64_t rx_timestamp_us);

/** Feeder configuration, mirrored to/from NVS under the "radar" namespace. */
typedef struct {
    bool     enabled;
    char     uuid[BEAST_FEEDER_UUID_MAX];
    char     feeder_name[32];
    char     host[64];
    uint16_t port;
} beast_feeder_config_t;

/** Current in-memory config (safe to call from HTTP task). */
void beast_feeder_get_config(beast_feeder_config_t *out);

/** Apply config from the web API; persists to NVS. Fields are validated
 *  (UUID length, host non-empty, port range). Returns ESP_ERR_INVALID_ARG
 *  on a rejected field; otherwise ESP_OK. */
esp_err_t beast_feeder_set_config(const beast_feeder_config_t *cfg);

/** Live feeder status for the UI. */
typedef struct {
    bool     enabled;
    bool     connected;
    uint32_t frames_queued;   /* frames currently waiting in the ring */
    uint32_t frames_sent;     /* total frames written to the socket */
    uint32_t frames_dropped;  /* ring-full drops (decoder never blocks) */
    uint32_t reconnects;
    int      last_error;      /* last socket errno, 0 if none */
} beast_feeder_status_t;

void beast_feeder_get_status(beast_feeder_status_t *out);

/** Erase feeder keys from NVS (settings/reset). */
void beast_feeder_reset_settings(void);