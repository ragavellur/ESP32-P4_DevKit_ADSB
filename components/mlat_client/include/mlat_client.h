#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLAT_MAX_HOST_LEN 128
#define MLAT_MAX_UUID_LEN 40  /* similar to beast feeder */
#define MLAT_MAX_NAME_LEN 32

typedef struct {
    bool enabled;
    char host[MLAT_MAX_HOST_LEN];
    uint16_t port;
    char uuid[MLAT_MAX_UUID_LEN];      /* station/user id as required by service */
    char station_name[MLAT_MAX_NAME_LEN];
    double lat;
    double lon;
    double alt_m;
    bool privacy;
    bool return_results;
    bool use_udp;
    bool use_zlib;
} mlat_client_config_t;

typedef struct {
    uint32_t frames_observed;
    uint32_t frames_eligible;
    uint32_t frames_submitted;
    uint32_t frames_dropped_queue;
    uint32_t frames_invalid_ts;
    uint32_t frames_ts_preamble;
    uint32_t frames_ts_fallback;
    uint32_t sync_pairs_seen;
    uint32_t sync_sent;
    uint32_t tracking_seen_sent;
    uint32_t tracking_lost_sent;
    uint32_t tcp_sent;
    uint32_t udp_sent;
    uint32_t reconnects;
    uint32_t protocol_errors;
    uint32_t server_results;
    bool connected;
    uint32_t queue_depth;
    uint32_t queue_high_water;
} mlat_client_stats_t;

void mlat_client_init(void);
void mlat_client_start(void);
void mlat_client_stop(void);

void mlat_client_get_config(mlat_client_config_t *out);
esp_err_t mlat_client_set_config(const mlat_client_config_t *cfg);
void mlat_client_reset_settings(void);

void mlat_client_get_stats(mlat_client_stats_t *out);

/* Push a decoded frame for potential MLAT submission.
 * Called from decoder hot path - must be non-blocking. */
void mlat_client_push_frame(const uint8_t *bytes, uint8_t bit_length,
                            int64_t rx_timestamp_us,
                            int64_t rx_timestamp_sample_us,
                            uint64_t sample_index,
                            uint32_t sample_rate_sps,
                            uint32_t preamble_offset_samples,
                            uint32_t icao);

#ifdef __cplusplus
}
#endif
