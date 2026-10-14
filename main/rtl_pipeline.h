#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_rtl_sdr.h"
#include "adsb_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ADS-B IQ capture pipeline for the ESP32-P4 devkit.
 *
 * Brings up the esp-rtl-sdr USB host driver (core 0), waits for an accepted
 * RTL-SDR device, applies the ADS-B need (2.048 MSPS @ 1090 MHz), streams
 * CU8 IQ into a PSRAM multi-slot ring, and hands each block to the ADS-B
 * decoder task running on core 1.
 *
 * Non-blocking: rtl_pipeline_init() spawns the driver/DSP/status tasks and
 * returns immediately; state is observable via rtl_pipeline_get_status().
 */

/* IQ slot geometry (PSRAM). CU8 interleaved. */
#define IQ_BLOCK_BYTES (64 * 1024)
#define IQ_RING_SLOTS 8u

typedef enum {
    RTL_PIPELINE_IDLE = 0,      /**< not started yet */
    RTL_PIPELINE_OPTAINING,     /**< waiting for device / driver init */
    RTL_PIPELINE_STREAMING,     /**< driver streaming, decoder active */
    RTL_PIPELINE_DEVICE_LOST,   /**< disconnected, driver waiting to retry */
    RTL_PIPELINE_DEFUNCT,       /**< fatal error; manual reset required */
} rtl_pipeline_phase_t;

typedef enum {
    GAIN_MODE_MANUAL = 0,
    GAIN_MODE_ADAPTIVE = 1,
} gain_mode_t;

typedef struct {
    rtl_pipeline_phase_t phase;      /**< coarse state */
    bool device_present;             /**< accepted profile currently attached */
    bool stream_requested;           /**< start() has been called */
    bool decode_ok;                  /**< decoder self-check passed */
    uint32_t effective_sps;          /**< driver effective sample rate */
    uint32_t programmed_sps;         /**< sample rate we asked for */
    uint32_t overrun_cnt;            /**< driver USB overruns */
    uint32_t consumer_drops;         /**< ring overflow drops (app side) */
    uint32_t ring_free_slots;        /**< free IQ ring slots right now */
    uint32_t frame_cnt;              /**< decoder frames seen (cumulative) */
    uint32_t crc_ok_cnt;             /**< decoder frames with valid CRC */
    uint32_t track_count;            /**< live aircraft tracks (1 Hz refresh) */
    int32_t  gain_tenth_db;          /**< applied manual gain, tenths of dB */
    uint8_t  gain_mode;              /**< 0 = manual, 1 = adaptive */
    uint8_t  rf_state;               /**< ESP_RTL_SDR_HEALTH_RF_* value */
    int16_t  sample_min;             /**< recent RF min sample (health) */
    int16_t  sample_max;             /**< recent RF max sample (health) */
} rtl_pipeline_status_t;

/* Spawn pipeline tasks and kick off bring-up. Usually called once from
 * app_main after NVS init. */
esp_err_t rtl_pipeline_init(void);

/* Fill *out with a metrics snapshot. Never blocks for long. */
void rtl_pipeline_get_status(rtl_pipeline_status_t *out);

/* Rich decode diagnostics for USB-less verification (served by /api/debug).
 * Everything needed to judge reception health from the web UI alone. */
typedef struct {
    uint32_t preambles_total;      /**< decoder preambles detected (cumulative) */
    uint32_t frames_total;         /**< candidate frames (cumulative) */
    uint32_t crc_total;            /**< CRC-valid frames (cumulative) */
    uint32_t df17_total;           /**< DF17 count (cumulative) */
    uint32_t bytes_total;          /**< IQ bytes delivered by driver (cumulative) */
    uint16_t mag_min;              /**< decoder magnitude min (cumulative) */
    uint16_t mag_max;              /**< decoder magnitude max (cumulative) */
    /* 60 s window (0 until the first full window elapses) */
    uint32_t win60_seconds;
    uint32_t win60_preamble;
    uint32_t win60_frames;
    uint32_t win60_crc;
    uint32_t win60_df17;
    uint32_t win60_bytes;
    uint32_t win60_effective_sps;
    /* Tunables / config actually in effect */
    uint32_t programmed_sps;
    uint32_t preamble_threshold;
    int32_t  gain_tenth_db;
    uint8_t  gain_mode;
    bool     dc_filter;
    bool     crc_fix;
    /* health */
    uint8_t  health_usb;
    uint8_t  health_rf;
    uint8_t  health_overall;
    float    health_efficiency;
    uint8_t  sample_min;
    uint8_t  sample_max;
    char     advice[96];
} rtl_pipeline_debug_t;

void rtl_pipeline_get_debug(rtl_pipeline_debug_t *out);

/* Snapshot up to cap live aircraft tracks (mutex-guarded). Returns count. */
uint32_t rtl_pipeline_fill_aircraft(adsb_aircraft_t *out, uint32_t cap);

/* Apply current settings (gain, gain_mode, dc_filter) to driver/decoder.
 * Called from web API after NVS update for immediate effect. */
esp_err_t rtl_pipeline_apply_settings(void);

/* Restart the pipeline with new settings (e.g. after sample rate change).
 * Stops streaming, reconfigures driver, restarts streaming. */
void rtl_pipeline_restart(void);

/* Update in-memory settings from web API.
 * Call this before rtl_pipeline_apply_settings() when settings changed via web API. */
void rtl_pipeline_update_settings(int gain_tenth_db, bool dc_filter, bool adaptive_gain,
                                   gain_mode_t gain_mode, bool aggressive, bool crc_fix,
                                   uint32_t sample_rate_sps);

/* Get current settings snapshot. */
void rtl_pipeline_get_settings(int *gain_tenth_db, _Bool *dc_filter, _Bool *adaptive_gain,
                                gain_mode_t *gain_mode, _Bool *aggressive, _Bool *crc_fix,
                                uint32_t *sample_rate_sps);

#ifdef __cplusplus
}
#endif