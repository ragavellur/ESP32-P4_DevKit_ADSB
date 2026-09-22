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
} rtl_pipeline_status_t;

/* Spawn pipeline tasks and kick off bring-up. Usually called once from
 * app_main after NVS init. */
esp_err_t rtl_pipeline_init(void);

/* Fill *out with a metrics snapshot. Never blocks for long. */
void rtl_pipeline_get_status(rtl_pipeline_status_t *out);

/* Snapshot up to cap live aircraft tracks (mutex-guarded). Returns count. */
uint32_t rtl_pipeline_fill_aircraft(adsb_aircraft_t *out, uint32_t cap);

/* Apply current settings (gain, gain_mode, dc_filter) to driver/decoder.
 * Called from web API after NVS update for immediate effect. */
esp_err_t rtl_pipeline_apply_settings(void);

#ifdef __cplusplus
}
#endif