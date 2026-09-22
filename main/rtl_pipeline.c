#include "rtl_pipeline.h"

#include <limits.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_rtl_sdr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "adsb_decoder_bridge.h"
#include "adsb_state.h"
#include "esp_log.h"
#include "nvs.h"

#define PIPELINE_TAG "rtl_pipe"
#define DSP_TAG "adsb_dsp"
#define STATUS_TAG "rtl_stat"

/* IQ ring geometry (CU8). 8 * 32 KiB = 256 KiB in PSRAM. */
#define IQ_BLOCK_BYTES (32 * 1024)

#define ADSB_FREQ_HZ 1090000000u
#define ADSB_RATE_SPS 2048000u

/* Waiting for a device/state change. */
#define DEVICE_WAIT_TICK pdMS_TO_TICKS(500)
#define ENUM_TIMEOUT_TICK pdMS_TO_TICKS(20000)

/* Settings NVS namespace and keys */
#define SETTINGS_NS "radar"
#define SETTINGS_KEY_GAIN "gain_tenth_db"
#define SETTINGS_KEY_DC_FILTER "dc_filter"
#define SETTINGS_KEY_ADAPTIVE_GAIN "adaptive_gain"
#define SETTINGS_KEY_GAIN_MODE "gain_mode"

/* Default settings (match current hardcoded values) */
#define DEFAULT_GAIN_TENTH_DB 496
#define DEFAULT_DC_FILTER false
#define DEFAULT_ADAPTIVE_GAIN false
#define DEFAULT_GAIN_MODE 0  /* 0=manual, 1=adaptive */

/* Adaptive gain parameters */
#define ADAPTIVE_GAIN_HYSTERESIS_S 3
#define ADAPTIVE_GAIN_STEP 1  /* ladder index step */

/* Local copy of R820T2 gain ladder (matches components/esp_rtl_sdr/private/gain_r820t2.hpp) */
typedef struct {
    int tenth_db;
    uint8_t reg05;
    uint8_t reg07;
} r820t2_gain_step_t;

static const r820t2_gain_step_t s_gain_ladder[] = {
    {0,   0x90, 0x60},
    {9,   0x91, 0x60},
    {14,  0x91, 0x61},
    {27,  0x92, 0x61},
    {37,  0x92, 0x62},
    {77,  0x93, 0x62},
    {87,  0x93, 0x63},
    {125, 0x94, 0x63},
    {144, 0x94, 0x64},
    {157, 0x95, 0x64},
    {166, 0x95, 0x65},
    {197, 0x96, 0x65},
    {207, 0x96, 0x66},
    {229, 0x97, 0x66},
    {254, 0x97, 0x67},
    {280, 0x98, 0x67},
    {297, 0x98, 0x68},
    {328, 0x99, 0x68},
    {338, 0x99, 0x69},
    {364, 0x9a, 0x69},
    {372, 0x9a, 0x6a},
    {386, 0x9b, 0x6a},
    {402, 0x9b, 0x6b},
    {421, 0x9c, 0x6b},
    {434, 0x9c, 0x6c},
    {439, 0x9d, 0x6c},
    {445, 0x9d, 0x6d},
    {480, 0x9e, 0x6d},
    {496, 0x9f, 0x6e},
};

#define S_GAIN_LADDER_COUNT (sizeof(s_gain_ladder) / sizeof(s_gain_ladder[0]))

/* Gain mode enum */
typedef enum {
    GAIN_MODE_MANUAL = 0,
    GAIN_MODE_ADAPTIVE = 1,
} gain_mode_t;

/* Persistent settings */
typedef struct {
    int gain_tenth_db;
    bool dc_filter;
    bool adaptive_gain;
    gain_mode_t gain_mode;
} radar_settings_t;

static radar_settings_t s_settings;

typedef struct {
    uint8_t *data;
    size_t bytes;
    uint32_t sequence;
} iq_slot_t;

typedef struct {
    esp_rtl_sdr_handle_t handle;
    iq_slot_t slots[IQ_RING_SLOTS];
    QueueHandle_t free_q;
    QueueHandle_t filled_q;
    SemaphoreHandle_t fsm_sem; /* posted by event callback for pipeline task */
    bool installed;
    bool started_req;
    bool device_present;
    bool decode_ok;
    esp_rtl_sdr_metrics_t metrics;
    /* consumer-side counters (written by DSP task, read by status task) */
    uint32_t frame_cnt;
    uint32_t crc_ok;
    uint32_t track_count;
    /* track-store lock (DSP task writes, status task expires/reads) */
    SemaphoreHandle_t track_mutex;
    TaskHandle_t dsp_handle;
} rtl_ctx_t;

static rtl_ctx_t s_ctx;

/* ------------------------------------------------------------------ */
/* Ring                                                                 */
/* ------------------------------------------------------------------ */

static bool ring_init(void)
{
    for (int i = 0; i < IQ_RING_SLOTS; i++) {
        s_ctx.slots[i].data = heap_caps_malloc(IQ_BLOCK_BYTES, MALLOC_CAP_SPIRAM);
        if (!s_ctx.slots[i].data) {
            ESP_LOGE(PIPELINE_TAG, "PSRAM slot %d alloc failed", i);
            return false;
        }
        s_ctx.slots[i].bytes = 0;
    }
    s_ctx.free_q = xQueueCreate(IQ_RING_SLOTS, sizeof(uint32_t));
    s_ctx.filled_q = xQueueCreate(IQ_RING_SLOTS, sizeof(uint32_t));
    if (!s_ctx.free_q || !s_ctx.filled_q) {
        ESP_LOGE(PIPELINE_TAG, "ring queues alloc failed");
        return false;
    }
    for (uint32_t i = 0; i < IQ_RING_SLOTS; i++) {
        xQueueSend(s_ctx.free_q, &i, 0);
    }
    return true;
}

/* Returns an owned slot index, or UINT32_MAX if none free. */
static uint32_t ring_take_free(void)
{
    uint32_t idx;
    if (xQueueReceive(s_ctx.free_q, &idx, 0) != pdTRUE) {
        return UINT32_MAX;
    }
    return idx;
}

static void ring_give_filled(uint32_t idx)
{
    if (xQueueSend(s_ctx.filled_q, &idx, 0) != pdTRUE) {
        xQueueSend(s_ctx.free_q, &idx, 0); /* drop: put back */
    }
}

/* ------------------------------------------------------------------ */
/* Settings (NVS)                                                      */
/* ------------------------------------------------------------------ */

static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NS, NVS_READONLY, &h) != ESP_OK) {
        goto defaults;
    }
    int32_t val32;
    if (nvs_get_i32(h, SETTINGS_KEY_GAIN, &val32) == ESP_OK) {
        s_settings.gain_tenth_db = val32;
    }
    uint8_t val8;
    if (nvs_get_u8(h, SETTINGS_KEY_DC_FILTER, &val8) == ESP_OK) {
        s_settings.dc_filter = val8 != 0;
    }
    if (nvs_get_u8(h, SETTINGS_KEY_ADAPTIVE_GAIN, &val8) == ESP_OK) {
        s_settings.adaptive_gain = val8 != 0;
    }
    if (nvs_get_u8(h, SETTINGS_KEY_GAIN_MODE, &val8) == ESP_OK) {
        s_settings.gain_mode = (gain_mode_t)val8;
    }
    nvs_close(h);
    return;
defaults:
    s_settings.gain_tenth_db = DEFAULT_GAIN_TENTH_DB;
    s_settings.dc_filter = DEFAULT_DC_FILTER;
    s_settings.adaptive_gain = DEFAULT_ADAPTIVE_GAIN;
    s_settings.gain_mode = DEFAULT_GAIN_MODE;
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i32(h, SETTINGS_KEY_GAIN, s_settings.gain_tenth_db);
    nvs_set_u8(h, SETTINGS_KEY_DC_FILTER, s_settings.dc_filter ? 1 : 0);
    nvs_set_u8(h, SETTINGS_KEY_ADAPTIVE_GAIN, s_settings.adaptive_gain ? 1 : 0);
    nvs_set_u8(h, SETTINGS_KEY_GAIN_MODE, (uint8_t)s_settings.gain_mode);
    nvs_commit(h);
    nvs_close(h);
}

static void settings_apply_initial(void)
{
    /* Apply DC filter setting to decoder */
    adsb_decoder_bridge_set_dc_filter(s_settings.dc_filter);
    ESP_LOGI(PIPELINE_TAG, "DC filter: %s", s_settings.dc_filter ? "ON" : "OFF");

    /* Apply initial gain mode and value */
    if (s_settings.gain_mode == GAIN_MODE_ADAPTIVE) {
        ESP_LOGI(PIPELINE_TAG, "Adaptive gain: ON (starting at %d.%d dB)",
                 s_settings.gain_tenth_db / 10, s_settings.gain_tenth_db % 10);
    } else {
        ESP_LOGI(PIPELINE_TAG, "Manual gain: %d.%d dB",
                 s_settings.gain_tenth_db / 10, s_settings.gain_tenth_db % 10);
    }
    (void)esp_rtl_sdr_set_tuner_gain_mode(s_ctx.handle, ESP_RTL_SDR_GAIN_MODE_MANUAL);
    (void)esp_rtl_sdr_set_tuner_gain(s_ctx.handle, s_settings.gain_tenth_db);
}

/* Find nearest gain ladder index for current gain setting */
static int gain_ladder_index(int gain_tenth_db)
{
    int best = 0;
    int best_err = INT_MAX;
    for (size_t i = 0; i < S_GAIN_LADDER_COUNT; ++i) {
        int err = abs(gain_tenth_db - s_gain_ladder[i].tenth_db);
        if (err < best_err) {
            best_err = err;
            best = i;
        }
    }
    return best;
}

/* Adaptive gain step function */
static void adaptive_gain_step(esp_rtl_sdr_health_info_t *health)
{
    if (s_settings.gain_mode != GAIN_MODE_ADAPTIVE) {
        return;
    }

    static int stable_count = 0;
    static int current_gain_idx = -1;

    if (current_gain_idx < 0) {
        current_gain_idx = gain_ladder_index(s_settings.gain_tenth_db);
    }

    bool need_down = (health->rf == ESP_RTL_SDR_HEALTH_RF_CLIPPING);
    bool need_up = (health->rf == ESP_RTL_SDR_HEALTH_RF_WEAK);

    if (need_down || need_up) {
        stable_count++;
        if (stable_count >= ADAPTIVE_GAIN_HYSTERESIS_S) {
            if (need_down && current_gain_idx > 0) {
                current_gain_idx--;
                s_settings.gain_tenth_db = s_gain_ladder[current_gain_idx].tenth_db;
                (void)esp_rtl_sdr_set_tuner_gain(s_ctx.handle, s_settings.gain_tenth_db);
                ESP_LOGW(PIPELINE_TAG, "Adaptive gain: stepped down to %d.%d dB (clipping)",
                         s_settings.gain_tenth_db / 10, s_settings.gain_tenth_db % 10);
                settings_save();
            } else if (need_up && current_gain_idx < (int)S_GAIN_LADDER_COUNT - 1) {
                current_gain_idx++;
                s_settings.gain_tenth_db = s_gain_ladder[current_gain_idx].tenth_db;
                (void)esp_rtl_sdr_set_tuner_gain(s_ctx.handle, s_settings.gain_tenth_db);
                ESP_LOGW(PIPELINE_TAG, "Adaptive gain: stepped up to %d.%d dB (weak)",
                         s_settings.gain_tenth_db / 10, s_settings.gain_tenth_db % 10);
                settings_save();
            }
            stable_count = 0;
        }
    } else {
        stable_count = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Track store feed (frame hook from decoder bridge)                   */
/* ------------------------------------------------------------------ */

static void bridge_to_state(const adsb_frame_info_t *f, uint32_t now_ms)
{
    if (!f) return;
    adsb_frame_t sf;
    memset(&sf, 0, sizeof(sf));
    sf.icao = f->icao;
    sf.type_code = f->type_code;
    memcpy(sf.callsign, f->callsign, sizeof(sf.callsign));
    sf.has_callsign = f->has_callsign;
    sf.altitude_ft = f->altitude_ft;
    sf.has_altitude = f->has_altitude;
    sf.speed_kts = f->speed_kts;
    sf.has_speed = f->has_speed;
    sf.heading_deg = f->heading_deg;
    sf.has_heading = f->has_heading;
    sf.vertical_rate_fpm = f->vertical_rate_fpm;
    sf.has_vertical_rate = f->has_vertical_rate;
    sf.cpr_latitude = f->cpr_latitude;
    sf.cpr_longitude = f->cpr_longitude;
    sf.cpr_odd = f->cpr_odd;
    sf.has_cpr = f->has_cpr;
    sf.signal = f->signal;
    if (s_ctx.track_mutex) xSemaphoreTake(s_ctx.track_mutex, portMAX_DELAY);
    adsb_state_update(&sf, now_ms);
    if (s_ctx.track_mutex) xSemaphoreGive(s_ctx.track_mutex);
}

static void on_frame_hook(const adsb_frame_info_t *f, void *ctx)
{
    (void)ctx;
    bridge_to_state(f, (uint32_t)(esp_timer_get_time() / 1000));
}

static bool pair_cpr_global(uint32_t icao, uint32_t lat_even, uint32_t lon_even,
                            uint32_t lat_odd, uint32_t lon_odd,
                            double *lat, double *lon)
{
    (void)icao;
    adsb_cpr_sample_t even = {lat_even, lon_even, false};
    adsb_cpr_sample_t odd = {lat_odd, lon_odd, true};
    return adsb_decoder_bridge_cpr_pair(&even, &odd, true, lat, lon);
}

/* ------------------------------------------------------------------ */
/* Driver events (runs on driver delivery task, core 0)                */
/* ------------------------------------------------------------------ */

static void on_event(esp_rtl_sdr_event_t event, const void *payload, void *ctx)
{
    (void)ctx;

    switch (event) {
    case ESP_RTL_SDR_EVT_ENUMERATED: {
        const esp_rtl_sdr_device_info_t *d =
            (const esp_rtl_sdr_device_info_t *)payload;
        if (d) {
            ESP_LOGI(PIPELINE_TAG, "RTL-SDR found: %s (vid=%04x pid=%04x hs=%d)",
                     d->product, d->vid, d->pid, d->high_speed);
        }
        break;
    }
    case ESP_RTL_SDR_EVT_READY:
        ESP_LOGI(PIPELINE_TAG, "RTL-SDR ready (accepted, idle)");
        s_ctx.device_present = true;
        if (s_ctx.fsm_sem) {
            xSemaphoreGive(s_ctx.fsm_sem);
        }
        break;
    case ESP_RTL_SDR_EVT_STREAM_STARTED:
        ESP_LOGI(PIPELINE_TAG, "stream started");
        s_ctx.started_req = true;
        break;
    case ESP_RTL_SDR_EVT_STOPPED:
        ESP_LOGI(PIPELINE_TAG, "stream stopped");
        s_ctx.started_req = false;
        break;
    case ESP_RTL_SDR_EVT_IQ_BLOCK: {
        const esp_rtl_sdr_iq_block_t *blk =
            (const esp_rtl_sdr_iq_block_t *)payload;
        if (!blk || blk->bytes == 0 || (blk->bytes & 1)) {
            break;
        }
        /* Cheap guard: reject absurd block sizes */
        if (blk->bytes > IQ_BLOCK_BYTES) {
            break;
        }
        uint32_t idx = ring_take_free();
        if (idx == UINT32_MAX) {
            break; /* consumer overload; driver counts its own overruns */
        }
        memcpy(s_ctx.slots[idx].data, blk->data, blk->bytes);
        s_ctx.slots[idx].bytes = blk->bytes;
        s_ctx.slots[idx].sequence = blk->sequence;
        ring_give_filled(idx);
        break;
    }
    case ESP_RTL_SDR_EVT_ERROR: {
        const esp_rtl_sdr_error_info_t *e =
            (const esp_rtl_sdr_error_info_t *)payload;
        if (e) {
            ESP_LOGE(PIPELINE_TAG, "driver error: %s", e->message);
        }
        if (s_ctx.fsm_sem) {
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(s_ctx.fsm_sem, &woken);
            /* This callback runs in a task context, not ISR, but consistent. */
        }
        break;
    }
    case ESP_RTL_SDR_EVT_DISCONNECTED:
        ESP_LOGW(PIPELINE_TAG, "device disconnected");
        s_ctx.device_present = false;
        if (s_ctx.fsm_sem) {
            xSemaphoreGive(s_ctx.fsm_sem);
        }
        break;
    case ESP_RTL_SDR_EVT_HEALTH: {
        const esp_rtl_sdr_health_info_t *h =
            (const esp_rtl_sdr_health_info_t *)payload;
        if (h && h->overall != ESP_RTL_SDR_HEALTH_OK
            && h->overall != ESP_RTL_SDR_HEALTH_UNKNOWN) {
            ESP_LOGW(PIPELINE_TAG, "health: usb=%d rf=%d eff=%.0f%% %s",
                     h->usb, h->rf, 100.0 * h->efficiency, h->advice);
        }
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Decoder task (core 1)                                               */
/* ------------------------------------------------------------------ */

static void adsb_decoder_task(void *arg)
{
    (void)arg;

    /* Decoder self-check at boot; log settle. */
    s_ctx.decode_ok = adsb_decoder_bridge_init();
    ESP_LOGI(DSP_TAG, "decoder self_check=%d", s_ctx.decode_ok);
    if (!s_ctx.decode_ok) {
        ESP_LOGE(DSP_TAG, "decoder self-check FAILED - frames will be ignored");
    }
    adsb_decoder_bridge_reset();
    adsb_decoder_bridge_set_frame_hook(on_frame_hook, NULL);

    for (;;) {
        uint32_t idx;
        if (xQueueReceive(s_ctx.filled_q, &idx, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        iq_slot_t *slot = &s_ctx.slots[idx];
        if (s_ctx.decode_ok) {
            adsb_decoder_bridge_process(slot->data, slot->bytes);
        }
        xQueueSend(s_ctx.free_q, &idx, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Status task (core 1)                                                */
/* ------------------------------------------------------------------ */

static void status_task(void *arg)
{
    (void)arg;
    uint32_t last_frames = 0;
    uint32_t last_crc = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        esp_rtl_sdr_metrics_t m;
        memset(&m, 0, sizeof(m));
        if (s_ctx.handle &&
            esp_rtl_sdr_get_metrics(s_ctx.handle, &m) == ESP_OK) {
            s_ctx.metrics = m;
        }

        adsb_decoder_bridge_stats_t ds;
        memset(&ds, 0, sizeof(ds));
        adsb_decoder_bridge_get_stats(&ds);

        const uint32_t frames_now = s_ctx.decode_ok ? ds.decoder_frames : 0;
        const uint32_t crc_now = s_ctx.decode_ok ? ds.decoder_crc_ok : 0;
        s_ctx.frame_cnt = frames_now;
        s_ctx.crc_ok = crc_now;

        ESP_LOGI(STATUS_TAG, "sps=%u eff=%u over=%u drops=%u | decoder: pre=%u "
                 "frm=%u(+%u/s) df17=%u crc=%u(+%u/s) mag=[%u,%u]",
                 m.sample_rate_sps, m.effective_sps, m.overruns,
                 m.consumer_drops,
                 ds.decoder_preambles, frames_now, frames_now - last_frames,
                 ds.decoder_df17, crc_now, crc_now - last_crc,
                 ds.decoder_magnitude_min, ds.decoder_magnitude_max);

        /* Adaptive gain based on health */
        if (s_ctx.handle && s_settings.gain_mode == GAIN_MODE_ADAPTIVE) {
            esp_rtl_sdr_health_info_t health;
            if (esp_rtl_sdr_get_health(s_ctx.handle, &health) == ESP_OK) {
                adaptive_gain_step(&health);
            }
        }

        /* 1 Hz track-store maintenance + count for status queries */
        if (s_ctx.track_mutex) {
            xSemaphoreTake(s_ctx.track_mutex, portMAX_DELAY);
        }
        const uint32_t expired = adsb_state_expire(
            (uint32_t)(esp_timer_get_time() / 1000));
        const uint32_t tracks = adsb_state_live_count();
        if (s_ctx.track_mutex) {
            xSemaphoreGive(s_ctx.track_mutex);
        }
        s_ctx.track_count = tracks;
        if (expired > 0 || tracks > 0) {
            ESP_LOGI(STATUS_TAG, "tracks=%u expired=%u/s", tracks, expired);
        }

        /* Refresh linux-style "frame rate" field for status queries */
        last_frames = frames_now;
        last_crc = crc_now;
    }
}

/* ------------------------------------------------------------------ */
/* Pipeline bring-up task (core 1)                                     */
/* ------------------------------------------------------------------ */

static void rtl_driver_task(void *arg)
{
    (void)arg;

    s_ctx.fsm_sem = xSemaphoreCreateBinary();
    if (!s_ctx.fsm_sem) {
        ESP_LOGE(PIPELINE_TAG, "fsm sem alloc failed");
        vTaskDelete(NULL);
        return;
    }

    esp_rtl_sdr_config_t cfg;
    esp_rtl_sdr_config_default(&cfg);
    cfg.host_library_already_installed = false;
    cfg.transfer_bytes = IQ_BLOCK_BYTES;
    cfg.transfer_count = 6;  /* 6 URBs × 32 KiB = 192 KiB buffer pool (47 ms @ 4.1 MB/s) */
    cfg.event_cb = on_event;
    cfg.event_ctx = NULL;
    cfg.usb_task_priority = 20;
    cfg.usb_task_core_id = 0;
    cfg.delivery_mode = ESP_RTL_SDR_DELIVERY_CALLBACK;

    esp_err_t ret = esp_rtl_sdr_install(&cfg, &s_ctx.handle);
    if (ret != ESP_OK) {
        ESP_LOGE(PIPELINE_TAG, "install failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    s_ctx.installed = true;
    ESP_LOGI(PIPELINE_TAG, "driver installed");

    /* Wait for device accept / ready (either event or state poll). */
    TickType_t deadline = xTaskGetTickCount() + ENUM_TIMEOUT_TICK;
    while (!s_ctx.device_present && xTaskGetTickCount() < deadline) {
        vTaskDelay(DEVICE_WAIT_TICK);
    }

    if (!s_ctx.device_present) {
        ESP_LOGE(PIPELINE_TAG,
                 "no RTL-SDR device within %dms (state=%s)",
                 ENUM_TIMEOUT_TICK / portTICK_PERIOD_MS,
                 esp_rtl_sdr_state_to_name(esp_rtl_sdr_get_state(s_ctx.handle)));
        /* Keep driver installed; a hotplug later may still attach. */
        for (;;) {
            xSemaphoreTake(s_ctx.fsm_sem, portMAX_DELAY);
            if (s_ctx.device_present) {
                break;
            }
        }
    }

    ret = esp_rtl_sdr_apply_need(s_ctx.handle, ESP_RTL_SDR_NEED_ADSB);
    if (ret != ESP_OK) {
        ESP_LOGE(PIPELINE_TAG, "apply_need failed: %s", esp_err_to_name(ret));
    }

    for (int start_attempt = 0; start_attempt < 3; ++start_attempt) {
        ret = esp_rtl_sdr_start(s_ctx.handle, &(esp_rtl_sdr_stream_config_t){
            .struct_size = sizeof(esp_rtl_sdr_stream_config_t),
            .preset = ESP_RTL_SDR_PRESET_CUSTOM_HZ,
            .frequency_hz = ADSB_FREQ_HZ,
            .sample_rate_sps = ADSB_RATE_SPS,
            .max_bytes = 0,
            .timeout_ms = 0,
        });
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(PIPELINE_TAG, "start attempt %d failed: %s",
                 start_attempt + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(PIPELINE_TAG, "start failed after retries: %s",
                 esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(PIPELINE_TAG, "streaming 1090 MHz @ 2.048 MSPS");

    /* Apply user settings (gain, DC filter, adaptive gain) */
    settings_apply_initial();

    /* Park: react to disconnect / driver errors by logging for now. */
    for (;;) {
        xSemaphoreTake(s_ctx.fsm_sem, portMAX_DELAY);
        if (!s_ctx.device_present) {
            ESP_LOGW(PIPELINE_TAG, "device gone; waiting for re-attach...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_rtl_sdr_refresh_device_list(s_ctx.handle);
            size_t n = 0;
            esp_rtl_sdr_get_device_count(s_ctx.handle, &n);
            ESP_LOGI(PIPELINE_TAG, "device count=%u", (unsigned)n);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public                                                              */
/* ------------------------------------------------------------------ */

esp_err_t rtl_pipeline_init(void)
{
    if (s_ctx.installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ring_init()) {
        return ESP_ERR_NO_MEM;
    }
    s_ctx.track_mutex = xSemaphoreCreateMutex();
    if (!s_ctx.track_mutex) {
        return ESP_ERR_NO_MEM;
    }
    adsb_state_init();
    adsb_state_set_cpr_pair(pair_cpr_global);

    /* Load persistent settings from NVS */
    settings_load();
    ESP_LOGI(PIPELINE_TAG, "Settings loaded: gain=%d.%d dB, dc_filter=%s, adaptive_gain=%s, gain_mode=%s",
             s_settings.gain_tenth_db / 10, s_settings.gain_tenth_db % 10,
             s_settings.dc_filter ? "ON" : "OFF",
             s_settings.adaptive_gain ? "ON" : "OFF",
             s_settings.gain_mode == GAIN_MODE_ADAPTIVE ? "adaptive" : "manual");

    BaseType_t ok = xTaskCreatePinnedToCore(rtl_driver_task, "rtl_driver",
                                            4096, NULL, 5, NULL, 1);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }
    ok = xTaskCreatePinnedToCore(status_task, "rtl_stat", 2048, NULL, 3, NULL, 1);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }
    ok = xTaskCreatePinnedToCore(adsb_decoder_task, "adsb_dsp", 8192, NULL, 6,
                                  &s_ctx.dsp_handle, 1);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(PIPELINE_TAG, "tasks started (rtl_driver=core1, adsb_dsp=core1, rtl_stat=core1)");
    return ESP_OK;
}

void rtl_pipeline_get_status(rtl_pipeline_status_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->stream_requested = s_ctx.started_req;
    out->device_present = s_ctx.device_present;
    out->decode_ok = s_ctx.decode_ok;
    out->effective_sps = s_ctx.metrics.effective_sps;
    out->overrun_cnt = s_ctx.metrics.overruns;
    out->consumer_drops = s_ctx.metrics.consumer_drops;
    out->ring_free_slots = uxQueueMessagesWaiting(s_ctx.free_q);
    out->frame_cnt = s_ctx.frame_cnt;
    out->crc_ok_cnt = s_ctx.crc_ok;
    out->track_count = s_ctx.track_count;
}

uint32_t rtl_pipeline_fill_aircraft(adsb_aircraft_t *out, uint32_t cap)
{
    if (!out || cap == 0) return 0;
    if (s_ctx.track_mutex) xSemaphoreTake(s_ctx.track_mutex, portMAX_DELAY);
    uint32_t n = adsb_state_fill(out, cap);
    if (s_ctx.track_mutex) xSemaphoreGive(s_ctx.track_mutex);
    return n;
}