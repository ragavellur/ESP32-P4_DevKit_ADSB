#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* C bridge over the single C++ ADS-B Decoder instance. The RTL DSP task is
 * the only consumer (single-threaded). Stable C ABI: header + counters only,
 * no decoder internals leak out. */

bool adsb_decoder_bridge_init(void);         /* boot self-check; true on pass */
void adsb_decoder_bridge_reset(void);        /* full reset incl. app counters */
void adsb_decoder_bridge_process(const uint8_t* cu8, size_t bytes);
const char* adsb_decoder_bridge_last_error(void); /* stable string; "" if none */

typedef struct {
    uint32_t decoder_preambles;
    uint32_t decoder_frames;
    uint32_t decoder_df17;
    uint32_t decoder_crc_ok;
    uint16_t decoder_magnitude_min;
    uint16_t decoder_magnitude_max;
    uint32_t app_frames;
    uint32_t app_df17;
    uint32_t app_crc_ok;
} adsb_decoder_bridge_stats_t;

void adsb_decoder_bridge_get_stats(adsb_decoder_bridge_stats_t* out);

/* Every CRC-valid frame, mirrored to plain C. NUL-terminated strings. */
typedef struct {
    uint32_t icao;
    uint8_t  type_code;
    char     callsign[9];
    bool     has_callsign;
    int      altitude_ft;
    bool     has_altitude;
    int      speed_kts;
    bool     has_speed;
    int      heading_deg;
    bool     has_heading;
    int      vertical_rate_fpm;
    bool     has_vertical_rate;
    uint32_t cpr_latitude; /* raw 17-bit CPR */
    uint32_t cpr_longitude;
    bool     cpr_odd;
    bool     has_cpr;
    int      signal; /* dB above noise floor (pulse - quiet) */
} adsb_frame_info_t;

typedef void (*adsb_frame_hook_t)(const adsb_frame_info_t* f, void* ctx);

/* Register a callback for every CRC-valid decoded frame (hot path, be quick).
 * Pass NULL hook to unregister. Returns previous hook. */
adsb_frame_hook_t adsb_decoder_bridge_set_frame_hook(adsb_frame_hook_t hook,
                                                     void* ctx);

/* Global CPR position decode (even+odd pair). Re-exposes the decoder's proven
 * decode_global_cpr() to C. Both samples must be the same aircraft's even
 * and odd position messages. Returns true on success. */
typedef struct {
    uint32_t latitude;  /* raw 17-bit CPR */
    uint32_t longitude;
    bool     odd;
} adsb_cpr_sample_t;

bool adsb_decoder_bridge_cpr_pair(const adsb_cpr_sample_t* even,
                                  const adsb_cpr_sample_t* odd,
                                  bool use_odd, double* latitude,
                                  double* longitude);

void adsb_decoder_bridge_set_dc_filter(bool enable);

#ifdef __cplusplus
}
#endif
