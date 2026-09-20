#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Aircraft track store (Phase 4).
 *
 * Pure C, no ESP-IDF dependencies, single-threaded by contract:
 * the producer (DSP task) calls adsb_state_update(); the 1 Hz status worker
 * calls adsb_state_expire() and adsb_state_fill(). Guard externally with a
 * mutex when producer and worker are distinct tasks.
 *
 * Global CPR decode is injected via adsb_state_set_cpr_pair(); on the ESP32
 * build main wires in the decoder's proven decode_global_cpr(). Host tests
 * inject a stub so the store logic is verified without the decoder.
 */

#define ADS_STATE_MAX_TRACKS 64
#define ADS_STATE_TTL_MS 60000u
#define ADS_STATE_CPR_MAX_AGE_MS 10000u

/* One CRC-valid decoded frame, plain-C mirror of the decoder output.
 * Strings are NUL-terminated. */
typedef struct {
    uint32_t icao;          /* 24-bit ICAO hex */
    uint8_t  type_code;     /* DF17 type code 0..31 */
    char     callsign[9];   /* empty. if has_callsign false */
    bool     has_callsign;
    int      altitude_ft;   /* barometric, ft */
    bool     has_altitude;
    int      speed_kts;
    bool     has_speed;
    int      heading_deg;   /* true north */
    bool     has_heading;
    int      vertical_rate_fpm;
    bool     has_vertical_rate;
    uint32_t cpr_latitude;  /* raw 17-bit CPR */
    uint32_t cpr_longitude;
    bool     cpr_odd;       /* true if odd-frame position message */
    bool     has_cpr;
    int      signal;        /* dB above noise floor */
} adsb_frame_t;

/* Global CPR decode hook. even+odd must be the same ICAO's two position
 * messages at the odd/even transition. Returns true and fills lat/lon. */
typedef bool (*adsb_cpr_pair_t)(uint32_t icao, uint32_t lat_even,
                                uint32_t lon_even, uint32_t lat_odd,
                                uint32_t lon_odd, double* lat, double* lon);

/* Per-aircraft track visible to the web layer. */
typedef struct {
    uint32_t icao;          /* 24-bit ICAO hex */
    char     flight[9];     /* callsign, NUL-terminated; empty if unknown */
    int      alt_baro;      /* ft, -1 if unknown */
    int      gs;            /* kts, -1 if unknown */
    int      heading;       /* deg true, -1 if unknown */
    int      baro_rate;     /* ft/min, -32768 if unknown */
    double   lat, lon;      /* valid only if has_position true */
    bool     has_position;
    int      signal;        /* dB above noise floor */
    uint32_t last_seen_ms;  /* monotonic ms tick of last frame */
    uint32_t revision;      /* bumped on every field change */
} adsb_aircraft_t;

#define ADS_STATE_SQUAWK_UNKNOWN 0

void adsb_state_init(void);

/* Inject the global-CPR pair decoder (ESP build: decoder's math). */
void adsb_state_set_cpr_pair(adsb_cpr_pair_t pair);

/* Feed one CRC-valid decoded frame. now_ms = monotonic ms tick. */
void adsb_state_update(const adsb_frame_t* f, uint32_t now_ms);

/* Expire tracks idle > ADS_STATE_TTL_MS. Call ~1 Hz. */
uint32_t adsb_state_expire(uint32_t now_ms);

/* Returns number of live tracks. Call ~1 Hz from status worker. */
uint32_t adsb_state_live_count(void);

/* Fill up to cap live tracks; returns number written. */
uint32_t adsb_state_fill(adsb_aircraft_t* out, uint32_t cap);

#ifdef __cplusplus
}
#endif