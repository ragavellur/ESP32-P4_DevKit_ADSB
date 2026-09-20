#include "adsb_state.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Validation gates                                                    */
/* ------------------------------------------------------------------ */

/* Call signs may contain A-Z 0-9 and spaces; require at least one visible
 * char and no control characters (dump1090-style acceptance). */
static bool callsign_ok(const char* s)
{
    if (!s) return false;
    int visible = 0;
    for (int i = 0; i < 8; ++i) {
        const char c = s[i];
        if (c == '\0') break;
        if (c < 0x20 || c == 0x7f) return false;
        if (c != ' ') visible = 1;
    }
    return visible != 0;
}

static bool alt_ok(int v) { return v >= -2000 && v <= 100000; }
static bool gs_ok(int v)  { return v >= 0 && v <= 2500; }
static bool hdg_ok(int v) { return v >= 0 && v <= 359; }
static bool vr_ok(int v)  { return v >= -30000 && v <= 30000; }

/* ------------------------------------------------------------------ */
/* Track table                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t icao;
    uint32_t last_seen_ms;
    uint32_t revision;
    int      signal;
    int      alt_baro;
    int      gs;
    int      heading;
    int      baro_rate;
    int      squawk;
    char     flight[9];
    double   lat, lon;
    bool     has_position;
    /* CPR pairing state (raw 17-bit, per parity) */
    uint32_t cpr_lat[2];  /* [0]=even, [1]=odd */
    uint32_t cpr_lon[2];
    uint32_t cpr_ms[2];
    bool     cpr_have[2];
} adsb_track_t;

static adsb_track_t s_tracks[ADS_STATE_MAX_TRACKS];
static adsb_cpr_pair_t s_pair = 0;

void adsb_state_init(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
}

void adsb_state_set_cpr_pair(adsb_cpr_pair_t pair)
{
    s_pair = pair;
}

static adsb_track_t* find_or_add(uint32_t icao)
{
    adsb_track_t* free_slot = 0;
    for (int i = 0; i < ADS_STATE_MAX_TRACKS; ++i) {
        adsb_track_t* t = &s_tracks[i];
        if (t->icao == icao) {
            return t;
        }
        if (t->icao == 0 && !free_slot) {
            free_slot = t;
        }
    }
    if (!free_slot) {
        /* Table full: evict the track longest-idle (AT LEAST track[0]). */
        free_slot = &s_tracks[0];
        uint32_t oldest_ms = s_tracks[0].last_seen_ms;
        for (int i = 1; i < ADS_STATE_MAX_TRACKS; ++i) {
            if (s_tracks[i].last_seen_ms < oldest_ms) {
                oldest_ms = s_tracks[i].last_seen_ms;
                free_slot = &s_tracks[i];
            }
        }
    }
    memset(free_slot, 0, sizeof(*free_slot));
    free_slot->icao = icao;
    free_slot->alt_baro = -1;
    free_slot->gs = -1;
    free_slot->heading = -1;
    free_slot->baro_rate = -32768;
    free_slot->squawk = ADS_STATE_SQUAWK_UNKNOWN;
    return free_slot;
}

/* Global CPR decode once both parities are fresh (pairing window 10 s). */
static void cpr_try_pair(adsb_track_t* t, uint32_t now_ms)
{
    if (!s_pair) return;
    if (!t->cpr_have[0] || !t->cpr_have[1]) return;
    if (now_ms - t->cpr_ms[0] > ADS_STATE_CPR_MAX_AGE_MS ||
        now_ms - t->cpr_ms[1] > ADS_STATE_CPR_MAX_AGE_MS) {
        return;
    }
    double lat, lon;
    if (s_pair(t->icao, t->cpr_lat[0], t->cpr_lon[0],
               t->cpr_lat[1], t->cpr_lon[1], &lat, &lon)) {
        t->lat = lat;
        t->lon = lon;
        t->has_position = true;
    }
}

void adsb_state_update(const adsb_frame_t* f, uint32_t now_ms)
{
    if (!f || f->icao == 0 || f->icao > 0xffffff) return;
    adsb_track_t* t = find_or_add(f->icao);

    if (f->has_callsign && callsign_ok(f->callsign)) {
        /* Keep an already-good callsign; accept a NEW one only if it
         * differs (rejects "  " pad garbage from a superseded frame). */
        if (t->flight[0] == '\0' || strncmp(t->flight, f->callsign, 8) != 0) {
            memcpy(t->flight, f->callsign, 8);
            t->flight[8] = '\0';
        }
    }
    if (f->has_altitude && alt_ok(f->altitude_ft)) t->alt_baro = f->altitude_ft;
    if (f->has_speed && gs_ok(f->speed_kts)) t->gs = f->speed_kts;
    if (f->has_heading && hdg_ok(f->heading_deg)) t->heading = f->heading_deg;
    if (f->has_vertical_rate && vr_ok(f->vertical_rate_fpm)) {
        t->baro_rate = f->vertical_rate_fpm;
    }
    if (f->signal > 0) t->signal = f->signal;

    if (f->has_cpr) {
        const int parity = f->cpr_odd ? 1 : 0;
        t->cpr_lat[parity] = f->cpr_latitude;
        t->cpr_lon[parity] = f->cpr_longitude;
        t->cpr_ms[parity] = now_ms;
        t->cpr_have[parity] = true;
        cpr_try_pair(t, now_ms);
    }

    t->last_seen_ms = now_ms;
    ++t->revision;
}

uint32_t adsb_state_expire(uint32_t now_ms)
{
    uint32_t expired = 0;
    for (int i = 0; i < ADS_STATE_MAX_TRACKS; ++i) {
        adsb_track_t* t = &s_tracks[i];
        if (t->icao == 0) continue;
        if (now_ms - t->last_seen_ms > ADS_STATE_TTL_MS) {
            memset(t, 0, sizeof(*t));
            ++expired;
        }
    }
    return expired;
}

uint32_t adsb_state_live_count(void)
{
    uint32_t n = 0;
    for (int i = 0; i < ADS_STATE_MAX_TRACKS; ++i) {
        if (s_tracks[i].icao != 0) ++n;
    }
    return n;
}

uint32_t adsb_state_fill(adsb_aircraft_t* out, uint32_t cap)
{
    uint32_t n = 0;
    for (int i = 0; i < ADS_STATE_MAX_TRACKS && n < cap; ++i) {
        const adsb_track_t* t = &s_tracks[i];
        if (t->icao == 0) continue;
        adsb_aircraft_t* a = &out[n];
        a->icao = t->icao;
        memcpy(a->flight, t->flight, sizeof(a->flight));
        a->alt_baro = t->alt_baro;
        a->gs = t->gs;
        a->heading = t->heading;
        a->baro_rate = t->baro_rate;
        a->lat = t->lat;
        a->lon = t->lon;
        a->has_position = t->has_position;
        a->signal = t->signal;
        a->last_seen_ms = t->last_seen_ms;
        a->revision = t->revision;
        ++n;
    }
    return n;
}