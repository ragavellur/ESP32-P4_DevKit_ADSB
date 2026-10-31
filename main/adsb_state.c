#include "adsb_state.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Validation gates                                                    */
/* ------------------------------------------------------------------ */

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
    bool     icao_is_icao;
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
    bool     cpr_surface[2];
    double   last_pos_lat;
    double   last_pos_lon;
    uint32_t last_pos_ms;
    bool     has_last_pos;
} adsb_track_t;

static adsb_track_t s_tracks[ADS_STATE_MAX_TRACKS];
static adsb_cpr_pair_t s_pair = 0;
static double s_rx_lat = 0.0;
static double s_rx_lon = 0.0;
static bool s_rx_loc_set = false;

/* Diagnostics */
static adsb_state_diag_t s_diag = {0};

void adsb_state_init(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
    memset(&s_diag, 0, sizeof(s_diag));
}

void adsb_state_get_diag(adsb_state_diag_t* out)
{
    if (!out) return;
    *out = s_diag;
}

void adsb_state_set_cpr_pair(adsb_cpr_pair_t pair)
{
    s_pair = pair;
}

void adsb_state_set_location(double lat, double lon)
{
    if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
        s_rx_lat = lat;
        s_rx_lon = lon;
        s_rx_loc_set = true;
    }
}

static adsb_track_t* find_or_add_by_addr(uint32_t icao, bool icao_is_icao)
{
    for (int i = 0; i < ADS_STATE_MAX_TRACKS; ++i) {
        adsb_track_t* t = &s_tracks[i];
        if (t->icao == icao && t->icao != 0 && t->icao_is_icao == icao_is_icao) {
            return t;
        }
    }
    adsb_track_t* free_slot = 0;
    for (int i = 0; i < ADS_STATE_MAX_TRACKS; ++i) {
        adsb_track_t* t = &s_tracks[i];
        if (t->icao == 0 && !free_slot) {
            free_slot = t;
        }
    }
    if (!free_slot) {
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
    free_slot->icao_is_icao = icao_is_icao;
    free_slot->alt_baro = -1;
    free_slot->gs = -1;
    free_slot->heading = -1;
    free_slot->baro_rate = -32768;
    free_slot->squawk = ADS_STATE_SQUAWK_UNKNOWN;
    return free_slot;
}

static double haversine_nm(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double km = R * c;
    return km * 0.539957;
}

static int cpr_nl(double lat)
{
    if (lat >= 87.0 || lat <= -87.0) return 1;
    return (int)floor(2.0 * M_PI / acos(1.0 - (1.0 - cos(M_PI / 30.0)) /
                                                    (cos(lat * M_PI / 180.0) * cos(lat * M_PI / 180.0))));
}

/* Local CPR for airborne using reference position */
static bool cpr_local_airborne_try(uint32_t lat_cpr, uint32_t lon_cpr, bool odd,
                                   double ref_lat, double ref_lon, double* out_lat, double* out_lon)
{
    if (ref_lat < -90.0 || ref_lat > 90.0 || ref_lon < -180.0 || ref_lon > 180.0) return false;
    if (lat_cpr >= 131072 || lon_cpr >= 131072) return false;
    const double yz = (double)lat_cpr / 131072.0;
    const double xz = (double)lon_cpr / 131072.0;
    const double dLat = 360.0 / (60.0 - (odd ? 1.0 : 0.0));
    double mod_lat = ref_lat - floor(ref_lat / dLat) * dLat;
    if (mod_lat < 0.0) mod_lat += dLat;
    int j = (int)floor(ref_lat / dLat) + (int)floor(mod_lat / dLat - yz + 0.5);
    double lat = dLat * ((double)j + yz);
    while (lat >= 270.0) lat -= 360.0;
    while (lat < -90.0) lat += 360.0;
    if (lat > 90.0) lat -= 180.0;
    if (lat < -90.0) lat += 180.0;
    if (lat < -90.0 || lat > 90.0 || isnan(lat) || isinf(lat)) return false;
    int nl = cpr_nl(lat);
    if (nl < 1) return false;
    int ni = nl - (odd ? 1 : 0);
    if (ni < 1) ni = 1;
    const double dLon = 360.0 / (double)ni;
    double mod_lon = ref_lon - floor(ref_lon / dLon) * dLon;
    if (mod_lon < 0.0) mod_lon += dLon;
    int m = (int)floor(ref_lon / dLon) + (int)floor(mod_lon / dLon - xz + 0.5);
    double lon = dLon * ((double)m + xz);
    while (lon >= 180.0) lon -= 360.0;
    while (lon < -180.0) lon += 360.0;
    if (lon < -180.0 || lon > 180.0 || isnan(lon) || isinf(lon)) return false;
    double d = haversine_nm(ref_lat, ref_lon, lat, lon);
    if (d > 180.0) return false;
    *out_lat = lat;
    *out_lon = lon;
    return true;
}



static bool cpr_local_surface_try(uint32_t lat_cpr, uint32_t lon_cpr, bool odd,
                                  double ref_lat, double ref_lon, double* out_lat, double* out_lon)
{
    if (ref_lat < -90.0 || ref_lat > 90.0 || ref_lon < -180.0 || ref_lon > 180.0) return false;
    if (lat_cpr >= 131072 || lon_cpr >= 131072) return false;
    const double yz = (double)lat_cpr / 131072.0;
    const double xz = (double)lon_cpr / 131072.0;
    const double dLat = 90.0 / (60.0 - (odd ? 1.0 : 0.0));
    double mod_lat = ref_lat - floor(ref_lat / dLat) * dLat;
    if (mod_lat < 0.0) mod_lat += dLat;
    int j = (int)floor(ref_lat / dLat) + (int)floor(mod_lat / dLat - yz + 0.5);
    double lat = dLat * ((double)j + yz);
    double candidates_lat[8];
    int ncand = 0;
    for (int off = -4; off <= 4 && ncand < 8; off++) {
        double c = lat + off * 90.0;
        while (c >= 90.0) c -= 180.0;
        while (c < -90.0) c += 180.0;
        candidates_lat[ncand++] = c;
    }
    double bestd = 1e18;
    double resolved_lat = lat;
    for (int k = 0; k < ncand; k++) {
        double dd = fabs(candidates_lat[k] - ref_lat);
        if (dd < bestd) { bestd = dd; resolved_lat = candidates_lat[k]; }
    }
    if (resolved_lat > 90.0) resolved_lat -= 180.0;
    if (resolved_lat < -90.0) resolved_lat += 180.0;
    if (resolved_lat < -90.0 || resolved_lat > 90.0) return false;
    int nl = cpr_nl(resolved_lat);
    if (nl < 1) return false;
    int ni = nl - (odd ? 1 : 0);
    if (ni < 1) ni = 1;
    const double dLon = 90.0 / (double)ni;
    double mod_lon = ref_lon - floor(ref_lon / dLon) * dLon;
    if (mod_lon < 0.0) mod_lon += dLon;
    int m = (int)floor(ref_lon / dLon) + (int)floor(mod_lon / dLon - xz + 0.5);
    double lon = dLon * ((double)m + xz);
    double candidates_lon[8];
    ncand = 0;
    for (int off = -4; off <= 4 && ncand < 8; off++) {
        double c = lon + off * 90.0;
        while (c >= 180.0) c -= 360.0;
        while (c < -180.0) c += 360.0;
        candidates_lon[ncand++] = c;
    }
    bestd = 1e18;
    double resolved_lon = lon;
    for (int k = 0; k < ncand; k++) {
        double ad = fabs(candidates_lon[k] - ref_lon);
        if (ad > 180.0) ad = 360.0 - ad;
        if (ad < bestd) { bestd = ad; resolved_lon = candidates_lon[k]; }
    }
    if (resolved_lon > 180.0) resolved_lon -= 360.0;
    if (resolved_lon < -180.0) resolved_lon += 360.0;
    if (resolved_lon < -180.0 || resolved_lon > 180.0) return false;
    double d = haversine_nm(ref_lat, ref_lon, resolved_lat, resolved_lon);
    if (d > 45.0) return false;
    *out_lat = resolved_lat;
    *out_lon = resolved_lon;
    return true;
}

/* Global CPR decode once both parities are fresh (pairing window 10 s). */
static void cpr_try_pair(adsb_track_t* t, uint32_t now_ms)
{
    if (!s_pair) return;
    if (!t->cpr_have[0] || !t->cpr_have[1]) return;
    if (t->cpr_surface[0] != t->cpr_surface[1]) {
        s_diag.cpr_reject_mismatch++;
        return;
    }
    bool is_surface = t->cpr_surface[0] || t->cpr_surface[1];
    if (now_ms - t->cpr_ms[0] > ADS_STATE_CPR_MAX_AGE_MS ||
        now_ms - t->cpr_ms[1] > ADS_STATE_CPR_MAX_AGE_MS) {
        s_diag.cpr_stale++;
        return;
    }
    if (is_surface) {
        /* Surface uses different NL; global CPR formula is similar but we must be careful.
         * Try global pair first but validate surface constraints. */
        double lat, lon;
        s_diag.cpr_global_attempt++;
        if (s_pair(t->icao, t->cpr_lat[0], t->cpr_lon[0],
                   t->cpr_lat[1], t->cpr_lon[1], &lat, &lon)) {
            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
                s_diag.cpr_global_fail++;
                s_diag.cpr_invalid++;
                return;
            }
            if (s_rx_loc_set) {
                double d = haversine_nm(s_rx_lat, s_rx_lon, lat, lon);
                /* Surface positions are usually close to ground/airports but still can be far; be reasonable */
                if (d > 250.0) {
                    s_diag.cpr_invalid++;
                    return;
                }
            }
            if (t->has_position) {
                double d2 = haversine_nm(t->lat, t->lon, lat, lon);
                /* Surface movement limits roughly */
                if (d2 > 50.0) {
                    s_diag.cpr_invalid++;
                    return;
                }
            }
            t->lat = lat;
            t->lon = lon;
            t->has_position = true;
            t->has_last_pos = true;
            t->last_pos_lat = lat;
            t->last_pos_lon = lon;
            t->last_pos_ms = now_ms;
            s_diag.cpr_global_success++;
            s_diag.tracks_pos_global++;
            return;
        } else {
            s_diag.cpr_global_fail++;
            return;
        }
    }
    double lat, lon;
    s_diag.cpr_global_attempt++;
    if (s_pair(t->icao, t->cpr_lat[0], t->cpr_lon[0],
               t->cpr_lat[1], t->cpr_lon[1], &lat, &lon)) {
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            s_diag.cpr_global_fail++;
            s_diag.cpr_invalid++;
            return;
        }
        if (s_rx_loc_set) {
            double d = haversine_nm(s_rx_lat, s_rx_lon, lat, lon);
            if (d > 360.0) {
                /* allow but be cautious; still possible far away */
            }
        }
        if (t->has_position && t->gs >= 0) {
            double d2 = haversine_nm(t->lat, t->lon, lat, lon);
            double dt_s = (now_ms > t->last_seen_ms) ? ((now_ms - t->last_seen_ms) / 1000.0) : 1.0;
            if (dt_s < 0.001) dt_s = 0.001;
            double max_nm = (t->gs / 3600.0) * dt_s * 2.5 + 5.0;
            if (d2 > max_nm && max_nm < 500.0) {
                s_diag.cpr_invalid++;
                return;
            }
        }
        t->lat = lat;
        t->lon = lon;
        t->has_position = true;
        t->has_last_pos = true;
        t->last_pos_lat = lat;
        t->last_pos_lon = lon;
        t->last_pos_ms = now_ms;
        s_diag.cpr_global_success++;
        s_diag.tracks_pos_global++;
        return;
    } else {
        s_diag.cpr_global_fail++;
    }
}

static void cpr_try_local(adsb_track_t* t, const adsb_frame_t* f, uint32_t now_ms)
{
    if (!f->has_cpr) return;
    if (t->has_position) return; /* prefer global or keep existing */
    bool is_surface = f->cpr_surface;
    s_diag.cpr_local_attempt++;
    if (is_surface) {
        double ref_lat = 0, ref_lon = 0;
        bool have_ref = false;
        if (t->has_last_pos && (now_ms - t->last_pos_ms < 120000)) {
            ref_lat = t->last_pos_lat; ref_lon = t->last_pos_lon; have_ref = true;
        } else if (s_rx_loc_set) {
            ref_lat = s_rx_lat; ref_lon = s_rx_lon; have_ref = true;
        }
        if (!have_ref) {
            s_diag.cpr_missing_ref++;
            return;
        }
        double lat, lon;
        if (cpr_local_surface_try(f->cpr_latitude, f->cpr_longitude, f->cpr_odd, ref_lat, ref_lon, &lat, &lon)) {
            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
                s_diag.cpr_local_fail++;
                s_diag.cpr_invalid++;
                return;
            }
            double dref = haversine_nm(ref_lat, ref_lon, lat, lon);
            if (dref > 45.0) {
                s_diag.cpr_local_fail++;
                s_diag.cpr_ref_outside++;
                return;
            }
            t->lat = lat; t->lon = lon; t->has_position = true;
            t->has_last_pos = true; t->last_pos_lat = lat; t->last_pos_lon = lon; t->last_pos_ms = now_ms;
            s_diag.cpr_local_success++;
            s_diag.tracks_pos_local++;
            return;
        }
        s_diag.cpr_local_fail++;
        return;
    }
    /* Airborne local CPR */
    double ref_lat = 0, ref_lon = 0;
    bool have_ref = false;
    if (t->has_last_pos && (now_ms - t->last_pos_ms < 300000)) {
        ref_lat = t->last_pos_lat; ref_lon = t->last_pos_lon; have_ref = true;
    } else if (s_rx_loc_set) {
        ref_lat = s_rx_lat; ref_lon = s_rx_lon; have_ref = true;
    }
    if (!have_ref) {
        s_diag.cpr_missing_ref++;
        return;
    }
    double lat, lon;
    if (cpr_local_airborne_try(f->cpr_latitude, f->cpr_longitude, f->cpr_odd, ref_lat, ref_lon, &lat, &lon)) {
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            s_diag.cpr_local_fail++;
            s_diag.cpr_invalid++;
            return;
        }
        double dref = haversine_nm(ref_lat, ref_lon, lat, lon);
        if (dref > 180.0) {
            s_diag.cpr_local_fail++;
            s_diag.cpr_ref_outside++;
            return;
        }
        if (t->has_position && t->gs >= 0) {
            double d2 = haversine_nm(t->lat, t->lon, lat, lon);
            double dt_s = (now_ms - t->last_seen_ms) / 1000.0;
            if (dt_s < 0.001) dt_s = 0.001;
            double max_nm = (t->gs / 3600.0) * dt_s * 3.0 + 10.0;
            if (d2 > max_nm && max_nm < 1000.0) {
                s_diag.cpr_invalid++;
                return;
            }
        }
        t->lat = lat; t->lon = lon; t->has_position = true;
        t->has_last_pos = true; t->last_pos_lat = lat; t->last_pos_lon = lon; t->last_pos_ms = now_ms;
        s_diag.cpr_local_success++;
        s_diag.tracks_pos_local++;
        return;
    }
    s_diag.cpr_local_fail++;
}

void adsb_state_update(const adsb_frame_t* f, uint32_t now_ms)
{
    if (!f || f->icao == 0 || f->icao > 0xffffff) return;
    adsb_track_t* t = find_or_add_by_addr(f->icao, f->icao_is_icao);

    if (f->has_callsign && callsign_ok(f->callsign)) {
        if (t->flight[0] == '\0' || strncmp(t->flight, f->callsign, 8) != 0) {
            memcpy(t->flight, f->callsign, 8);
            t->flight[8] = '\0';
        }
    }
    if (f->has_altitude && alt_ok(f->altitude_ft)) {
        /* For TC5-8 surface we must not store altitude as baro from position; but altitude_ft comes from parsed frame
         * - caller/state must respect cpr_surface; we still store if valid but caller should be careful */
        if (!(f->cpr_surface && f->has_cpr)) {
            t->alt_baro = f->altitude_ft;
        } else {
            /* surface position messages don't carry barometric altitude in same way - skip */
        }
    }
    if (f->has_speed && gs_ok(f->speed_kts)) t->gs = f->speed_kts;
    if (f->has_heading && hdg_ok(f->heading_deg)) t->heading = f->heading_deg;
    if (f->has_vertical_rate && vr_ok(f->vertical_rate_fpm)) {
        t->baro_rate = f->vertical_rate_fpm;
    }
    if (f->signal > 0) t->signal = f->signal;

    if (f->has_cpr) {
        const int parity = f->cpr_odd ? 1 : 0;
        /* Never mix surface/airborne parity types */
        if (t->cpr_have[parity] && t->cpr_surface[parity] != f->cpr_surface) {
            /* clear conflicting parity */
            t->cpr_have[parity] = false;
        }
        t->cpr_lat[parity] = f->cpr_latitude;
        t->cpr_lon[parity] = f->cpr_longitude;
        t->cpr_ms[parity] = now_ms;
        t->cpr_have[parity] = true;
        t->cpr_surface[parity] = f->cpr_surface;
        s_diag.cpr_even_odd[f->cpr_odd ? 1 : 0]++;
        cpr_try_pair(t, now_ms);
        if (!t->has_position) {
            cpr_try_local(t, f, now_ms);
        }
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
