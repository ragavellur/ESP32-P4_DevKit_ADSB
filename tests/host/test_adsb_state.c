/* Host unit test for adsb_state.c — proves CPR pairing, TTL expiry,
 * validation gates, cap and revision logic WITHOUT the decoder/ESP.
 * Build: cc -I main -o /tmp/test_adsb_state main/adsb_state.c test_adsb_state.c
 */
#include <stdio.h>
#include <string.h>

#include "adsb_state.h"

static int g_pair_calls;
static double g_fake_lat;
static double g_fake_lon;

static bool fake_pair(uint32_t icao, uint32_t lat_even, uint32_t lon_even,
                      uint32_t lat_odd, uint32_t lon_odd,
                      double* lat, double* lon)
{
    (void)icao;
    if (lat_even == 0 || lon_even == 0 || lat_odd == 0 || lon_odd == 0) {
        return false;
    }
    ++g_pair_calls;
    *lat = g_fake_lat;
    *lon = g_fake_lon;
    return true;
}

static int failures = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            ++failures;                                                      \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);    \
        }                                                                    \
    } while (0)

static adsb_frame_t make_frame(uint32_t icao)
{
    adsb_frame_t f;
    memset(&f, 0, sizeof(f));
    f.icao = icao;
    return f;
}

int main(void)
{
    adsb_state_init();
    adsb_state_set_cpr_pair(fake_pair);
    g_fake_lat = 52.2658;
    g_fake_lon = 3.9389;

    /* 1. Single even CPR frame alone -> no position yet. */
    {
        adsb_frame_t f = make_frame(0x4840d6);
        f.has_cpr = true;
        f.cpr_odd = false;
        f.cpr_latitude = 1000;
        f.cpr_longitude = 2000;
        f.has_callsign = true;
        strcpy(f.callsign, "KLM1023");
        f.has_altitude = true;
        f.altitude_ft = 38000;
        f.has_speed = true;
        f.speed_kts = 436;
        f.has_heading = true;
        f.heading_deg = 271;
        f.has_vertical_rate = true;
        f.vertical_rate_fpm = -64;
        f.signal = 33;
        adsb_state_update(&f, 1000);
        CHECK(g_pair_calls == 0, "single even frame must not pair yet");
        CHECK(adsb_state_live_count() == 1, "one live track");
    }

    /* 2. Odd frame within the 10 s window -> global pair + position. */
    {
        adsb_frame_t f = make_frame(0x4840d6);
        f.has_cpr = true;
        f.cpr_odd = true;
        f.cpr_latitude = 900;
        f.cpr_longitude = 1500;
        adsb_state_update(&f, 10500); /* 9.5 s after even -> in window */
        CHECK(g_pair_calls == 1, "even+odd in window must call pair once");
    }

    /* 2b. Position must have landed on the track. */
    {
        adsb_aircraft_t a;
        CHECK(adsb_state_fill(&a, 1) == 1, "fill returns track");
        CHECK(a.icao == 0x4840d6, "icao round-trip");
        CHECK(a.has_position, "position resolved");
        CHECK(a.lat == 52.2658 && a.lon == 3.9389, "lat/lon from pair fn");
        CHECK(strcmp(a.flight, "KLM1023") == 0, "callsign stored");
        CHECK(a.alt_baro == 38000, "altitude stored");
        CHECK(a.gs == 436, "ground speed stored");
        CHECK(a.heading == 271, "heading stored");
        CHECK(a.baro_rate == -64, "vertical rate stored");
        CHECK(a.signal == 33, "signal stored");
    }

    /* 3. Odd frame coming >10 s after even -> must NOT pair. */
    {
        adsb_frame_t f = make_frame(0x4840d6);
        f.has_cpr = true;
        f.cpr_odd = true;
        f.cpr_latitude = 901;
        f.cpr_longitude = 1501;
        adsb_state_update(&f, 30000); /* 29 s after even -> stale */
        CHECK(g_pair_calls == 1, "stale pair window must reject");
    }

    /* 4. Validation gates: garbage callsign rejected, out-of-range fields. */
    {
        adsb_frame_t f = make_frame(0x4840d6);
        f.has_callsign = true;
        strcpy(f.callsign, "\x01GARBAGE"); /* control char */
        f.has_altitude = true;
        f.altitude_ft = 9999999; /* implausible */
        f.has_speed = true;
        f.speed_kts = 99999;
        f.has_heading = true;
        f.heading_deg = -40;
        adsb_state_update(&f, 31000);
        adsb_aircraft_t a;
        CHECK(adsb_state_fill(&a, 1) == 1, "track survives gates");
        CHECK(strcmp(a.flight, "KLM1023") == 0, "bad callsign ignored");
        CHECK(a.alt_baro == 38000, "bad altitude ignored");
        CHECK(a.gs == 436, "bad speed ignored");
        CHECK(a.heading == 271, "bad heading ignored");
    }

    /* 5. Separate ICAOs create separate tracks. */
    {
        adsb_frame_t f = make_frame(0xabcdef);
        f.has_callsign = true;
        strcpy(f.callsign, "IND4781");
        adsb_state_update(&f, 40000);
        CHECK(adsb_state_live_count() == 2, "second track added");
    }

    /* 6. TTL expiry: simulate 61 s silence -> first track expires. */
    {
        adsb_state_update(&(adsb_frame_t){.icao = 0xabcdef}, 41000);
        adsb_state_update(&(adsb_frame_t){.icao = 0x4840d6}, 41000);
        /* bump both to keep alive past the expiry check below */
        adsb_state_update(&(adsb_frame_t){.icao = 0x4840d6}, 41500);
        CHECK(adsb_state_expire(103000) == 2,
              "both idle > TTL (61s/62s) expire in one pass");
        adsb_aircraft_t a;
        CHECK(adsb_state_fill(&a, 1) == 0, "all tracks expired");
    }

    /* 7. Cap: 64 unique ICAOs is the hard ceiling. */
    {
        adsb_state_init();
        for (uint32_t i = 0; i < ADS_STATE_MAX_TRACKS + 8; ++i) {
            adsb_frame_t f = make_frame(0x1000 + i);
            adsb_state_update(&f, 1000 + i);
        }
        CHECK(adsb_state_live_count() == ADS_STATE_MAX_TRACKS,
              "track count capped at 64");
    }

    /* 8. Revision advances on each update of the same track. */
    {
        adsb_state_init();
        adsb_frame_t f = make_frame(0x123456);
        adsb_state_update(&f, 1000);
        adsb_aircraft_t a1, a2;
        adsb_state_fill(&a1, 1);
        adsb_state_update(&f, 2000);
        adsb_state_fill(&a2, 1);
        CHECK(a2.revision > a1.revision, "revision increments per update");
    }

    if (failures == 0) {
        printf("PASS: all adsb_state host tests OK\n");
        return 0;
    }
    printf("FAIL: %d assertion(s) failed\n", failures);
    return 1;
}