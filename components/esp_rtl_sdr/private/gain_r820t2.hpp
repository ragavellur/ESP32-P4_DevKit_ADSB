#pragma once

/**
 * R820T2/R860 manual gain stage sequence. The advertised gain values are
 * nominal; register 05 selects the LNA stage and register 07 the mixer stage.
 *
 * Anchors (real, hardware-confirmed, 2026-09-11):
 *   0.0 dB  -> reg05=0x90 reg07=0x60 (observed default/idle state on every
 *              real V3c and Blog V4 session immediately after tuner init,
 *              before any gain change is requested)
 *   49.6 dB -> reg05=0x9f reg07=0x6e (official RTL-SDR Blog driver,
 *              rtl_sdr.exe -g 49.6, single continuous session, confirmed
 *              settled and held with ZERO further register writes for
 *              4+ minutes -- the one fully unambiguous data point from
 *              this investigation)
 *
 * The previous linear interpolation was disproved on V3c hardware: a stable
 * 99.1 MHz signal clipped at low requests, then weakened as the requested
 * gain increased. These pairs follow the R820T2's non-linear alternating
 * LNA/mixer stage sequence. Absolute gain remains nominal until calibrated
 * against a controlled RF source.
 */

#include <cstddef>
#include <cstdint>

struct R820T2GainStep {
    int tenth_db;
    uint8_t reg05;
    uint8_t reg07;
};

constexpr R820T2GainStep kR820T2GainSteps[] = {
    {0, 0x90, 0x60},     // 0.0 dB  -- anchor
    {9, 0x91, 0x60},     // 0.9
    {14, 0x91, 0x61},    // 1.4
    {27, 0x92, 0x61},    // 2.7
    {37, 0x92, 0x62},    // 3.7
    {77, 0x93, 0x62},    // 7.7
    {87, 0x93, 0x63},    // 8.7
    {125, 0x94, 0x63},   // 12.5
    {144, 0x94, 0x64},   // 14.4
    {157, 0x95, 0x64},   // 15.7
    {166, 0x95, 0x65},   // 16.6
    {197, 0x96, 0x65},   // 19.7
    {207, 0x96, 0x66},   // 20.7
    {229, 0x97, 0x66},   // 22.9
    {254, 0x97, 0x67},   // 25.4
    {280, 0x98, 0x67},   // 28.0
    {297, 0x98, 0x68},   // 29.7
    {328, 0x99, 0x68},   // 32.8
    {338, 0x99, 0x69},   // 33.8
    {364, 0x9a, 0x69},   // 36.4
    {372, 0x9a, 0x6a},   // 37.2
    {386, 0x9b, 0x6a},   // 38.6
    {402, 0x9b, 0x6b},   // 40.2
    {421, 0x9c, 0x6b},   // 42.1
    {434, 0x9c, 0x6c},   // 43.4
    {439, 0x9d, 0x6c},   // 43.9
    {445, 0x9d, 0x6d},   // 44.5
    {480, 0x9e, 0x6d},   // 48.0
    {496, 0x9f, 0x6e},   // 49.6 -- anchor
};

constexpr size_t kR820T2GainStepCount =
    sizeof(kR820T2GainSteps) / sizeof(kR820T2GainSteps[0]);

inline size_t r820t2_nearest_gain_index(int tenth_db)
{
    size_t best = 0;
    int64_t best_err = INT64_MAX;
    for (size_t i = 0; i < kR820T2GainStepCount; ++i) {
        const int64_t err =
            static_cast<int64_t>(tenth_db) - kR820T2GainSteps[i].tenth_db;
        const int64_t aerr = err < 0 ? -err : err;
        if (aerr < best_err) {
            best_err = aerr;
            best = i;
        }
    }
    return best;
}
