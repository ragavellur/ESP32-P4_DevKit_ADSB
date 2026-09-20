#pragma once

#include "rtl_control.hpp"

/*
 * Provisional, non-invasive R820T2 identity trace (register 0 chip-id).
 *
 * Blog V3 streaming (0.8.0-rc3) is experimental/provisional: when selected, the
 * driver reuses the evidence-backed R820T2 USB IR value remap (0x74→0x34) that
 * Nooelec provisional uses for the same tuner-address template — not invented
 * V3-unique silicon init tables. No first-party V3 capture exists yet; this is
 * community hardware soak, not Hardware-verified.
 */
constexpr RtlControlRecord kBlogV3ProbeSelect = {
    0x0034, 0x0610, 0x40, 1, {0x00, 0, 0, 0, 0, 0, 0, 0}};
constexpr RtlControlRecord kBlogV3ProbeRead = {
    0x0034, 0x0600, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}};

/* First-party V3c Q-branch transition, captured 2026-09-14. */
constexpr RtlControlRecord kBlogV3DirectEnable[] = {
    {0xb120, 0x0011, 0x40, 1, {0x1a, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0x1520, 0x0011, 0x40, 1, {0x00, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0x0820, 0x0010, 0x40, 1, {0x4d, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0x0620, 0x0010, 0x40, 1, {0x90, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
};

constexpr RtlControlRecord kBlogV3DirectDisable[] = {
    {0x1520, 0x0011, 0x40, 1, {0x01, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0x0620, 0x0010, 0x40, 1, {0x80, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
};

/* Captured before restoring the R820T2/R860 register block. */
constexpr RtlControlRecord kBlogV3TunerRepeaterOn[] = {
    {0x0120, 0x0011, 0x40, 1, {0x18, 0, 0, 0, 0, 0, 0, 0}},
    {0x0120, 0x000a, 0xc0, 1, {0, 0, 0, 0, 0, 0, 0, 0}},
};
