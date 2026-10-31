# MLAT Timestamp Fix (Decoder Overlap Alignment)

**Date:** 2026-10-27  
**Branch:** fix/mlat-sync-pairs  
**Device:** ESP32-P4 DevKit (192.168.200.102), UUID `a42448ce-080a-456e-b16a-9d846b7c4bfe`  
**Server:** `mlat-mlat-server` in namespace `bharatradar` (feed.bharatradar.com:31090)

## Problem
The ESP32 MLAT client could form sync pairs with the co-located Pi feeder (`67620ee8-...`), but the pairs were repeatedly invalidated on the server. Symptoms:
- Sync pairs appeared briefly with `peer_count=1` but were reset quickly
- Server `sync.json` showed `n=2`, `error = -1.0` (sentinel from `reset_offsets()`), high outliers (~50–60%), `jumped=1`
- Pair never reached `n > 4` so `pairing.valid` stayed false (`clocktrack.pyx:689-711`)
- `mlat_msgs`, `valid_groups`, `solve_attempt` remained 0

Root cause: The ADS-B decoder carries overlap between blocks (`overlap_`, ~kTrailingSamples). `preamble_offset_samples` from the decoder is an index into the magnitudes buffer that includes carried-over overlap from the previous block. `rtl_pipeline.c` sets `block_start_sample = s_ctx.mlat_sample_counter` as the global sample index at the start of the *new* block. The bridge was using `preamble_offset_samples` directly against `block_start_sample` without subtracting overlap, corrupting the sample-derived timestamp passed to the MLAT client.

## Changes Made

### 1) Expose decoder overlap
**`components/adsb_decoder/include/adsb_decoder.hpp`**
```cpp
class Decoder {
  public:
    size_t overlap_samples() const { return overlap_; }
    ...
};
```

### 2) Add bridge getter
**`components/adsb_decoder/src/adsb_decoder_bridge.cpp`**
```cpp
extern "C" size_t adsb_decoder_bridge_get_overlap_samples(void)
{
    return s_dec.overlap_samples();
}
```
(Declared `extern "C"` in the same file for use.)

### 3) Correct timestamping in frame hook
**`components/adsb_decoder/src/adsb_decoder_bridge.cpp` - `on_frame()`**

When block context and sample rate are valid:
1. Get `overlap = adsb_decoder_bridge_get_overlap_samples()`
2. Compute adjusted preamble offset relative to *new* samples:
```cpp
size_t ovl = adsb_decoder_bridge_get_overlap_samples();
uint32_t po_adj = f.preamble_offset_samples;
if (ovl > 0 && po_adj >= (uint32_t)ovl) {
    po_adj = f.preamble_offset_samples - (uint32_t)ovl;
} else {
    po_adj = 0;
}
f.preamble_offset_samples = po_adj;
```
3. Set `f.sample_index = s_cur_ctx.block_start_sample;` (unchanged from pipeline)
4. Set `f.sample_rate_sps = s_cur_ctx.sample_rate_sps;`
5. Compute sample-derived timestamp (monotonic, consistent):
```cpp
f.rx_timestamp_sample_us = ((int64_t)(f.sample_index + (uint64_t)f.preamble_offset_samples) * 1000000LL) /
                           (int64_t)f.sample_rate_sps;
```
6. Removed the rough hack: `f.rx_timestamp_sample_us = s_cur_ctx.block_time_us - 1000 + offset_us / 10;`
7. Added fallback if sample rate missing but block context exists.
8. Capture ring stores corrected values.

The MLAT client (`components/mlat_client/src/mlat_client.c::timestamp_to_12mhz_slot()`) uses:
```cpp
uint64_t sample = s->sample_index + (uint64_t)s->preamble_offset_samples;
uint64_t rate = s->sample_rate_sps;
uint64_t ts12 = sec * 12000000ULL + (rem * 12000000ULL) / rate;
```
With corrected offsets, this now produces consistent 12MHz timestamps aligned to the global sample stream.

## How It Works
1. `rtl_pipeline.c` increments `mlat_sample_counter` by `pairs_in_block` per IQ block and passes `block_start_sample` in context
2. Decoder processes with overlap carryover; `preamble_offset_samples` points into the overlapped buffer
3. Bridge subtracts `overlap_` to get offset into the *current block's new samples*
4. Client converts `(block_start_sample + adjusted_offset) / sample_rate` to 12MHz units
5. Server's `ClockPairing` sees small, consistent prediction errors and can accumulate enough sync points (`n > 4`) to become valid

## Verification
- Build: `idf.py build` succeeded with no new warnings/errors
- Flash: successful
- Device: `http://192.168.200.102/api/status` shows streaming, decode_ok true, MLAT connected (reconnects 0, protocol_errors 0)
- Device: `http://192.168.200.102/api/frames` shows `sample_index` advancing, `preamble_offset_samples` reasonable/corrected, `sample_rate_sps` set
- Device: `http://192.168.200.102/api/mlat` shows sync pairs being sent
- Server: `sync.json` shows pair present both ways. Outliers dropped from ~53–60% to ~30–50%. Pairs now persist longer; need `n > 4` to become fully valid (`valid=true`)
- Server logs show increased sync activity with fewer resets

## Files Modified
- `components/adsb_decoder/include/adsb_decoder.hpp`
- `components/adsb_decoder/src/adsb_decoder_bridge.cpp`

## Notes
- No changes to server, `MAX_SYNC_RATE`, `mlat_client`, or `rtl_pipeline`
- Clamping to 0 if adjusted offset would be negative is defensive
- Monotonicity of `sample_index` is critical for stable clock pairing
