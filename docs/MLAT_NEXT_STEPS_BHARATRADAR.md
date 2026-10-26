# Next Steps: BharatRadar MLAT (Corrected Plan)

## Context
- **Existing feeder**: This firmware already feeds to BharatRadar via the native BEAST feeder (`main/beast_feeder.c`). It connects to `feed.bharatradar.com:30004`, uses BEAST framing (`0x1A 0x32`/`0x33`), 6-byte 12MHz timestamp (`rx_timestamp_us * 12`), RSSI `0xFF`, UUID-based handshake (`0x1A 0xE4 <UUID>`). Configured via `/api/feeder` with UUID stored in NVS.
- **BharatRadar feeders repo**: https://github.com/bharatradar/feeders (scripts for Raspberry Pi: `bharatradar-feeder` installs readsb+mlat-client, `feeder-self-register.sh` looks up registration via `my.bharatradar.com`).
- **Current MLAT client**: Native implementation added to this ESP32-P4 firmware following mutability/mlat-client JSON protocol, disabled by default, with sample-derived timing groundwork.

## Phase 1: Regression Testing (Immediate - Can Do Now)
Verify existing functionality remains intact after MLAT integration.

| Task | Priority | Notes |
|---|---|---|
| **Flash with MLAT disabled** | P0 | Flash the newly built firmware. MLAT defaults to disabled. |
| **Verify ADS-B decoding** | P0 | Check `/data/aircraft.json`, `/api/status`, serial logs. Ensure aircraft tracking unchanged. |
| **Verify BharatRadar BEAST feeder** | P0 | Check `/api/feeder` - confirm existing UUID/config intact. Watch serial for `beast_feed` connection status. Feeder must continue working unchanged. |
| **Verify web UI** | P0 | Load UI, check for console errors. No UI changes were made. |
| **Test MLAT API** | P1 | `GET /api/mlat` returns defaults (disabled). `POST /api/mlat` validates and persists config. |
| **Resource monitoring** | P1 | Watch heap/tasks - MLAT task on core 0 priority 3 with separate queue shouldn't impact DSP on core 1. |

## Phase 2: Timestamp Accuracy Improvements (Critical - Code Changes)
True MLAT requires accurate reception timestamps. Current implementation uses block-aligned estimates.

| Task | Priority | Notes |
|---|---|---|
| **Expose preamble sample offsets from decoder** | P0 | Modify `adsb_decoder` (C++) to report the sample index/offset of detected preamble within the processed buffer in the frame callback. |
| **Thread offsets through bridge/pipeline** | P0 | Extend `adsb_frame_info_t`/bridge to carry preamble offset, compute `reception_us = block_time_us + offset_us` using actual sample rate. |
| **Prefer sample-derived over decode-time** | P0 | Make sample-derived timestamps primary; only fall back to decode-time with explicit counters/metrics. |
| **Validate timing** | P1 | Test with known signals/recorded IQ if possible. Document precision. |
| **Track timestamp quality in stats** | P1 | Add counters for "good sample-derived" vs "fallback decode-time" timestamps to `/api/mlat` stats. |

**Note:** This is the main technical blocker for useful MLAT results regardless of which MLAT network is used.

## Phase 3: BharatRadar MLAT Service Integration (Requires User Action)
BharatRadar operates its own MLAT network. Follow their process, not third-party FlightAware/PlaneFinder.

| Task | Priority | Notes |
|---|---|---|
| **Check existing registration** | P0 | Use `feeder-self-register.sh` logic or check on BharatRadar. Your feeder UUID is configured in `/api/feeder` (same UUID used for BEAST feeding). |
| **Register/link MLAT if needed** | P0 | Visit https://my.bharatradar.com, log in, and add/link your station UUID to enable MLAT contribution per BharatRadar's process. Some setups auto-register; others require manual linking. |
| **Configure MLAT on device** | P0 | Set `POST /api/mlat` with BharatRadar MLAT server host/port, same station UUID/identity, accurate lat/lon/alt from your installation. **Do not use hardcoded credentials** - configure via API/NVS. |
| **Test connectivity** | P0 | Enable MLAT, monitor `/api/mlat`: `connected`, `frames_submitted`, `reconnects`, `protocol_errors`. Watch device logs. |
| **Verify contribution** | P0 | Check BharatRadar's site/map/stats for your station showing MLAT activity. TCP connection alone is not proof. |

**Reference:** https://github.com/bharatradar/feeders - follow BharatRadar's current recommended setup for MLAT configuration/endpoints.

## Phase 4: Hardening & Documentation (Ongoing)
| Task | Priority | Notes |
|---|---|---|
| **Protocol tuning** | P2 | Adjust to match BharatRadar's specific MLAT server requirements if they differ from generic mutability protocol (check their mlat-client usage). |
| **UDP/compression** | P2 | Implement only if BharatRadar MLAT requires it. |
| **Licensing review** | P1 | mlat-client is GPLv3 - document implications for distribution as noted in completion report. |
| **Update docs** | P1 | Add BharatRadar-specific MLAT setup instructions to README/docs with registration steps. |

## Immediate Recommendation
1. **Phase 1 regression test first** - flash and verify BEAST feeding + decoding unchanged.
2. **Tackle Phase 2 timestamp improvements** - highest technical priority for MLAT usefulness.
3. **Follow BharatRadar's exact registration process** (Phase 3) using your existing feeder UUID; do not follow generic FlightAware instructions.
