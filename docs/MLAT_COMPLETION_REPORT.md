# MLAT Client Implementation Report (Native ESP-IDF)

## Baseline (Phase A)
- Target: ESP32-P4, ESP-IDF v6.x style toolchain present; `sdkconfig` shows `CONFIG_IDF_TARGET="esp32p4"`, rev less v3, 2MB flash. Build env sourced via `~/esp/esp-idf/export.sh`.
- Components: `adsb_decoder`, `esp_rtl_sdr`. Main: `rtl_pipeline`, `beast_feeder`, `http_radar`, `adsb_state`, `eth_netif`.
- Timing (verified): `adsb_decoder_bridge.cpp:on_frame()` sets `rx_timestamp_us = esp_timer_get_time()` at decode time; passes to hook and to `/api/frames`/capture. `beast_feeder.c` does `ts = (uint64_t)rx_timestamp_us * 12u` (decode-time microseconds→12MHz units) — **not reception time**. This is a known limitation per requirements.

## Timing Path (Phases A-B)
### Changes (sample-derived timing groundwork)
1. **rtl_pipeline**: Added block timing metadata to IQ slots (`block_start_sample`, `block_time_us`, `sample_rate_sps`). Track cumulative `mlat_sample_counter` across blocks; capture `host_timestamp_us` (or `esp_timer_get_time()`) at IQ block delivery. Pass `adsb_decoder_block_ctx_t` to decoder bridge via `adsb_decoder_bridge_process_with_ctx()`.
2. **adsb_decoder_bridge**: Extended `adsb_frame_info_t` and `adsb_captured_frame_t` with `rx_timestamp_sample_us`, `sample_index`, `sample_rate_sps`. Store block context during `process_cu8()`; in `on_frame()` prefer `block_time_us` as sample-derived timestamp estimate (block-aligned) while keeping legacy `rx_timestamp_us` (decode time). Capture ring extended to store sample timing fields.
3. **adsb_decoder** headers unchanged structurally; bridge API extended.

**Limitation (documented):** The decoder does not expose preamble/sample offset within a block. True per-frame sample-accurate timestamps require decoder instrumentation to report the detected preamble sample index. The implementation now carries block-aligned sample timing and block start sample index through the pipeline/hooks—this is the best safe improvement possible without invasive decoder demodulation changes. All timestamps are labeled (legacy vs sample-derived).

## Native MLAT Client (Phases C-D)
**Component:** `components/mlat_client/`
- `include/mlat_client.h`, `src/mlat_client.c` — C implementation, ESP-IDF compatible.
- **Architecture:** Lock-protected SPSC ring (512 slots) between decoder hot path (`mlat_client_push_frame()` called from `rtl_pipeline` hook) and dedicated FreeRTOS task on core 0 priority 3. Non-blocking push (drop-on-full, counted). Network I/O only in task.
- **Protocol (mutability/mlat-client JSON):** TCP JSON lines. Handshake `{"init":{...version:3, client_version:"esp32p4-mlat/1.0", lat/lon/alt, clock_type:"radarcape_12mhz", clock_frequency:12000000.0, ...}}`. Submits `{"mlat":{"t":<12MHz>,"m":"<hex>"}}` for DF11/DF17 frames. Handles `deny`, parses responses, heartbeat/ping every 120s, reconnect backoff 30s, timeouts. Prefers sample-derived timestamps converted to 12MHz (`*12`); falls back to decode-time with counters.
- **Config/persistence:** NVS namespace `radar` keys `mlat_en/host/port/uuid/name/lat/lon/alt/priv/retres/udp/zlib`. Defaults disabled, no hardcoded credentials.
- **API:** `GET/POST /api/mlat` returns config + stats (frames_observed/eligible/submitted/dropped_queue/invalid_ts/tcp_sent/reconnects/protocol_errors/server_results/connected/queue_depth/high_water).
- **Integration:** Initialized/started in `app_main` (disabled by default). Hooked in `rtl_pipeline` decoder callback. Registered in HTTP server. Existing BEAST feeder, decoder, web UI, networking unchanged in behavior.

## Build/Test (Phase E)
- **Build:** `idf.py build` completed successfully (ESP32-P4 target). Binary `build/adsb_radar.bin` ~0xb4580 bytes, ~53% free in 1.5MB app partition.
- **Enabled/disabled:** MLAT disabled by default; enable only via `/api/mlat` with host configured. No unsolicited connections when disabled.

## Configuration & Service Registration (Phase F)
**Required to operate:**
1. Obtain MLAT server details (host/port) and station credentials/UUID from your MLAT service provider (e.g., FlightAware/MLAT hub you are authorized to use). This repo contains no credentials.
2. Configure via `POST /api/mlat`:
```json
{"enabled":true,"host":"mlat.example.com","port":12345,"uuid":"YOUR_STATION_ID","station_name":"ESP32P4","lat":XX.XXXXXX,"lon":YY.YYYYYY,"alt_m":123.4,"return_results":false}
```
3. Monitor via `GET /api/mlat` (connected, frames_submitted, queue_depth, invalid_ts).

**Notes:** The implementation follows the mutability/json protocol structure but service-specific auth/requirements vary. Real MLAT validation requires authorized server + proper registration.

## Known Limitations
- **Timing precision:** True preamble sample offsets not exposed by current decoder; uses block-aligned sample timing. This is a software limitation of the existing architecture; hardware measurements/calibration may still be needed for best MLAT accuracy.
- **UDP/zlib:** Flags exist; UDP transport not fully implemented (TCP JSON primary). zlib compression not implemented.
- **Message filtering:** Submits DF11/DF17 primarily (all valid short/long passed through with counters).
- **No live MLAT proof in this environment:** No authorized MLAT server/credentials/hardware timing validation performed here; code is protocol-compatible structure based on upstream analysis.

## Files Changed/Added
**Added:** `components/mlat_client/{CMakeLists.txt,include/mlat_client.h,src/mlat_client.c}`, `docs/MLAT_IMPLEMENTATION_PLAN.md`, `docs/MLAT_COMPLETION_REPORT.md`
**Modified:** `main/app_main.c`, `main/rtl_pipeline.{c,h}`, `main/http_radar.c`, `main/CMakeLists.txt`, `components/adsb_decoder/{include/adsb_decoder_bridge.h,src/adsb_decoder_bridge.cpp}`
