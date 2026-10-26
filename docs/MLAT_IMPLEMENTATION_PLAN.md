# Native MLAT Client Implementation Plan

## Baseline (Phase A) — Facts

### Target/environment
- ESP32-P4, ESP-IDF (target esp32p4), rev less v3, Flash 2MB (DIO 40MHz)
- Components: `components/adsb_decoder/`, `components/esp_rtl_sdr/`
- Build: `source ~/esp/esp-idf/export.sh && idf.py build`
- Main task priorities: driver/DSP on core1, feeder/HTTP/status on core0; see `docs/LEARNING.md` L24.

### Existing timing (CRITICAL)
- `adsb_decoder_bridge.cpp:89` sets `f.rx_timestamp_us = (int64_t)esp_timer_get_time()` at decode time (in `on_frame` callback), then copies to `s_capture` and passes to hook.
- `adsb_captured_frame_t` and `adsb_frame_info_t` both have `rx_timestamp_us` (decode-time).
- `rtl_pipeline.c:395` calls `beast_feeder_push(f->bytes, f->bit_length, f->rx_timestamp_us)` from the decoder hook (synchronously on `adsb_dsp` core1/prio6).
- `beast_feeder.c:263`: `uint64_t ts = (uint64_t)s->rx_timestamp_us * 12u;` — **multiplies decode-time microseconds by 12 to get nominal 12MHz units**. Hub uses relative deltas; this is NOT a true sample-timestamp. Matches the problem described.
- `esp_rtl_sdr_iq_block_t` has `host_timestamp_us` (from esp_timer at event time) — currently unused for per-frame timing.
- No continuous sample counter, no block-start sample index, no frame-position-within-block tracking in the pipeline/decoder hook chain.
- Decoder processes 64KB IQ blocks in chunks (overlap carried across) at 2.4MSPS; variable DSP scheduling/queue delays mean decode-time timestamps are inaccurate for MLAT.

### IQ pipeline
- 8 slots × 64KB PSRAM, SPSC queues (free/filled). `rtl_pipeline.c:449` checks `blk->bytes > IQ_BLOCK_BYTES`. Consumer: `adsb_dsp` task takes filled blocks via `xQueueReceive(filled_q, &idx, portMAX_DELAY)`, calls `adsb_decoder_bridge_process(slot->data, slot->bytes)`, returns slot to free_q.
- Drop-on-full at ring (`ring_drops` counter). Driver pushes via `on_iq_block()` callback.

### Decoder
- C++ `adsb_radar::adsb_rx::Decoder` behind C bridge. `process_cu8()` splits large blocks into `process_pairs()` chunks with overlap carried across. CRC-fix is O(1) table-based. Hook `adsb_frame_hook_t` called per CRC-valid frame. Bridge logs on specific TC and copies raw bytes/bit_length for feeder.

### BEAST feeder (existing)
- Native C, separate core0 prio3 task, lock-free SPSC ring (256 slots) between hook (decode task) and feeder task; drop-on-full.
- BEAST: `0x1A`, type `0x32` (7B DF11) or `0x33` (14B DF17), 6-byte big-endian 12MHz timestamp, RSSI `0xFF` (none), doubles every `0x1A`. TCP to `feed.bharatradar.com:30004`, handshake `0x1A 0xE4 <UUID>` (readsb-style), no pong/location. NVS config `radar/feed_{en,uuid,name,host,port}`. `/api/feeder` GET/POST.

### Upstream mlat-client (mutability/mlat-client)
- **Protocol**: JSON-over-TCP to MLAT server (default port varies by service). Client sends JSON lines; server responds with JSON lines. Optional zlib compression for TCP writes, optional UDP transport for MLAT messages.
- **Handshake**: Client sends `{"init":{"version":<int>,"client_version":"...","selective_traffic":bool,"return_results":bool,"udp_transport":int,"compress":int,"lat":f,"lon":f,"alt":f,"clock_type":"...","clock_frequency":f,"clock_epoch":int,"clock_jitter":f,"clock_accuracy":f,"privacy":bool,"split_syncs":bool,"connection_id":"..."},...}` (see `jsonclient.py:JsonServerConnection` and coordinator). Server replies with `{"mlat":{"type":"start"/"deny"/...}}` and may offer UDP key/port.
- **Message submission (JSON)**: `{"mlat":{"t":<timestamp>,"m":"<hex>"}}` where `t` is the timestamp in the clock units the client declared (12MHz for Beast/Radarcape style; see modes_reader modes_message timestamp as "12MHz timestamp"), `m` is hex of the raw Mode-S message bytes.
- **Sync messages**: `{"sync":{"et":...,"em":"...","ot":...,"om":"..."}}, {"ssync":{"t":...,"m":"..."}}` depending on mode (cooperative MLAT).
- **Clock reports / clock type**: Client declares clock characteristics in init; may send clock error estimates. UDP transport uses custom binary framing with delta encoding, rebase, sync/mlat/ssync types (struct formats in jsonclient.py).
- **Keepalive**: heartbeat interval 120s, inactivity timeout 60s; reconnect 30s.
- **Results**: Server may send position results back if `return_results` enabled.
- **Selection/filtering**: coordinator/receiver decide which messages to submit (uses message type, etc.). Messages come from receiver adapters; mlat-client has Beast, AVR, Radarcape, SBS, etc. readers.
- **Clock**: timestamps must be monotonic, ideally from a stable free-running clock. For MLAT, absolute timebase matters less than stability and accurate relative timing across receivers; the protocol carries timestamps in the declared clock units.

**Critical for our port**: We must NOT use decode-time timestamps. We need reception-time timestamps in appropriate units (12MHz-like or whatever the server expects). The upstream expects timestamps tied to the receiver's sampling/decoding clock at reception time.

## Key gaps for native ESP32-P4 MLAT
1. **Timing (blocker for correctness)**: No sample-index tracking through IQ blocks. `host_timestamp_us` exists but is set at USB/driver event time (after USB transfer completes) — also has latency/jitter; not the same as sample arrival. To get true reception time we need: continuous sample counter at acquisition, know preamble position within sample stream (and within block), account for DSP filtering/demod delays. The current decoder finds preambles but doesn't expose their sample offsets to the bridge/hook.
2. **Protocol**: Need JSON+TCP client matching mutability protocol; optional UDP harder on constrained network/tasking. Must handle handshake, mlat/sync messages, keepalive, reconnect, zlib if offered, parsing.
3. **Message selection**: Only certain DF/frames useful for MLAT (e.g., DF11, DF17, DF18 in some cases) — upstream filters.
4. **Integration**: Must not block IQ/DSP; use separate task/queue; don't break existing BEAST feeder or web UI.
5. **Config**: Add MLAT settings (enabled, server, lat/lon/alt, callsign/station ID, etc.) following NVS patterns like beast_feeder.

## Minimal viable approach (phased)
### Phase B — Timing first
- Add sample counter in `rtl_pipeline`: increment by samples in each IQ block as they are enqueued/processed (track cumulative samples since start). Store `block_start_sample_index` and `block_start_host_us` (from `host_timestamp_us` or esp_timer) with each slot or pass through.
- Modify decoder bridge to capture preamble/frame position? Requires decoder changes to report sample offset of detected frame. Alternatively, estimate from when frame is processed relative to block start + known processing — still has DSP latency. Better: track in pipeline by correlating — complex.
- Alternative: Use `host_timestamp_us` from IQ block as `block_arrival_time`, subtract estimated processing latency based on block position? Still approximate. Or use a calibration.
- **Pragmatic**: For each IQ block passed to decoder, record `block_start_sample` and `block_time_us` (prefer `host_timestamp_us`, else `esp_timer_get_time()`). In the bridge, when a frame comes via hook, we know it came from processing that block context — but we don't know which block. Need to thread block context through.
- Change: pass block metadata (start_sample, time_us, sample_rate) to `adsb_decoder_bridge_process()` or store in thread-local/context; have bridge tag frames with estimated timestamp = `block_time_us + sample_offset/sample_rate`. To get sample_offset need decoder to report it.
- This requires touching `adsb_decoder` component (C++) to expose preamble/sample position. Inspect decoder code.

### Phase C/D — MLAT client component
- Create `components/mlat_client/` or `main/mlat_client.c/h` following project style.
- JSON client using lwIP sockets (like beast_feeder). Separate FreeRTOS task, bounded queue of timestamped frames from decoder hook (or from pipeline), non-blocking.
- Handshake per mutability protocol, support TCP (JSON); UDP optional/later if needed.
- Clock: use a monotonic clock; for 12MHz units compute from sample-derived timestamps calibrated to actual sample rate.
- Message format: DF11 (7 bytes) and DF17 (14 bytes) per upstream filtering; submit hex in JSON `m` field with timestamp `t`.
- Config in NVS under `radar/mlat_*`, API endpoints `/api/mlat` similar to feeder, optional UI section later only if necessary.

### Risks/notes
- Timing accuracy is the hard part; may need calibration. Document limitations.
- Must not block decoder task.
- Licensing: mlat-client is GPLv3; ESP32 firmware distribution implications — document.
- No credentials/hardware measurements available now — implement protocol fully, test with mocks/logs, document what's needed for live validation.
