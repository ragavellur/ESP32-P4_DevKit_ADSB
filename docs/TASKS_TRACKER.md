# ADS-B Radar — Task Tracker

> Tracking status: **Not Started** / **In Progress** / **Blocked** / **Completed** / **Cancelled**
> Update the `Status` cell of every task as work proceeds.

## Summary

| Phase | Description | Status |
|---|---|---|
| Phase 0 | Board bring-up (toolchain, ports, C6 ESP-Hosted firmware) | Complete (P4 boots; **board has NO C6 — see N6**) |
| Phase 1 | Project skeleton (IDF project, sdkconfig, vendored components) | Complete |
| Phase 2 | RTL-SDR driver bring-up (enumerate, stream 2.048 MSPS IQ) | Complete — streaming, zero drops |
| Phase 3 | ADS-B decoder (port decoder, self_check, hook IQ) | Complete — pipe validated, RF-limited |
| Phase 4 | Aircraft track store (CPR pairing, TTL, fields) | Complete |
| Phase 5 | Networking (Ethernet EMAC + IP101GRI PHY, netif, DHCP) | Complete — link up 100 Mbps, DHCP 192.168.200.102 |
| Phase 6 | Web server + radar page (routes, aircraft.json, /api/status) | In Progress — end-to-end verified, feedback tweaks remain |
| Phase 7 | Core optimization & validation | Not Started |

## Platform notes (found by trial, save future hours)

| # | Note | Value in sdkconfig |
|---|---|---|
| N1 | This DevKit's ESP32-P4 is silicon **rev v1.3** — build with `ESP32P4_SELECTS_REV_LESS_V3=y`, else esptool rejects the bootloader | `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`, `CONFIG_ESP32P4_REV_MIN_100=y` |
| N2 | rev<3.0 P4 does NOT support 400 MHz CPU — must use 360 | `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=360` |
| N3 | Driver's USB probe on CPU0 starves IDLE0 ~5 s → task WDT; disable CPU0 idle check | `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=n` |
| N4 | Wait for `EVT_READY`/`device_present`, NOT state `IDLE`, before `start()` | rtl_pipeline.c |
| N5 | Pipeline streams clean (`eff≈2.048M over=0 drops=0`, `mag=[0..]`); 0 valid frames = RF-limit (window antenna) — `AIRCRAFT` log line prints on any CRC-ok frame | rtl_pipeline.c:44 dB manual + bridge on_frame log |
| N6 | **Waveshare ESP32-P4-ETH has NO ESP32-C6 onboard** — verified against official schematic PDF (zero C6/SDIO/antenna references; GPIO18–23 go to 40-pin headers J1/J2, GPIO54 to J3). All ESP-Hosted/CP work was chasing a chip that isn't there. WiFi is impossible; use the Ethernet (IP101GRI RMII). | main/eth_netif.c; sdkconfig.defaults |
| N7 | `esp_eth_driver_install()` in this IDF takes **2 args** (ETH_DEFAULT_CONFIG + out handle), and bringing RMII ref-clock low plus a 2 ms `vTaskDelay` before reading PHY ID avoids an init race | main/eth_netif.c:PHY_ID read |
| N8 | 2026-09-19: 0 frames is **RF, not DSP** — proven by a temporary FM probe: tuned to 98.3 MHz the health metric flipped to `RF_CLIPPING` (sample_max≥250) while 1090 MHz stays `mag=[0,36]` (quiet). Chain, tuner LO, gain (49.6 dB confirmed applied), stream (over=0 drops=0) all verified. Left open: "no aircraft in range at this antenna/position" vs "antenna wrong/misconnected for L-band" vs physical access to a UHF/L-band strong source for confirmation. | gain 496 set in main/rtl_pipeline.c |

## Open decisions (resolve before Phase 3)

| # | Decision | Options | Status |
|---|---|---|---|
| D1 | Decoder license path | OrcSDR AGPL port (rec.) / BSD libmodes lineage | Deferred (proceed with OrcSDR port, isolated for swap) |
| D2 | AP credentials | SSID `Raga-Radar`, WPA2 key `[redacted]` | **Cancelled** — board has no C6/WiFi; networking is Ethernet-only |
| D3 | Ethernet PHY on board | Not connected; WiFi AP only for now | **Resolved** — IP101GRI RMII wired + working (DHCP 192.168.200.102) |
| D4 | RF strategy while frames stay 0 | antenna placement / gain (44 dB manual now) vs. swap to proven `demod1090` path | Open — 44 dB wins current soak |
| D5 | Local CPR fallback (<=60s/60NM) | implement behind pair hook | Open — not needed while global decode works; hook ready |

---

## Phase 0 — Board bring-up

| # | Task | Status |
|---|---|---|
| 0.1 | Source IDF 6.2 env (`source ~/esp/esp-idf/export.sh`) and confirm `idf.py` works | In Progress |
| 0.2 | Install `pyserial` for esptool/monitor | In Progress |
| 0.3 | Confirm native USB-Serial/JTAG port (`/dev/cu.usbmodem5B900947191`) responds | Pending |
| 0.4 | Check chip: `idf.py set-target esp32p4` on hello-world; flash + monitor | Pending |
| 0.5 | Flash ESP-Hosted firmware to ESP32-C6 companion | Pending (moved to Phase 5) |
| 0.6 | Verify `esp_hosted_connect_to_slave()` + C6 firmware version in boot log | Pending (moved to Phase 5) |

## Phase 1 — Project skeleton

| # | Task | Status |
|---|---|---|
| 1.1 | Create IDF project structure at repo root (`CMakeLists.txt`, `main/`, `components/`) | Complete |
| 1.2 | Write `sdkconfig.defaults` (target esp32p4, PSRAM, USB DMA in PSRAM, dual-core) | Complete |
| 1.3 | Write `partitions.csv` | Complete |
| 1.4 | Vendor `esp_rtl_sdr` v0.8.0-rc3 into `components/` | Complete |
| 1.5 | Build empty app + boot log on target | Complete |

## Phase 2 — RTL-SDR driver bring-up

| # | Task | Status |
|---|---|---|
| 2.1 | `esp_rtl_sdr_install()` + wait `EVT_ENUMERATED`,`EVT_READY` | Complete |
| 2.2 | Log device info: HS speed, profile (V3/V4), capabilities | Complete |
| 2.3 | `esp_rtl_sdr_apply_need(ESP_RTL_SDR_NEED_ADSB)` + `start()` (1090 MHz / 2.048 MSPS) | Complete |
| 2.4 | Wire EVT_IQ_BLOCK -> 8x32KiB free/filled ring (OrcSDR pattern) | Complete |
| 2.5 | Instrument metrics: effective SPS, overruns, consumer drops | Complete |
| 2.6 | Gate: sustain 2.048 MSPS with zero drops | Complete (eff≈2.048M, 99.9%+, over=0 drops=0) |

## Phase 3 — ADS-B decoder

| # | Task | Status |
|---|---|---|
| 3.1 | Port `adsb_decoder.cpp` (magnitude, preamble gate, interpolation, CRC 0xfff409, DF17/DF11) | Complete |
| 3.2 | Pass boot-time `self_check()` | Complete |
| 3.3 | Hook `process_cu8()` per ring block from `adsb_dsp_task` (Core 1) | Complete |
| 3.4 | Log frames/sec + df17/crc stats | Complete (+ `AIRCRAFT` line on CRC-ok frame) |

## Phase 4 — Aircraft track store

| # | Task | Status |
|---|---|---|
| 4.1 | Implement track store (64 tracks, keyed by ICAO) | Complete |
| 4.2 | Fields: hex, flight, alt_baro, gs, heading, baro_rate, lat/lon, signal, last_seen | Complete |
| 4.3 | CPR pairing (even+odd <=10s) + global decode + optional local fallback | Complete (global decode via decoder's proven math, injection point for local fallback) |
| 4.4 | TTL expiry (60 s default) + revision counter | Complete |
| 4.5 | Squawk placeholder (`null`) for Phase 1 | Complete (ADS_STATE_SQUAWK_UNKNOWN) |
| 4.6 | Hysteresis/validation gates for callsign & fields (LakeShark pattern) | Complete (gates: callsign control-char/blank rejected, alt/gs/hdg/vr bounds) |

Phase 4 verified by host unit test `tests/host/test_adsb_state.c` (pace, pairing window, TTL, gates, cap, revision) + clean target boot. Awaiting live frames (RF-limited) to observe tracking end-to-end.

## Phase 5 — Networking (Ethernet)

> **Board has no ESP32-C6 / no WiFi** (see N6). Phase 5 is Ethernet-only.

| # | Task | Status |
|---|---|---|
| 5.1 | ~~ESP-Hosted C6 bring-up~~ | **Cancelled** — no C6 on board |
| 5.2 | ~~SoftAP `P4-Radar` @ 192.168.4.1~~ | **Cancelled** — WiFi impossible |
| 5.3 | Bring up Ethernet: EMAC + IP101GRI PHY (auto PHY addr) + reset GPIO51 | Complete |
| 5.4 | DHCP on ETH netif + default route | Complete (IP 192.168.200.102, gw .1, 100 Mbps link) |
| 5.5 | httpd binds INADDR_ANY (reachable on LAN) | Complete |

## Phase 6 — Web server + radar page

| # | Task | Status |
|---|---|---|
| 6.1 | Embed adapted `radar.html` in firmware | Complete |
| 6.2 | `GET /` and `/radar` route | Complete |
| 6.3 | `GET /data/aircraft.json` (dump1090 shape) | Complete |
| 6.4 | `GET /api/status` (NVS coords, msg rate, aircraft count, USB metrics, per-interface IPs) | Complete |
| 6.5 | Receiver coords from NVS (`radar pos <lat> <lon>` console cmd; default Pune) | Complete |
| 6.6 | Radar page accepts receiver coords (replace hardcoded Pune const) | Complete |
| 6.7 | Verify page on phone/PC over LAN (Ethernet) | In Progress — verified via curl + headless Chrome on 192.168.200.102; final user check on devices pending |
| 6.8 | Responsive mobile layout: `#ui` + Settings panels stacked (no overlap) | Complete — `#hud-stack` flex column on ≤768px / ≤520px height; desktop unchanged; browser-measured. Device visual check pending |
| 6.9 | Receiver lat/lon from web UI (browser geolocation or manual entry, NVS-persisted) | Complete — `/api/location` GET/POST; Settings panel lat/lon + Use Browser Location + Apply; immediate re-center. Device visual check pending |
| 6.12 | Honest HUD status (`ACFT` / `ON SCREEN` / `NO POS`) + dynamic title `Raga Radar (<nearest major city>)` | Complete — status now splits raw tracks vs plotted blips vs no-position; title uses offline ~75-city Haversine table (fallback lat/lon >150 km). Verified via mock-server browser test + `idf.py build`. LEARNING L19, L20 |
| 6.13 | Fix settings (`/api/settings` POST) parser + 2.4 MSPS decode + redundant Adaptive Gain control | Complete — value-bounded JSON extraction (L21); 2.4 MSPS `bit_spacing`/`half_bit_spacing` 1200/600 → 2400/1200 (L22); removed duplicate Adaptive Gain checkbox (Gain Mode is authoritative). Verified HTML parse + `node --check` + `idf.py build`. LEARNING L21, L22 |

## Phase 7 — Core optimization & validation

| # | Task | Status |
|---|---|---|
| 7.1 | Pin all tasks per core model (Core 0 network, Core 1 DSP) | Not Started |
| 7.2 | Tune priorities; verify no consumer drops / USB overruns at 2.048 MSPS | Not Started |
| 7.3 | Expose per-core CPU % / heap / USB metrics on `/api/status` | Not Started |
| 7.4 | Validate live ADS-B: frames/sec, track lifecycle, CPR positions stable | Not Started |
| 7.5 | Validate radar amplitude/trails/sweep/tooltips parity with reference `radar.html` | Not Started |
| 7.6 | Validate Ethernet route lookup (FROM/TO) when cabled | Not Started |
| 7.7 | Fix decoder dropping ~half of each 64 KiB IQ block (`kMagnitudeCapacity` 16640 < 32768 pairs) | Complete — chunked `process_cu8`/`process_pairs`; crc ≈2× (L23) |
| 7.8 | Fix status/adaptive-gain starvation + WDT flood (all tasks on core 1) | Complete — `rtl_stat` → core 0; disable idle-CPU1 WDT (L24) |
| 7.9 | Characterize tuner gain vs decode rate | Complete — **objective-dependent**: ~35 dB maximises crc/s, **~42.1 dB maximises range (57→94 NM)**; factory default 496 is the worst tested config. Operational gain = 42.1 dB (L26, relabelled) |
| 7.10 | 2.4 MSPS decode path | **Blocked** — preamble detection fails at 2.4 (~5/s vs ~45/s); L22 fix was insufficient (L25). Keep 2048 |
| 7.11 | Sanity-check `/api/settings` "defaults" (gain 496 / filters off) | Open — footgun for "Reset to Defaults" (L26) |
| 7.12 | Fix `/api/status` always reporting `phase:"idle"` / `programmed_sps:"0"` | Complete — derive `phase` from `device_present`/`started_req`; set `programmed_sps` from metrics (L27) |
| 7.13 | In-page health alerts (RF clipping/weak, USB overrun, decoder drops, ring running hot, gain near max) with click-to-see-fix + Apply | Complete — banner polls `/api/status` every 3 s; exposes `rf_state`/`gain_tenth_db`/`gain_mode`/`sample_min`/`sample_max`; each alert offers an exact settings change with a one-click Apply. Verified via browser DOM tests (10/10 logic + render) on the live device |

| 7.14 | Fix "phone sees flights, ESP32 does not" | Complete — **2.048 MSPS decodes 0 messages on ANY decoder**; ported readsb `demodulate2400` matched-filter + moved to 2.4 MSPS + NVS migration (L28). 0 CRC/s & 0 tracks → ~5 CRC/s & 9 tracks (4 positioned) |
| 7.15 | `/api/debug` diagnostics for USB-less verification | Complete — cumulative totals, 60 s window, live rates, signal levels, USB/RF health + advice, ring_free, config. Backs the in-page `#decode-stats` readout |
| 7.16 | Fix false "RF clipping → reduce gain" alert | Complete — RF min/max latched forever + 1-in-64 I-only scan; now a 32-block rolling window, full I+Q scan, reset on `start()`, 8-block streak required (L29) |
| 7.17 | Live decode-rate readout in radar UI | Complete — `#decode-stats` polls `/api/debug` every 3 s, per-second rates from cumulative deltas; green/amber/red by CRC vs preamble flow |
| 7.18 | Close DSP throughput gap (~24% of stream) | Complete — replaced per-sample `std::sqrt` (microcoded on RV32F) with integer AMB. CRC 11.0→27.7/s, samples/s 576k→1.14M, overruns 0, tracks held. LEARNING L31 |

---

## Change log

| Date | Change |
|---|---|---|
| 2026-09-19 | Created tracker from approved implementation plan |
| 2026-09-19 | Phases 5–6: board has **no C6/WiFi** (schematic-verified) → Ethernet-only. EMAC+IP101GRI netif up, DHCP 192.168.200.102; httpd serving radar page fully (curl + headless Chrome verified). Wifi/ESP-Hosted wiring removed (`wifi_ap.*`, esp_wifi/esp_hosted deps). |
| 2026-10-12 | Phase 6.8: fixed mobile HUD overlap — wrapped `#ui` + `#settings-panel` in `#hud-stack` (`display:contents` desktop; flex column ≤768px/≤520px height). Moved Settings positioning to CSS. LEARNING L15. Verified via browser-measured rects + `idf.py build`. |
| 2026-10-12 | Phase 6.9: receiver location from web UI — new `GET`/`POST /api/location` (validated, NVS-persisted via `http_radar_set_receiver`), header getter/setter, NVS 0,0 accepted. Settings panel "Receiver Location" (lat/lon inputs + Use Browser Location + Apply, instant re-center). LEARNING L16 (geolocation needs secure context). Verified with mock-server browser test + `idf.py build`. |
| 2026-10-12 | Phase 6.10: fix "Raga Radar - Pune" panel not fully collapsing — `#status`/`#user-location` were outside `#ui-content`. New `applyMenuState()` toggles all three; verified initial/expanded/collapsed at desktop + mobile. LEARNING L17. |
| 2026-10-12 | Phase 6.11: fix route lookup — `api.adsb.lol/api/0/routeset` sends no CORS headers (browser `Failed to fetch`). Switched to `api.adsbdb.com/v0/callsign/<cs>` (GET, `ACAO: *`), mapped `flightroute` into the existing tooltip shape; per-callsign GET with cache/dedupe + 8 s timeout. Verified in-browser (IGO123 → VIDP-VILH / DEL-IXL) + not-found path + `idf.py build`. LEARNING L18. |
| 2026-10-13 | Phase 6.12: HUD honesty + dynamic title. Status line now `ACFT (raw tracks) / ON SCREEN (plotted blips) / NO POS (no lat-lon)`; replaced the misleading raw-count `ACFT`. Title changed from hardcoded "Raga Radar - Pune" to `Raga Radar (<nearest major city>)` via an offline ~75-city Haversine table (fallback `lat, lon` if >150 km). Verified via mock-server browser test (Hyderabad/Pune/remote) + HTML parse + `node --check` + `idf.py build` (56% free). LEARNING L19, L20. Build-only — device not flashed (offline). |
| 2026-10-13 | Phase 6.13: settings/decode fixes. `/api/settings` POST used `strstr(p,"true"/"false")` which scanned the whole remaining body → `false` fields followed by a `true` field were misread (L21); replaced with value-bounded `json_get_value()`. 2.4 MSPS `bit_spacing`/`half_bit_spacing` were half the correct milli-sample values (1200/600) → 0 frames; fixed to 2400/1200 (L22). Removed the vestigial duplicate "Adaptive Gain" checkbox (Gain Mode drives `adaptive_gain_step`, `rtl_pipeline.c:280`). Verified HTML parse (33 unique ids, 0 dup) + `node --check` + concat + `idf.py build` (56% free). Build-only — device not flashed (offline). |
| 2026-10-13 | Phase 7.7-7.10: live decode diagnosis on hardware (USB serial + web UI at 192.168.1.39). **7.7:** decoder dropped ~half of every 64 KiB IQ block (`kMagnitudeCapacity`=16640 vs 32768 pairs) → chunked `process_cu8`/`process_pairs`; crc ≈2× (L23). **7.8:** `rtl_stat` was starved on core 1 by the prio-6 `adsb_dsp` (gain/status never ran; IDLE1 WDT flood) → moved to core 0, disabled idle-CPU1 WDT (L24). **7.9:** manual-gain sweep (crc/s over 15 s windows) → **35.0 dB optimal**, 40/49.6 dB worse; factory default 496 is the worst tested config (L26). **7.10:** 2.4 MSPS still ~0 — preamble detection fails (~5/s vs ~45/s); L22 was insufficient (L25). Final live config: 2048000 sps, 35.0 dB manual, dc_filter/aggressive/crc_fix ON (NVS-persisted); 11-14 tracks concurrently (user saw ~19), ~47 NM max, `rf=OK`, `over=0 drops=0`. |
| 2026-10-13 | Phase 7.12-7.13: diagnostic warnings + status fixes (on hardware, device at **192.168.1.6**, antenna in window). **7.12:** `/api/status` always reported `phase:"idle"` and `programmed_sps:"0"` — `rtl_pipeline_get_status()` never set them; now derives `phase` from `device_present`/`started_req` and fills `programmed_sps` (L27). **7.13:** added `rf_state`/`gain_tenth_db`/`gain_mode`/`sample_min`/`sample_max` to `/api/status`; new in-page alert banner (top-center desktop, stacked mobile) polls it every 3 s and warns on RF clipping/weak, USB overrun, decoder drops, IQ ring running hot, and gain-near-max — each expandable (click) to show the exact settings change with a one-click **Apply** (POSTs `/api/settings`). Healthy devices show no banner. Verified: HTML parse (43 unique ids, 0 dup), `node --check` both scripts, `/api/status` JSON validated (compiled format string + `json.loads`), 10/10 alert-logic unit tests, and live browser render tests (all 6 alert types + dismiss + multi-alert suffix) on the running device. Flashed + boot-verified (`gain=42.1` survives reboot, crc climbing, `rf=1`). |
| 2026-10-13 | Phase 7.9 **relabelled**: gain is objective-dependent, not a single optimum. Direct A/B on the live device at 2048 kSPS (all else constant): **35.0 dB → ~5.5 crc/s, smp=[101,152], max 57 NM** vs **42.1 dB → ~2.7 crc/s, smp=[80,181], max 94 NM**. Higher gain trades frame rate for ~2× range and more positioned tracks. Operational gain set to **42.1 dB**. L26 relabelled (original crc/s sweep retained). |
| 2026-10-14 | Phase 7.14-7.17: **reception fixed.** Same dongle+antenna through real `readsb` on the Mac proved **2.048 MSPS yields 0 CRC at every gain/ppm** while 2.4 MSPS yields 780 — the old rate simply cannot decode Mode-S on this R820T2, contradicting L22/L25. Ported readsb `demodulate2400` (linear amplitude, 1 Hz DC high-pass, noise-adaptive preamble gate, 5-phase `slice_phase0-4` correlation, skip `msgbits*8/4`) replacing the fixed-phase 2-point slicer; switched the default to 2.4 MSPS everywhere with an NVS migration (`2048000 → 2400000`) so existing units self-heal, and `/api/settings` now rejects 2048000. Verified offline on the real capture first (650 CRC / 289 distinct vs readsb 883 / 419), then on hardware: **9 tracked aircraft, 4 positioned**. Also fixed the RF-health false alarm that latched `sample=[0,255]` at boot and advised "reduce gain" while decoding normally — it drove gain to 0 dB and killed reception (L29, L30). Added `/api/debug` + `#decode-stats` live readout so reception is verifiable with no USB. |
| 2026-10-14 | Phase 7.18: **AMB magnitude** (L31). `adsb_dsp` was discarding ~76% of the 2.4 MSPS stream; per-sample `std::sqrt` is a microcoded software call on the RV32P4. Replaced with integer Alpha Max Plus Beta Min scaled x415 to preserve the magnitude units the readsb `slice_phase0..4` slicers and preamble threshold expect — no slicer re-tuning needed. Hardware: CRC **11.00 → 27.72/s (+152%)**, samples/s **576,717 → 1,139,780** (~24% → ~47% of stream), overruns 0, tracks 19 held, self_check=1. Validated offline on a real capture amplitude-scaled across swing 108/54/27/10 so the weak-signal case was covered — AMB beats float at every level (304/264/199/92 vs 289/241/144/36), which is why the earlier quantised int-sqrt table failed on-device. |
| 2026-10-14 | Phase 7.19: **BharatRadar feeder** (L32). `/api/frames` + a 64-entry capture ring first exposed real device frames (`adsb_frame_info_t` gained `bytes[14]`/`bit_length`/`rx_timestamp_us`; the bridge previously decoded fields and *discarded the raw frame*). Validated end-to-end offline before writing any C: 64 real frames replayed through the proven Python feeder (`br_feeder.py`) put 4 aircraft on `map.bharatradar.com`. Then ported the encoder to `main/beast_feeder.c` (BEAST `0x1A 0xE4`+UUID handshake, `0x32`/`0x33`, 0x1A-doubling, 6B 12MHz stamp, RSSI `0xFF`, no location, no pong) — **differential-tested byte-identical to the Python reference on 4 vectors** incl. 0x1A escaping. Decode hook only enqueues into a 256-slot lock-free SPSC ring (drop-on-full) so the prio-6 `adsb_dsp` never blocks; socket I/O runs on a new core-0 prio-3 task with reconnect backoff, honouring L24. NVS string settings (`feed_en`/`feed_uuid`/`feed_name`/`feed_host`/`feed_port`) + `/api/feeder` GET/POST + Web UI section. Feeder ships **disabled by default**. Hardware: `connected:true sent:6943 dropped:0 reconnects:0`, live map showed 6 new aircraft (AXB2717/IGO947/AKJ384V/AXB2743) absent before the native feeder. Hit the L32 handler cap (13 routes vs `max_uri_handlers=12`) on first flash — raised to 16. |
| 2026-10-14 | Phase 7.20: **hotfix — feeder boot loop (L33).** `v0.5.0` started `beast_feeder_start()` before `eth_netif_start()`; lwIP `getaddrinfo()` asserts `tcpip_send_msg_wait_sem ... Invalid mbox` when the tcpip thread is absent -> panic -> reboot -> repeat, unbreakably, because the feeder UUID is persisted in NVS so every boot re-triggers it. Unit was unusable (no web UI, no Ethernet) and needed a reflash. Fixed by gating all name resolution on `IP_EVENT_ETH_GOT_IP` and starting the feeder after `eth_netif_start()`. **Root cause of the escape:** the feeder was only ever enabled at runtime over `/api/feeder`, never tested with it persisted in NVS at boot (L33). Verified: 0 reboots in 20 s, feeder `connected:true sent:953 dropped:0`, decode 23-44 crc/s. |
| 2026-10-14 | Documentation: added `README.md` (product overview, signal chain, architecture, task/core map, build + flash, first-run setup, full HTTP API table, feeder protocol, NVS key reference, repo layout, testing, measured performance, known issues) and a **licensing/attribution audit** (L34). Audit corrected a wrong assumption: `components/esp_rtl_sdr/` is **AGPL-3.0** and its header states it is a clean-room implementation, *not* a librtlsdr port. `components/adsb_decoder/` is original code whose **algorithm is ported from readsb (GPL-3.0)**. Both flagged in the README as items to resolve before distributing a binary/closed product — notably AGPL §13, which is triggered by this project's network web UI. |
| 2026-10-14 | Warm-up sample-range collapse: **investigated, does not reproduce.** Tracked 27.7 min past the reported 1-2 min onset on the stable `v0.5.1` build — swing steady 40-72 (collapse state was 6), CRC climbing, 0 overruns/drops, `crc_yield_pct` 100%. Best explanation: all original observations were taken while the device was in the L33 reboot loop, where the R820T2 is re-initialised continuously. **No code change made** — deliberately, since a speculative fix to a non-reproducing symptom risks the working decode path. LEARNING L35. |
