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

## Phase 7 — Core optimization & validation

| # | Task | Status |
|---|---|---|
| 7.1 | Pin all tasks per core model (Core 0 network, Core 1 DSP) | Not Started |
| 7.2 | Tune priorities; verify no consumer drops / USB overruns at 2.048 MSPS | Not Started |
| 7.3 | Expose per-core CPU % / heap / USB metrics on `/api/status` | Not Started |
| 7.4 | Validate live ADS-B: frames/sec, track lifecycle, CPR positions stable | Not Started |
| 7.5 | Validate radar amplitude/trails/sweep/tooltips parity with reference `radar.html` | Not Started |
| 7.6 | Validate Ethernet route lookup (FROM/TO) when cabled | Not Started |

---

## Change log

| Date | Change |
|---|---|---|
| 2026-09-19 | Created tracker from approved implementation plan |
| 2026-09-19 | Phases 5–6: board has **no C6/WiFi** (schematic-verified) → Ethernet-only. EMAC+IP101GRI netif up, DHCP 192.168.200.102; httpd serving radar page fully (curl + headless Chrome verified). Wifi/ESP-Hosted wiring removed (`wifi_ap.*`, esp_wifi/esp_hosted deps). |