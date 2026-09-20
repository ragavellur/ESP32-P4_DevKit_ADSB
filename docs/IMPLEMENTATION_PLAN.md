# ADS-B Radar on ESP32-P4 (no display) + RTL-SDR — Implementation Plan

Date: 2026-09-19
Status: APPROVED (build not started)

---

## 1. Goal (Phase 1)

1. Receive ADS-B at 1090 MHz from an RTL-SDR attached to the ESP32-P4 High-Speed USB host port.
2. Decode Mode-S / ADS-B frames into aircraft tracks (callsign, altitude, ground speed, vertical rate, position via CPR).
3. Serve a conventional PPI radar display over an internal web server (WiFi AP + optional Ethernet),
   styled exactly like `/Users/raghavan/Documents/radar.html`.
4. Optimize for correct ESL (ESP32-P4: 2 x RISC-V cores @ 400 MHz) utilization: USB + network on Core 0, DSP + decode on Core 1.

---

## 2. Reference repositories (analyzed)

| Repo | Use | Key takeaways |
|---|---|---|
| `hardcoreerik/OrcSDR` | AGPL-3.0 | C++ ADS-B decoder `adsb_decoder.cpp` (preambles, interpolation, CRC, CPR), task model for exp U4SQL driver, web console on port 80, ESP-Hosted C6 bring-up |
| `hardcoreerik/esp-rtl-sdr` | AGPL-3.0, v0.8.0-rc3 | Clean-room RTL2832U USB host driver for P4; `NEED_ADSB` intent = 1090 MHz @ 2.048 MSPS; proven 3x32KiB bulk URB profile for Tab5 |
| `SAMS0N1TE/LakeShark` | GPL-3.0 (libmodes BSD-2 lineage) | Headless W4 boards (`nano_headless_ble`, `wifi6_headless_ble`), RTL task model, ESP-Hosted AP/STA + HTTP file server, aircraft JSONL feed |

---

## 3. Environment check (verified on this machine)

| Item | Value |
|---|---|
| ESP-IDF | 6.2 at `~/esp/esp-idf`, target `esp32p4` present |
| Serial (primary) | `/dev/cu.usbmodem5B900947191` — native USB-Serial/JTAG (flash + console) |
| Serial (backup) | `/dev/cu.wchusbserial5B900947191` — UART bridge (CH9102) |
| RTL-SDR | Connected to HS USB host port (electrical check at bring-up) |
| Radar reference | `radar.html` fetches dump1090-style `/data/aircraft.json` |
| Networking | ESP32-C6 companion (Wi-Fi 6/BLE) over ESP-Hosted SDIO; P4 has internal Ethernet MAC (PHY on devkit, verify) |

### Decisions made with user
- **Wireless:** AP only ("P4-Radar"); STA disabled on WiFi.
- **Ethernet:** if the PHY is present on the board, Ethernet is the only upstream (internet) interface.
- **Receiver position:** fixed coordinates stored in NVS (defaults: Pune 18.480718, 73.898235).
- **Console:** native USB-Serial/JTAG.

### Open decisions (blockers for completion before finalize)
- **Decoder license path:** OrcSDR AGPL port (recommended) vs BSD libmodes lineage from LakeShark.
- **AP credentials:** SSID `P4-Radar`, open or WPA2 key.
- **Ethernet presence on the actual board:** confirm RJ45/PHY.

---

## 4. Signal chain

```
Antenna → RTL-SDR (1090 MHz @ 2.048 MSPS)
   → P4 High-Speed USB Host (DMA in PSRAM)
   → esp-rtl-sdr driver                 [Core 0]
   → IQ block ring (8 × 32 KiB PSRAM)   [ring slots]
   → ADS-B DSP task (magnitude+decode)  [Core 1]
   → Aircraft track store (CPR pairing, TTL)  [Core 1]
   → JSON snapshot (~1 Hz, mutex)       [Core 1 producer]
   → httpd :80  /  /radar  /data/aircraft.json  /api/status   [Core 0]
   → C6 WiFi AP 192.168.4.1  +  Ethernet netif (internet)        [Core 0]
```

### Sample-rate rationale
- 2.048 MSPS is the ADS-B baseline (RTL2832 quantized) and the proven P4 continuous path (`ESP_RTL_SDR_RATE_2048K`).
- Frame window = 120 us; bit centers every 2048 samples; preample peaks at samples 0/2/7/9.

---

## 5. Core utilization model

| Core 0 (host + network) | Core 1 (DSP + application) |
|---|---|
| USB Host lib task (esp-rtl-sdr, prio 20, `usb_task_core_id = 0`) | `adsb_dsp_task` (prio 6) — magnitude + preamble + CRC decode |
| Driver IQ callback (prio 19) — memcpy free slot -> filled queue | inline track update + CPR global decode |
| ESP-Hosted C6 tasks (SDIO, lwIP) | 1 Hz `status/expire` worker (prio 3): msg-rate, drops, TTL, JSON publish |
| Ethernet netif (+DHCP default route) | |
| `httpd` task (prio 5) — static + JSON | |

Notes:
- The decoder is the only heavy consumer and stays exclusively on Core 1.
- IQ callback performs only a fast memcpy (never decode/paint/network).
- httpd binds `0.0.0.0` so it serves on both AP and Ethernet addresses.

---

## 6. Implementation phases

### Phase 0 — Board bring-up
- Init IDF env, install pyserial.
- Flash C6 with ESP-Hosted firmware (required; factory C6 lacks it).
- Verify `esp_hosted_connect_to_slave()` success.

### Phase 1 — Project skeleton
- New IDF project in this repo root.
- `sdkconfig.defaults` (target esp32p4, PSRAM, USB DMA in PSRAM, dual-core FreeRTOS).
- Vendor `esp_rtl_sdr` v0.8.0-rc3 under `components/`.

### Phase 2 — RTL-SDR driver bring-up
- Install -> enumerate -> log HS speed, profile (V3/V4), capabilities.
- `apply_need(NEED_ADSB)` + `start()`, IQ ring (8x32KiB).
- Instrument: effective SPS, overruns, consumer drops (`get_metrics`).

### Phase 3 — ADS-B decoder
- Port `adsb_decoder.cpp` (CU8 -> magnitude, preamble gate, interpolation, CRC 0xfff409; DF17 + DF11).
- Run boot-time `self_check()`.
- Hook `process_cu8` per ring block.

### Phase 4 — Aircraft track store
- Up to 64 tracks keyed by ICAO; fields hex, flight, alt_baro, gs, heading, baro_rate, lat/lon, signal, last_seen.
- CPR pairing: even+odd within 10 s -> global decode; optional local fallback (<=60s/60NM).
- TTL expiry 60 s; revision counter.
- Squawk: not decoded by reference impls -> `null` in Phase 1.

### Phase 5 — Networking (AP + Ethernet)
- C6 softAP SSID `P4-Radar`, IP 192.168.4.1. STA disabled.
- Ethernet: P4 EMAC + LAN8720 (config or skip if no PHY); DHCP client; Ethernet is default route.
- httpd binds INADDR_ANY.

### Phase 6 — Web server + radar page
- Routes: `/`, `/radar` (embedded radar.html); `/data/aircraft.json`; `/api/status`.
- Reference page adaptation: receiver coords from `/api/status` (fallback NVS), keep sweep/trails/filters.
- Receiver coords provisioned via console command (`radar pos <lat> <lon>`), default Pune.

### Phase 7 — Core optimization & validation
- Pin tasks per table; verify 2.048 MSPS sustained, zero consumer drops, no USB overruns.
- Expose per-core CPU % / heap / USB metrics on `/api/status`.
- Validate: C6 up, AP broadcasting, RTL enumerated HS, `self_check` passes, frames/sec climb, radar page shows blips/trails, Ethernet route lookup works when cabled.

---

## 7. Proposed file layout

```
ESP32-P4_DevKit_ADSB/
├── CMakeLists.txt / sdkconfig.defaults / partitions.csv
├── components/
│   ├── esp_rtl_sdr/          # pinned v0.8.0-rc3 (vendored)
│   └── adsb_decoder/         # ported decoder + self_check
├── main/
│   ├── CMakeLists.txt, idf_component.yml
│   ├── app_main.cpp          # boot, task creation, core pinning
│   ├── rtl_pipeline.c        # install/need/start, IQ ring
│   ├── adsb_state.c          # track store, CPR pairing, expiry
│   ├── wifi_ap.c             # ESP-Hosted C6 bring-up + softAP
│   ├── eth_netif.c           # Ethernet (LAN8720) + default route
│   ├── http_server.c         # httpd, handlers, JSON snapshot
│   ├── console.c             # provisioning (radar pos, ssid/pass, gain)
│   └── web/radar.html        # adapted example (embedded)
```

---

## 8. Risks & fallbacks

1. **IDF 6.2 vs reference 5.4/5.5** — minor API drift in USB host / esp-hosted / WiFi. Mitigation: vendored pinned components; fallback install IDF 5.5.4 alongside.
2. **RTL-SDR power** (~300 mA) on HS host port — use OTG adapter; use powered hub if unstable.
3. **Ethernet presence** — if no PHY, Phase 6 degrades to AP-only (radar page unaffected).
4. **C6 ESP-Hosted firmware** — most fiddly manual step; do first, verify with boot log.
5. **Licensing** — AGPL/GPL copyleft obligations of reused files; attribution headers retained; hobby use fine, distribution must keep licenses.

---

## 9. License note

- `esp-rtl-sdr`: AGPL-3.0.
- OrcSDR decoder: AGPL-3.0.
- LakeShark libmodes decoder lineage: BSD-2-Clause (per bundled `LICENSE.libmodes`).
- This project: licence to be chosen; keep all reused-file notices intact.