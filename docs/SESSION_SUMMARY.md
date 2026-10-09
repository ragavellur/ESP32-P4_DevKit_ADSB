# Session Summary

## Result today: full firmware chain booted and validated on target

- Clean `idf.py build` → flash → boot on the ESP32-P4 DevKit.
- RTL-SDR dongle (Nooelec NESDR Nano 2+, R820T2) enumerated: 1090 MHz @ 2.048 MSPS.
- Streaming is clean: `sps=2048000 eff≈2.048M over=0 drops=0` (99.9%+ USB efficiency).
- Decoder fed real CU8 IQ, `self_check()` passes, `AIRCRAFT icao=...` log line prints on any CRC-valid frame.
- Current limit is **RF signal strength**, not firmware: `mag=[0..43]`, 0 valid frames in ~5 min of soak with antenna near window. 44 dB manual gain was the only setting that produced any decoder frame during the session.

## Phase 4: aircraft track store implemented (second session block)

- New `main/adsb_state.{h,c}`: pure-C track store — 64 ICAO-keyed tracks, even+odd CPR pairing in a 10 s window, TTL 60 s expiry, per-track revision counter, squawk placeholder, validation gates (callsign control-char/blank rejection; alt/gs/hdg/vertical-rate bounds).
- Bridge gained a C frame hook (`adsb_decoder_bridge_set_frame_hook`) and a C-linkable global-CPR pair (`adsb_decoder_bridge_cpr_pair`) that reuses the decoder's proven `decode_global_cpr()`.
- `rtl_pipeline.c` wires: decoder → hook → `adsb_state_update` (guard mutex, `esp_timer_get_time()` ticks); status task expires + counts tracks each second.
- Verified two ways: (1) host unit test `tests/host/test_adsb_state.c` — pairing window, stale-pair rejection, TTL, gates, 64-cap eviction, revision increment (found+fixed a real NULL-deref on the eviction path); (2) clean target boot — streaming intact, zero drops, no regressions.
- Note: with `mag=[0..35]` and no aircraft bursts, the store currently has nothing to track — it is ready and verified, waiting on RF.

## Session 2 — Networking over Ethernet + radar page live on LAN (not yet done → done)

### Hardware finding (saved hours): the board has NO ESP32-C6, so WiFi is impossible
- The DevKit is the **Waveshare ESP32-P4-ETH (PoE, no-header)** — NOT the Espressif ESP32-P4-Function-EV-Board.
- Verified against the official schematic PDF (`files.waveshare.com/wiki/ESP32-P4-ETH/ESP32-P4-ETH-datasheet.pdf`): **zero** references to ESP32-C6, SDIO_CLK/CMD/DATA (R8, R16), C6_U0RXD/TXD, CHIP_PU, or antenna/RF. GPIO18/19/20/21/22/23 route to 40-pin headers J1/J2; GPIO54 to J3.
- WiFi is physically impossible on this board. All ESP-Hosted / C6-firmware work was chasing a chip that isn't there (root cause of the CMD5 timeouts `0x107`). AP and ESP-Hosted plans are **cancelled**.
- **Replaced by Ethernet**: `idf.py --target esp32p4 esp-eth phy ip101` (IP101GRI), EMAC + RMII on the board's integrated PHY. Cable was already connected.

### Ethernet integration (Phase 5), then radar page (Phase 6)
- `main/eth_netif.{h,c}`: EMAC + IP101GRI (RMII), auto PHY-address scan, reset GPIO51, DHCP with a retry/don't-give-up loop, default route. Note: this IDF's `esp_eth_driver_install()` takes **2 args** (config + out handle); ref-clock low + 2 ms delay before PHY-ID read avoids an init race (N7).
- `app_main.c` calls `eth_netif_start()` (guarded `CONFIG_ADSB_RADAR_ETH`); Kconfig replaced with `ADSB_RADAR_ETH`.
- `main/CMakeLists.txt`: removed `wifi_ap.c`, `esp_wifi`, `espressif__esp_hosted`; added `eth_netif.c`, `esp_eth`; `main/idf_component.yml` now only `espressif/usb 1.5.0`; `wifi_ap.{c,h}` deleted.
- `sdkconfig.defaults` rewritten (Ethernet/NVM defines; ESP-Hosted/WiFi config gone). Clean reconfigure + build (app ≈654 KB, 58% free of 1.5 MB), flashed, boot-verified over USB serial.

### Verified on target over the wire
- Boot log: **Ethernet attached, link up 100 Mbps, MAC e8:f6:0a:e7:20:d2, DHCP IP 192.168.200.102** (gw 192.168.200.1). RTL-SDR still streaming at 2.048 MSPS with decode_ok; httpd up.
- `curl` over LAN: `GET /` → 200 (radar page 21664 B "Raga Radar - ADS-B"), `GET /api/status` → valid JSON (phase idle, device_present true, decode_ok true, sps 2046702, overruns/drops 0, Pune coords), `GET /data/aircraft.json` → dump1090-shape JSON (aircraft array empty — no aircraft in range, RF-limited as before).
- Headless Chrome at `http://192.168.200.102/`: page renders — Raga Radar - Pune, Range/Sweep/AltFilter/Route/Code/Show-Trails controls, `SYNCED | ACFT: 0`, `RADAR REF: 18.4807, 73.8982`.

### Next steps
1. User-facing check: browse `http://192.168.200.102/` on a phone/PC on the same LAN; confirm radar sweep backdrop + controls render.
2. RF strategy remains the gating item (D4) — with a functional web UI the gain/antenna soak now can be watched from the page.
3. Phase 7: pin tasks per core, expose CPU/heap/USB metrics on `/api/status`.

## Changes made this session

- `main/rtl_pipeline.c`: aligned to real on-disk driver API (dropped invented `ESP_RTL_SDR_STATE_READY`, status field renames), wait on `EVT_READY`/`device_present` before `start()` + retries, fixed dup `IQ_RING_SLOTS`, added manual 44 dB tuner gain after start.
- `components/adsb_decoder/src/adsb_decoder_bridge.cpp`: `on_frame` now logs `AIRCRAFT` line (icao/type_code/callsign/alt/speed/heading) on CRC-valid frames.
- `sdkconfig`: `ESP32P4_SELECTS_REV_LESS_V3=y`, `REV_MIN_FULL=100` (chip is rev v1.3), `ESP_DEFAULT_CPU_FREQ_MHZ=360` (400 MHz unsupported on rev<3.0), `ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=n`.
- `docs/TASKS_TRACKER.md`: Phases 0–3 marked complete; added platform-notes (N1–N5) and open decision D4.
- **Phase 4 (new):** `main/adsb_state.{h,c}` track store; bridge frame hook + CPR pair; `rtl_pipeline.c` wiring; `tests/host/test_adsb_state.c` host test; tracker Phase 4 complete.
- `docs/LEARNING.md` created: L1–L10 + recurring meta-pattern (verify before editing).
- **Session 2:** `main/eth_netif.{h,c}` (EMAC+IP101GRI RMII, auto PHY addr, GPIO51 reset, DHCP); `app_main.c` + Kconfig `ADSB_RADAR_ETH`; CMake/idf_component.yml/sdkconfig.defaults de-WiFi'd and de-ESP-Hosted'd; `wifi_ap.{c,h}` deleted; clean rebuild + flash; docs (tracker, summary) updated. Verified Ethernet link/DHCP, HTTP routes, radar page render via headless Chrome.

## Reference repos inspected (for provenance only, not merged)

- `SAMS0N1TE/esp32p4-rtl-sdr-v4` (EOL, moved to `SAMS0N1TE/LakeShark`): working ADS-B for ESP32-P4 + RTL-SDR V4/R828D, raw `rtlsdr_*` API + `demod1090.c`/`mode-s.c`; uses tuner AUTO gain; USB host CPU0 / decode CPU1. Cloned to `/tmp/ref_esp32p4-rtl-sdr-v4`.
- `kvhnuke/esp32-rtl-sdr`: classic ESP32 lineage of the above. Cloned to `/tmp/ref_esp32-rtl-sdr`.

## Next steps (not yet done)

1. User-facing check as above: confirm the radar page on a phone/PC on the LAN (Phase 6.7).
2. Decide RF strategy (D4): reposition antenna / longer soak vs. swap decoder to proven `demod1090` path — now watchable live from the radar page.
3. Once frames appear: validate track store end-to-end (pairing → `lat/lon`) in the UI, then Phase 7 (per-core pinning, metrics on `/api/status`).
4. Flashing note: this session flashed via native `/dev/cu.usbmodem5B900947191`; capture via direct `pyserial` read works — avoid esptool `run`/`read_mac` while capturing (drops chip into download mode); re-flash resets into run mode.

## 2026-10-09 — SDLC: public GitHub repo + AGENTS.md

- Created public repo **github.com/ragavellur/ESP32-P4_DevKit_ADSB**; pushed `main` (13 commits) plus the 11 existing semver tags.
- Added root **`AGENTS.md`** codifying the process: branch model (`main` + `feat/*`/`fix/*`/`docs/*`/`chore/*`), Conventional Commits, pre-flash verification gate (real HTML parse + `node --check` on JS + duplicate-identifier grep), mandatory docs updates (`LEARNING.md`/`TASKS_TRACKER.md`/`SESSION_SUMMARY.md`), build/flash commands, and the never-commit list.
- Decode parity reference captured (read-only, from Pi `192.168.200.20`): Raspberry Pi runs `readsb` at 2.4 MSPS with `--gain auto --dcfilter --fix --preamble-threshold 24`; our P4 runs 2.048 MSPS manual gain. Parity work (2.4 MSPS correlation demod, auto gain, preamble threshold) is queued for Phase 3.
- Note: Mac moved from `192.168.200.x` to `192.168.1.x`; Pi inspection is deferred until the Mac rejoins the `192.168.200.x` LAN.