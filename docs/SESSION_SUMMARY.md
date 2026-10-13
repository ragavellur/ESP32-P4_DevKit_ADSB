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

## 2026-10-12 — Fix mobile HUD panel overlap (`fix/mobile-hud-overlap`)

- **Problem:** On phones the main control panel (`#ui`) and the Settings panel overlapped; Settings covered the Range/Sweep/Alt controls.
- **Root cause:** Both panels were absolutely positioned in opposite corners; the `max-width:768px` media query restyled only `#ui`, leaving `#settings-panel` at `min-width:250px` → intersection at ~390px width.
- **Change (only `main/radar_data/radar.html`):**
  - Wrapped `#ui` + `#settings-panel` in new `#hud-stack`; `display:contents` on desktop (pixel-identical corners), flex column stack on small screens.
  - Moved Settings panel positioning out of inline `style=` into a `#settings-panel` CSS rule so it is overridable.
  - Mobile rules: full-width panels, `box-sizing:border-box`, `gap:8px`, `max-height:100dvh; overflow-y:auto`, per-content `max-height` + scroll, `touch-action:pan-y`; media query now `(max-width:768px), (max-height:520px)` so landscape phones also stack.
- **Verified (no hardware):** HTML parsed (no nesting errors, no duplicate ids 28/28 unique); both `<script>` blocks pass `node --check`; measured bounding rects in a real browser — mobile 500×667 (expanded: no overlap, stack scrolls), landscape 844×390 (no overlap), desktop 1280×800 (stack `display:contents`, `#ui` top-left @15,15, Settings top-right @15 — unchanged). `idf.py build` clean (57% free).
- **Pending:** on-device flash/visual check once the board is back on the LAN.
- Next: `feat/web-receiver-location` (lat/lon from browser geolocation or manual entry, persisted in NVS, `/api/location`).

## 2026-10-12 — Receiver location from the web UI (`feat/web-receiver-location`)

- **Feature:** set the radar reference lat/lon from the browser (for travelling), persisted in NVS and applied live.
- **Firmware (`http_radar.c`/`.h`):** new `GET /api/location` → `{lat,lon}`; new `POST /api/location` (parses `lat`/`lon`, validates finite + ±90/±180, else `400`) → `http_radar_set_receiver()` (NVS + in-memory) → echoes values. Declared `http_radar_set_receiver`/`http_radar_get_receiver` in the header. `nvs_load_receiver()` now accepts a stored `0.0` (presence-based, not `!=0`). 9 URI handlers registered.
- **UI (`radar.html`):** Settings panel "Receiver Location" section — Latitude/Longitude inputs, **Use Browser Location** (fills fields only; expects manual Apply), **Apply Location** (POSTs and re-centers the sweep immediately by updating the global `receiverLat/Lon`), plus an inline status line for success/errors.
- **Caveat (LEARNING L16):** browser geolocation requires a secure context; on plain-HTTP LAN it fails, so manual entry is the reliable path. The UI shows a clear message when detection fails.
- **Verified (no hardware):** HTML parsed clean (33 unique ids), both scripts pass `node --check`, `idf.py build` clean (57% free). Functional browser test against a local mock server: loadLocation fills inputs; Apply persists + updates `receiverLat/Lon`; out-of-range rejected client-side (server untouched); geolocation success fills fields, failure shows manual-entry hint.
- **Pending:** on-device flash + real-browser check (geolocation will fail over HTTP; manual entry is expected to work).

## 2026-10-12 — "Raga Radar - Pune" panel did not fully collapse

- **Report:** After collapsing the top-left panel, the `SYNC ERR: Failed to fetch` and `User GPS: …` lines stayed visible, so it looked un-collapsed.
- **Root cause:** `toggleMenu()` only toggled `#ui-content`; `#status` and `#user-location` are siblings inside `#ui` but were **outside** `#ui-content`, so they were never hidden. (LEARNING L17.)
- **Fix (`radar.html`):** replaced the inline two-line toggle with `applyMenuState()`, which sets display on `#ui-content`, `#status`, and `#user-location` together and updates the `[+]/[-]` label; called on load and on every toggle.
- **Verified (no hardware):** both scripts pass `node --check`; browser test — desktop reload shows all expanded; mobile (500×667) reload starts fully collapsed (status/SYNC ERR hidden), expands to show all, collapses fully again.
- **Note:** `SYNC ERR: Failed to fetch` is the `/data/aircraft.json` fetch failing at the network level (device unreachable from the browser), not a UI regression; it now correctly hides when collapsed.

## 2026-10-12 — Route lookup fixed (CORS)

- **Report:** Route data gave a CORS-style `Failed to fetch` error; user asked to either disable the route API or make it work.
- **Root cause (reproduced locally):** `POST https://api.adsb.lol/api/0/routeset` with `Content-Type: application/json` triggers a preflight; the endpoint replies `201` but with **no `Access-Control-Allow-Origin`**, so the browser blocks it (LEARNING L18). Confirmed via `curl -X OPTIONS` and in-browser (`TypeError: Failed to fetch`).
- **Fix (`radar.html`):** switched to `GET https://api.adsbdb.com/v0/callsign/<CALLSIGN>` (sends `ACAO: *`, no preflight). Mapped `response.flightroute.{origin,destination}` into the existing tooltip fields (`airport_codes`, `_airport_codes_iata`, `_airports[]`). Per-callsign GETs, cached in `routeMemory`, de-duped with a `routeInflight` set, 8 s `AbortController` timeout; failures drop the `pending` marker so a later sweep retries.
- **Verified (no hardware, local mock server):** `fetchAircraftData()` → adsbdb `GET 200` (no preflight, no adsb.lol request); IGO123 resolved to VIDP–VILH / DEL–IXL with correct short + detailed formatting; unknown callsign → `not_found`; browser console clean. `node --check` clean; `idf.py build` clean.
- **Pending:** on-device flash + real-browser check once Ethernet is connected.
## 2026-10-13 — Honest HUD status + dynamic title (`feat/hud-honesty-title`)

- **Report:** The HUD said `ACFT: 5` (later 7) while only 2–3 blips were on the radar; user asked why, then asked to (1) make the status show `ACFT / ON SCREEN / NO POS` and (2) make the title `Raga Radar (<closest major city>)`, falling back to `lat, lon`. Explicitly **build only — do not flash** (board offline).
- **ACFT vs drawn (root cause):** `ACFT` was `latestApiData.length` — the raw `/data/aircraft.json` array, which includes tracks with no decoded position. The radar only draws aircraft that pass `!ac.lat||!ac.lon` (radar.html `:461`), the altitude band, the range ring, and the sweep-crossed check. LEARNING L19.
- **Status line (`radar.html`):** now `SYNCED | ACFT: <raw> | ON SCREEN: <radarTargets.size> | NO POS: <no lat/lon>`, replacing the misleading single `ACFT` + `CACHED RTE`. Removed the now-unused `activeCacheCount`.
- **Title (`radar.html`):** added an embedded ~75-city Indian table + `haversineKm`/`nearestCity`/`updateTitle`; title = `Raga Radar (<city>)` within 150 km, else `Raga Radar (<lat>, <lon>)`. Called at init, from `loadReceiver()`, and immediately in `applyLocation()` after a location change. Span got `id="ui-title"`. LEARNING L20 (prefer an offline table over a geocoder).
- **Verified (no hardware, local mock server on :8766):** title = `Raga Radar (Hyderabad)` for the live receiver coords; `Raga Radar (Pune)` for Pune; `Raga Radar (-33.00, -70.00)` for a remote point. Status after a sweep: `ACFT: 5 | ON SCREEN: 3 | NO POS: 2`. HTML parsed clean (34 unique ids, no nesting errors); both scripts `node --check` OK; `idf.py build` clean (56% free).
- **Pending:** flash when the board is connected; then confirm on the real page (title should read the configured city).

## 2026-10-13 — Settings parser + 2.4 MSPS decode fixes (`fix/settings-parser-2p4msps`)

- **Trigger:** While diagnosing "why only 25–35 NM?" the user (who does not accept trial-and-error) asked to land **all** identified fixes in code before connecting the device to USB. Build-only; no flash. Investigation had already proven there is **no software range limit** in the UI or firmware — Range/Sweep are client-side (`radar.html`), and the only distance check is a display filter (`radar.html:537`).
- **Bug 1 — `/api/settings` POST mis-parsed booleans (`http_radar.c`):** each key was found with `strstr(buf,"\"key\"")` then the value with `strstr(p,"true"/"false")`, which scans the *entire rest of the body*. So `"adaptive_gain":false` followed by `"aggressive":true` read back as `true`; same for `dc_filter`. Added `json_get_value()` (bounded between `:` and the next `,}`]`, trimmed) and routed every field through it; `gain_mode` still accepts `0/1` or `manual`/`adaptive`. LEARNING L21.
- **Bug 2 — 2.4 MSPS decoded 0 frames (`adsb_decoder.cpp:239-240`):** `interpolate()` indexes in milli-samples, so 1 bit (1.0 µs) = `rate×1000` = 2048 @2.048 MSPS / **2400** @2.4 MSPS. The 2.4 branch had `bit_spacing=1200`, `half_bit_spacing=600` (half a bit). Fixed to `2400`/`1200`; corroborated by the already-correct `kFrameSamples=288` and `first_center≈8 µs`. LEARNING L22.
- **Cleanup — duplicate "Adaptive Gain" control:** the checkbox was vestigial (`[`adaptive_gain_step`](rtl_pipeline.c:280)` gates on `gain_mode`), so it could never actually change behaviour. Removed it; **Gain Mode** (Manual/Adaptive) is the single control and `saveSettings()` derives `adaptive_gain` from it. `settingsForm`/`loadSettings`/listener references cleaned up.
- **RF finding (not code):** driving the live device (192.168.1.39) in the browser, a manual gain sweep 496→100 showed `fps` ~1.25–1.75 at high gain, **0 at ≤180**, `signal` never above ~12, and max reach pinned ≈18 NM — with `overruns 0`, `drops 0`. The pipeline is healthy and gain clearly works; the very low RF level points at the **antenna/cable/LNA chain**, not software. Range ceiling is an RF problem to solve on the roof, not in firmware.
- **Verified (no hardware):** HTML parses (33 unique ids, 0 dup), both `<script>` blocks `node --check` OK, concatenated scripts OK (no cross-script redeclarations), `onclick` handlers (`toggleMenu`/`toggleSettingsPanel`/`closeTarget`) all defined, `idf.py build` clean (56% free, only the pre-existing `quiet_len` unused-but-set warning in `adsb_decoder.cpp:232`).
- **Pending:** flash when the user connects USB; then (a) confirm 2.4 MSPS now decodes frames, (b) confirm DC Filter/Adaptive Gain toggles persist, (c) continue RF diagnosis for range.

## 2026-10-13 (later) — Live decode diagnosis: two real bugs found, gain characterized

- **Context:** Flashed the 6.13 build, connected USB serial (`/dev/cu.usbmodem5B900947191`) and later Ethernet (device at **192.168.1.39**, reachable in the browser). Goal: justify why the device decoded far fewer flights than the phone (~17 at 60-100 NM vs 3-4 at 25-35 NM).
- **Data (before):** 2.048 MSPS, gain 40 dB (NVS), adaptive mode; `pre≈60/s`, `crc≈2-3/s`, tracks 2-8, `mag=[0,55]`; log flooded with `task_wdt: IDLE1 (CPU 1)`; the 1 Hz status line barely appeared.
- **Bug 1 — decoder dropped ~half of every IQ block (L23):** `kMagnitudeCapacity=16640` was sized for the old 32 KiB block, but the pipeline hands the decoder `IQ_BLOCK_BYTES = 64*1024` = 32768 sample-pairs, so everything past sample 16640 was discarded silently. Split `process_cu8()` into sub-runs calling a new `process_pairs()` (DC filter + overlap carried across). Immediate ≈2× in crc rate.
- **Bug 2 — status/gain task starved (L24):** `rtl_driver`(5), `adsb_dsp`(6), `rtl_stat`(3) were all on core 1; the saturating DSP starved `rtl_stat`, so gain changes/status never ran and core-1 IDLE tripped the WDT every ~5 s. Moved `rtl_stat` to core 0 and disabled the idle-CPU1 WDT. Status now prints every second; WDT flood gone (0 hits).
- **Gain characterization (L26):** two back-to-back manual sweeps measuring crc/s over 15 s windows → **35.0 dB optimal** (7.49 & 4.80 crc/s), 40 dB and 49.6 dB clearly worse (more gain → more noise preambles, `pre` up to +80/s while crc fell). Set the device to **2048000 sps / 35.0 dB / manual / dc_filter+aggressive+crc_fix ON** (NVS-persisted via `/api/settings`).
- **2.4 MSPS (L25):** still unusable — `pre` collapses to ~+5/s, crc ~0. This **corrects L22**: the 2400/1200 bit-spacing fix was necessary but not sufficient; the failure is in preamble detection (hypothesis: `quiet_start=19` lands on the first data bit rather than the guard). Reverted the device to 2.048 MSPS. Marked 2.4 MSPS as unsupported.
- **Result:** with the fixes + outdoor antenna + 35 dB, observed **11-14 tracks concurrently** (user reported up to ~19), max decoded range ~47 NM, `rf=OK`, `over=0 drops=0`, crc ~6-15/s depending on traffic. Huge improvement over the starting 3-4 aircraft.
- **Open item:** `/api/settings` "defaults" are `gain 496 / all filters off` — "Reset to Defaults" picks the worst-tested config (not changed).
- **Honest caveat:** ADS-B traffic varies minute-to-minute, so absolute crc/s numbers are only comparable within a single back-to-back sweep; the gain ranking is robust (two sweeps agree) but the exact crc/s values are not.
- **Pending:** commit these fixes on the branch and (optionally) tag after the user's final device check.
