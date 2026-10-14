# Learning Log — what went wrong and how we fixed it

Honest record of the mistakes made during the ADS-B bring-up so they are never repeated.
Each entry: symptom → root cause → fix → the rule to follow forever.

---

## L1. Hand-written file bodies corrupted — draft junk leaked into files

- **Symptom:** Multiple source files (`rtl_pipeline.c`, `adsb_decoder_bridge.cpp`, headers) kept "reappearing corrupt" — stray tokens (`Decoder boy`, `pump_stats`, nonsense) inside code.
- **Root cause:** The corruption was NEVER on disk. The on-disk files were byte-clean the whole time. The garbage lived inside the **agent's own hand-typed write()/heredoc drafts**. Any file produced from a human-authored body by hand was suspect; files produced via `perl -pi` / `python3` heredocs / small targeted edits were byte-exact.
- **Fix:** Verify on-disk reality first (`cat`/`Read`), trust it, and only ever make **small mechanical edits** (`perl -pi`, targeted `Edit`). Never rewrite a whole file from memory.
- **Rule:** The file on disk is the truth. Read before editing. If a whole-file rewrite is ever truly needed, generate it programmatically (python) and `cat` it back to confirm byte-for-byte.

## L2. Wrote code against an invented API instead of the real header

- **Symptom:** 6 compile errors in `rtl_pipeline.c` — `ESP_RTL_SDR_STATE_READY` undeclared, status struct fields (`driver_started`, `overruns`, `ring_free`, `crc_ok`) that don't exist, duplicate `IQ_RING_SLOTS`.
- **Root cause:** The pipeline was written against an API **recalled from memory / inferred**, not the actual on-disk driver header. The real driver has NO `STATE_READY` (states are `UNINSTALLED/IDLE/STREAMING/STOPPING/FAULT/STARTING`) and the status struct uses `stream_requested/overrun_cnt/ring_free_slots/crc_ok_cnt`.
- **Fix:** `grep`/`Read` the real header, then `perl` the .c to match the .h exactly. Build once, let the compiler enumerate every remaining mismatch, fix them all in one pass.
- **Rule:** NEVER invent API. The header (`*.h`) on disk is the contract. Verify symbol names/fields before writing a single call site.

## L3. Reinventing the wheel against a proven reference

- **Symptom:** Many hours rewriting driver glue and a decoder when the user pointed at working repos (`SAMS0N1TE/esp32p4-rtl-sdr-v4`, `kvhnuke/esp32-rtl-sdr`).
- **Root cause:** We built a parallel, from-scratch architecture instead of first checking what the proven code for this exact SoC+dongle does (raw `rtlsdr_open`/`set_center_freq(1090000000)`/`set_sample_rate` + AUTO tuner gain + `demod1090`).
- **Fix:** Cloned both repos to `/tmp`, read their call graph, and used their proven config (1090 MHz, R820T2, decode on CPU1) as the ground truth for RF-side decisions.
- **Rule:** Before writing fresh DSP/driver code, look for a working reference for the same hardware. Use the proven config as the baseline; only diverge with evidence.

## L4. Interactive `menuconfig` in a non-interactive shell → hang

- **Symptom:** `idf.py menuconfig` hung in the tool and had to be killed (120 s timeout).
- **Root cause:** Menuconfig is a TUI needing a real terminal; and even with a TTY it blocks waiting for input. Also, changes made in menuconfig don't round-trip into a scripted flow.
- **Fix:** Toggle every `sdkconfig` value by **editing `sdkconfig` directly** with `python3` regex, then `idf.py build`. Rebuild regenerates dependent configs.
- **Rule:** Never run interactive config tools in automation. Edit `sdkconfig`/`sdkconfig.defaults` as text, then rebuild.

## L5. Flashing rejected: bootloader wants chip rev 3.0/3.1, chip is v1.3

- **Symptom:** `esptool: 'bootloader.bin' requires chip revision in range [v3.1 – v3.99] (this chip is revision v1.3)`.
- **Root cause:** This DevKit's ESP32-P4 is early silicon **rev v1.3**, but IDF's default minimum is 3.0/3.1. The Kconfig choice only lists 3.0/3.1 unless `ESP32P4_SELECTS_REV_LESS_V3` is enabled, which unlocks v0.x/v1.0 choices.
- **Fix:** `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`, `CONFIG_ESP32P4_REV_MIN_100=y`, `REV_MIN_FULL=100`. Then the bootloader accepted the v1.3 chip.
- **Rule:** When esptool refuses, it is telling you the truth — read the chip's actual revision from the error, then set the matching `REV_MIN`.

## L6. Boot panic `sp_clk_init.c:105 (res)` — CPU freq unsupported on old silicon

- **Symptom:** After fixing the flash issue, the app crashed immediately at boot in `esp_clk_init` → PSRAM/clk bring-up (MCAUSE=2).
- **Root cause:** `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=400` — the 400 MHz CPU frequency is only valid on rev ≥3.0. On rev<3.0 the CPLL only provides 360/180/90.
- **Fix:** `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=360`. Confirmed in `rtc_clk.c` that 400 is gated behind `ESP32P4_SELECTS_REV_LESS_V3` being unset.
- **Rule:** On pre-v3.0 P4, CPU = 360 MHz max. When bring-up panics in clock code, check the freq table for the selected silicon revision first.

## L7. Task WDT starved while driver enumerated USB

- **Symptom:** `task_wdt: got triggered — IDLE0 (CPU 0)` mid-boot; driver's USB probe blocked on CPU0 for ~5 s.
- **Root cause:** The high-priority USB CLI/probe work on CPU0 prevented the IDLE0 task from feeding the watchdog during enumeration.
- **Fix:** `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=n` (probe is one-shot at boot, legal CPU0 work).
- **Rule:** If a legitimate high-priority boot path starves an idle task, disable that idle check rather than fighting priority tuning.

## L8. `start()` called before the dongle existed

- **Symptom:** `start failed: ERROR` 10 ms after install; then the dongle appeared ~5 s later and streaming never started.
- **Root cause:** The wait-loop keyed on driver state `== IDLE`, which is true immediately after install even with no device. The real "device accepted" signal is the `EVT_READY` event → `device_present` flag.
- **Fix:** Wait on `s_ctx.device_present` (set by `EVT_READY`, which also signals `fsm_sem`), plus 3 `start()` retries.
- **Rule:** For hotpluggable devices, wait on the device-ready **event flag**, never on a generic state value. Retry the start.

## L9. RF interpretation: confusing "no frames" with a firmware bug

- **Symptom:** After the stream went clean, 0 preambles/frames caused repeated gain flailing (AUTO → 30 → 44 → 49.6 dB).
- **Root cause:** The decoder pipeline was healthy; the difference was **RF signal strength** (`mag` max only ~43 of 255; the one frame all session came at 44 dB). No amount of software could create aircraft bursts that aren't reaching the antenna (window position, sky view, ADS-B traffic density).
- **Fix:** Use the `mag=[min,max]` stat as the RF health signal. Max `mag` near 255 = saturation (lower gain); max ~40 with 0 frames = weak RF (not code). Record the evidence, pick the best gain (44 dB), and validate the DSP path with the decoder's own `self_check()` so firmware is exonerated.
- **Rule:** Separate "is the firmware correct?" from "is the signal strong enough?" Use independent metrics (`self_check`, `mag` range, USB eff) to prove which one you're chasing before touching code.

## L10. Reference repos left unrecorded

- **Symptom:** Could spend time re-discovering the same P4-specific knowledge.
- **Root cause:** Proven info (rev-1.3 quirks, CPU-1 decode pinning, transfer-size/tuner-gain hints) lived only in conversation.
- **Fix:** `docs/SESSION_SUMMARY.md` (state + changes) and `docs/TASKS_TRACKER.md` platform notes N1–N5 now capture it; this LEARNING.md records the how-to-avoid.
- **Rule:** Any hard-won platform fact goes into the tracker/platform notes in the same session it was learned.

---

## L11. Dropdown value mismatch — API returns strings but options have numeric values

- **Symptom:** Gain Mode dropdown appears blank on load and after save, despite having valid options (`0`=Manual, `1`=Adaptive). Other dropdowns (Sample Rate) work correctly.
- **Root cause:** `/api/settings` returns `gain_mode: "manual"` or `"adaptive"` (strings), but the dropdown `<option>` values are `"0"` and `"1"`. Setting `select.value = "manual"` finds no matching `<option value="manual">`, so browser shows blank.
- **Fix:** In `loadSettings()`, map both numeric and string values to dropdown values:
  ```javascript
  const gm = s.gain_mode;
  if (gm === 0 || gm === "0" || gm === "manual") settingsForm.gainMode.value = "0";
  else if (gm === 1 || gm === "1" || gm === "adaptive") settingsForm.gainMode.value = "1";
  else settingsForm.gainMode.value = "0";
  ```
  In `saveSettings()`, send numeric values to backend:
  ```javascript
  gain_mode: parseInt(settingsForm.gainMode.value, 10)
  ```
- **Rule:** Dropdown option `value` attributes must exactly match the API response format. When API returns different formats (string vs numeric), normalize in the client before setting `select.value`.

## L12. Settings auto-save on change causes UX friction

- **Symptom:** Every change in settings panel immediately triggered a save, causing device restarts on sample rate change and spurious network traffic.
- **Fix:** Remove auto-save on `change` events. Use explicit "Save Settings" button. Only sample rate change requires device restart (handled by firmware).
- **Rule:** Settings that require restart should be explicit. Auto-save is appropriate only for non-destructive settings.

## L13. Sample Rate dropdown visibility — template consistency

- **Symptom:** Sample Rate dropdown worked correctly while Gain Mode didn't, despite similar structure.
- **Root cause:** Sample Rate API returns numeric values (`2048000`, `2400000`) that exactly match option values (`"2048000"`, `"2400000"`). Gain Mode returned strings (`"manual"`, `"adaptive"`) not matching option values (`"0"`, `"1"`).
- **Rule:** Maintain consistent value types between API responses and dropdown option values. If backend changes, update both ends or add client-side normalization.

## L14. Live credential committed into docs, then published in a public repo

- **Symptom:** After making the repo public, `docs/TASKS_TRACKER.md` was found to contain `WPA2 key raga@098` — the same string used as the Raspberry Pi SSH and `sudo` password.
- **Root cause:** A decision-table row ("AP credentials") recorded a real secret verbatim. It was harmless while the repo was local-only, but publishing made it world-readable, and it also lives in git history (redacting the current file does not remove the old commits).
- **Fix:** Redact the literal in the working tree; treat the secret as compromised and **rotate it** (change the Pi password). Verify with a repo-wide `grep` for `password|secret|token|wpa2|sshpass|ghp_` before publishing.
- **Rule:** Secrets never belong in source, docs, or example configs — even in "cancelled"/historical rows. Publishing is irreversible; grep for credentials before the first push, and rotate anything that ever slipped in.

## L15. Two corner-anchored HUD panels overlapped on mobile

- **Symptom:** On a phone, the main control panel (`#ui`, top-left) and the Settings panel (`#settings-panel`, top-right) overlapped and the Settings panel covered the NM/Sweep controls.
- **Root cause:** Both panels were `position:absolute` pinned to opposite corners. The `@media (max-width:768px)` block only restyled `#ui` (to `width:85vw`) and **never touched `#settings-panel`**, which kept `min-width:250px`; at ~390px viewport the two boxes intersected, and the later-in-DOM panel painted on top. The collapsed `#ui` also stayed 85vw wide, so even closed it collided.
- **Fix:** Wrap both panels in `#hud-stack`. Desktop keeps the exact corner layout via `#hud-stack{display:contents}` (wrapper generates no box). On small screens the wrapper becomes a `position:fixed; display:flex; flex-direction:column; max-height:100dvh; overflow-y:auto` column, so panels flow vertically and cannot overlap; each gets `box-sizing:border-box; width:100%`. Moved the Settings panel's positional styles out of the inline `style=` (inline would beat the stylesheet) into a `#settings-panel` rule so mobile can override. Added `(max-height:520px)` to the media query so landscape phones (which are >768px wide) also stack.
- **Rule:** Never anchor two variable-height HUD panels to two corners and assume a max-width media query fixes mobile — restyle **every** panel, prefer a single flex/stack container, keep layout rules in the stylesheet (not inline) so they can be overridden, and verify with measured bounding-client-rects at multiple viewport sizes, not by eye.

## L16. Browser geolocation does not work on plain-HTTP LAN origins

- **Symptom:** "Use Browser Location" would fail on the device's `http://192.168.200.102/` page even though the same code works elsewhere.
- **Root cause:** `navigator.geolocation` requires a **secure context** (HTTPS, or `localhost`). A device served over plain HTTP on a LAN IP is not secure, so Chrome/Safari/Firefox reject the call. The browser does not fail loudly — the error callback fires.
- **Fix:** Treat geolocation as best-effort: try it, and on failure show an explicit "enter lat/lon manually" message. Manual entry is the reliable path and must always be available. (Serving self-signed HTTPS is possible but larger scope.)
- **Rule:** Never depend on a secure-context-only browser API for a device UI served over plain HTTP. Provide a manual fallback for every such feature, and tell the user *why* the convenience path failed.

---

## L17. Collapsing a HUD panel left `#status` / `#user-location` stranded on screen

- **Symptom:** "Raga Radar - Pune" panel would not "fully collapse" — after tapping the header, the controls hid but the `SYNC ERR: Failed to fetch` and `User GPS: …` lines stayed visible, making the panel look un-collapsed (reported on mobile).
- **Root cause:** `toggleMenu()` only toggled `#ui-content`. `#status` and `#user-location` were siblings inside `#ui` but **outside** `#ui-content`, so they were never hidden. Any panel body element placed next to the toggled container is invisible to the collapse logic.
- **Fix:** Centralised `applyMenuState()` toggles `#ui-content`, `#status`, and `#user-location` together (and drives the `[+]/[-]` label), called both on load and on every toggle.
- **Rule:** A collapsible container must include **every** element that should hide. When you add a new line/section to a collapsible panel, either put it inside the toggled element or extend the toggle to cover it — and verify all three states (initial, expanded, collapsed) at desktop and mobile widths.

---

## L18. Route lookup failed: `api.adsb.lol/api/0/routeset` sends no CORS headers

- **Symptom:** Browser console showed `TypeError: Failed to fetch` (looked like "CORS or something") and route data never appeared; the failure was swallowed by the page's `try/catch`, so it was hard to see.
- **Root cause:** The page is served over `http://` on a LAN IP (not `localhost`). A `POST` with `Content-Type: application/json` triggers a CORS **preflight** (`OPTIONS`). `api.adsb.lol/api/0/routeset` answers the preflight with `201` but sends **no** `Access-Control-Allow-Origin`, so the browser blocks the request entirely. (Verified with `curl -X OPTIONS … -H "Origin: …"`.)
- **Fix:** Switched the route lookup to `https://api.adsbdb.com/v0/callsign/<CALLSIGN>` — a plain **GET** that sends `Access-Control-Allow-Origin: *` (no preflight), and mapped its `response.flightroute.{origin,destination}` into the shape the tooltip already expected (`airport_codes`, `_airport_codes_iata`, `_airports[]`). Per-callsign GETs replace the single batched POST; results are cached and de-duped via a `routeInflight` set, with an 8 s `AbortController` timeout.
- **Rule:** Before wiring any third-party API into a browser page, `curl -D -` it (including an `Origin` header, and an `OPTIONS` preflight for non-simple requests) to confirm it actually sends CORS headers. Prefer simple `GET`s with `Access-Control-Allow-Origin: *`. If the API can't do CORS, a browser cannot call it — the only options are a server-side proxy or a different endpoint — never assume the fetch "just works".

---

## Recurring meta-pattern (the big one)

Almost every wrong turn above traces to **not verifying reality before acting**:
- not reading the file before rewriting (L1),
- not grepping the header before calling the API (L2),
- not checking the reference repo before building our own (L3),
- not reading the chip revision from the esptool error (L5),
- not reading the CPU-freq table for the silicon (L6),
- not reading the mag stat as an RF gauge (L9),
- not matching API response format to dropdown option values (L11).

**The discipline: READ FIRST, VERIFY, THEN EDIT. Every tool answer is data. Interpret the data before changing code.**
---

## L19. "ACFT" counted raw tracks, not plotted blips

- **Symptom:** The HUD read `ACFT: 5` (or more) while only 2 dots were visible on the radar; the user reasonably read this as "the decoder is missing flights".
- **Root cause:** The status line used `latestApiData.length` — the length of the raw `/data/aircraft.json` array. That array includes every *track* the decoder currently holds, including ones with no decoded position (Mode-S-only / CPR not yet resolved). The radar only draws an aircraft that passes `if (!ac.lat || !ac.lon) return;`, then the altitude band, the range ring, and finally the sweep-crossed check. So `ACFT` was never the on-screen count.
- **Fix:** Status line now reports three honest numbers: `ACFT: <raw tracks> | ON SCREEN: <radarTargets.size> | NO POS: <tracks without lat/lon>`.
- **Rule:** A label must name what it actually measures. When a count that "looks like N" is derived from a raw feed, separate the raw total from the filtered/rendered subset, or you will chase phantom decoder bugs.

---

## L20. Reverse-geocoding for the title: use an offline city table, not an API

- **Context:** The HUD title was a hardcoded "Raga Radar - Pune" and did not follow the configured receiver location.
- **Decision:** Label the location with the **nearest major city from an embedded table** (Haversine over ~75 Indian city centroids) and fall back to `lat, lon` when nothing is within 150 km. This keeps the title working with **no internet** and avoids another CORS/rate-limit dependency (cf. L18).
- **Rule:** For a small, bounded, latency-free lookup (labeling a location), prefer a bundled table over a network geocoder. Reserve external APIs for data that genuinely cannot be embedded.

---

## L21. Settings POST parser matched `true`/`false` anywhere in the body

- **Symptom:** Toggling **DC Filter** or **Adaptive Gain** off in the web UI appeared to do nothing; `/api/settings` kept echoing `true`. Turning off one switch seemed to silently re-enable it.
- **Root cause:** `h_settings_post()` located each key with `strstr(buf, "\"key\"")`, then found the `:` and ran `strstr(p, "true")` / `strstr(p, "false")` from that point. `strstr` scans the *rest of the whole body*, not the value of that key. So `"adaptive_gain":false` followed later by `"aggressive":true` was parsed as `adaptive_gain = true`. Any `false` field whose name sorts before a `true` field was corrupted. (Both `dc_filter` and `adaptive_gain` are affected.)
- **Fix:** Added `json_get_value()` — a tiny bounded extractor that reads from the `:` up to the next `,`/`}`/`]`, trims spaces, and returns just that value. All fields (`gain`, `gain_mode`, `sample_rate_sps`, `dc_filter`, `adaptive_gain`, `aggressive`, `crc_fix`) now compare the extracted token, not a global substring. Numeric `gain_mode` still accepts `0`/`1` or `manual`/`adaptive`.
- **Also:** `adaptive_gain` is **vestigial** — `adaptive_gain_step()` gates on `gain_mode` (`rtl_pipeline.c:280`), not `adaptive_gain`. The duplicate "Adaptive Gain" checkbox was removed; **Gain Mode** (Manual/Adaptive) is the single control, and the POST derives `adaptive_gain` from it. 
- **Rule:** Never `strstr` a JSON body for a boolean literal — it matches *anywhere*. Extract the value between delimiters for the specific key. Prefer a real parser if the payload ever grows.

---

## L22. Bit spacing must scale with sample rate (2.4 MSPS was half the right value)

- **Symptom:** Selecting **2400000** sps made the decoder produce **0 frames** while 2048000 sps decoded normally; `effective_sps` confirmed the pipeline really ran at 2.4 MSPS.
- **Root cause:** In `AdsbDecoder::detect()` the per-rate constants were `bit_spacing = 1200` and `half_bit_spacing = 600` for 2.4 MSPS, versus `2048`/`1024` for 2.048 MSPS. `interpolate()` indexes samples in **milli-samples**, so one Mode-S bit (1.0 µs) is `sample_rate * 1000 / 1e6` milli-samples: **2048** at 2.048 MSPS and **2400** at 2.4 MSPS. 1200 is half of a bit — every bit sample point landed mid-bit, so framing was garbage. The other 2.4 constants (`kFrameSamples = 288 = 120 µs × 2.4`, `first_center ≈ 8 µs`) were already consistent with 2400, which corroborated the bug.
- **Fix:** `bit_spacing = 2400`, `half_bit_spacing = 1200` (`adsb_decoder.cpp:239-240`).
- **Rule:** Any timing constant tied to sample rate must be derived from `sample_rate × time`, never hand-tuned per branch. When one constant in a branch disagrees with the others, suspect a typo, not a design.

---

## L23. Decoder silently dropped ~half of every IQ buffer (capacity < block size)

- **Symptom:** Steady decode of only ~2-3 frames/s at 2.048 MSPS, with `eff≈2.0 MSPS` and `over=0 drops=0` — no error raised, just a persistently low rate (easy to mislabel as "weak signal").
- **Root cause:** `AdsbDecoder::process_cu8()` copies each block into the fixed `magnitudes_` buffer under `if (count < kMagnitudeCapacity)`, with `kMagnitudeCapacity = 16640` — sized for the old **32 KiB** block. The pipeline now hands the decoder `IQ_BLOCK_BYTES = 64*1024` = **32768 sample-pairs** (the driver fills slots with the real URB length, up to 65536 bytes), so every sample past 16640 in each block was dropped — roughly **half of all IQ**, silently.
- **Fix:** `process_cu8()` now splits each block into sub-runs of `kMagnitudeCapacity - kMaxFrameSamples` pairs and calls a new `process_pairs()` per sub-run (DC-filter accumulators + `overlap_` carried across sub-runs). No samples dropped, no frame straddling a sub-run boundary missed. `kMaxFrameSamples = 288` (2.4 MSPS frame length).
- **Measured effect:** crc rate ≈ doubled immediately on the same RF (2-3/s → ~5/s); after moving the antenna outdoors, 2-3/s → ~15/s (burst) with tracks reaching 14-15 (user reported up to ~19).
- **Rule:** A fixed-size staging buffer must be sized against the *actual* producer block size. A silent `if (count < cap)` drop is worse than a crash — it masquerades as weak signal. When a producer block size changes, grep every consumer buffer that assumed the old size.

## L24. Status/adaptive-gain task starved on the saturated core 1

- **Symptom:** (a) `E task_wdt: IDLE1 (CPU 1) did not reset` every ~5 s (log flood); (b) gain changes from the web UI and adaptive stepping appeared to do nothing; (c) the 1 Hz status line rarely printed.
- **Root cause:** `rtl_driver` (prio 5), `adsb_dsp` (prio 6) and `rtl_stat` (prio 3) were **all pinned to core 1**. `adsb_dsp` runs the 2 MSPS decoder flat-out, so the prio-3 `rtl_stat` — which applies manual/adaptive gain and logs status — was starved, and core-1 IDLE never ran (`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=y`).
- **Fix:** moved `rtl_stat` to **core 0** (prio 4, stack 3072). Status now logs cleanly every second and gain changes apply; also disabled `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1` (a saturated core-1 idle is expected; decode/CRC continue), which stopped the WDT flood.
- **Rule:** On a dual-core MCU never co-locate a saturating DSP task and the housekeeping that services it on the same core at lower priority. Don't watch the idle task of a core you intend to keep saturated.

## L25. Corrects L22: 2.4 MSPS is still unusable (preamble detection, not bit spacing)

- **Correction to L22:** setting `bit_spacing`/`half_bit_spacing` to 2400/1200 was necessary but **not sufficient**. On the live device 2.4 MSPS still yields almost nothing: preamble count collapses from ~**+45/s** (2.048) to ~**+5/s**, crc ~0/s, while `eff≈2.39 MSPS` and `over=0 drops=0`.
- **Where it fails:** `pre` is incremented *before* any bit decoding, so the fault is in preamble detection. The 2.4 branch uses `pulse_idx = {1,3,9,11}`, `first_center ∈ [17000,21000]`, `quiet_start = 19`. **Hypothesis (unverified):** `quiet_start = 19` samples = 7.9 µs sits at the start of the first data bit (8 µs = 19.2 samples) instead of inside the 4.5-8 µs guard, inflating the "quiet" level so the pulse-over-quiet threshold rejects real preambles; the ~2.4-sample-wide 2.4 MSPS pulses may also tie with neighbours under the strict `<=` local-max test.
- **Status:** **2.4 MSPS is not a supported path.** Keep the default 2048000. A real fix needs the 2.4 preamble constants re-derived (quiet window in the guard, pulse-peak indices) and verified on live traffic.
- **Rule:** A fix is not proven until the real signal path exercises it. Mark partially-verified branches explicitly rather than declaring them done.

## L26. Gain is objective-dependent: 35 dB maximises frame rate, ~42 dB maximises range

> **Relabelled 2026-10-13** — the original conclusion below optimised **crc/s only**, which hid a large range trade-off. Since the goal is range/track coverage (not raw frame throughput), the operational gain is **≈42.1 dB**. Original sweep data retained.

- **Method:** With the fixed pipeline, two back-to-back manual-gain sweeps measured crc/s over ~15-16 s windows (traffic drifts slowly, so adjacent windows are comparable):
  - Sweep 1 (desc): 496→4.57, 450→3.91, 400→5.91, **350→7.49**, 300→5.74 crc/s
  - Sweep 2 (asc): 300→3.37, 325→3.24, **350→4.80**, 375→4.22, 400→2.68 crc/s
- **Frame-rate view:** **≈350 tenths-dB (35.0 dB) gave the highest crc/s.** 40 dB and 49.6 dB were clearly worse on that metric. At 49.6 dB `pre` rose to ~+80/s (more noise preambles) while crc *fell* — extra front-end gain lifted the noise floor faster than the signal.
- **Correction (2026-10-13) — crc/s was the wrong sole metric.** Re-measured on the live device (2048 kSPS, dc_filter/aggressive/crc_fix ON), holding everything but gain constant:

  | gain | crc/s | tracks | sample swing | max decoded range |
  |---|---|---|---|---|
  | 35.0 dB | ~5.5 | 3-5 | `smp=[101,152]` | 57 NM |
  | **42.1 dB** | ~2.7 | 4-6 | `smp=[80,181]` | **94 NM** |

  Higher gain widens the sample swing and **nearly doubles range** (57 → 94 NM) and gives more positioned tracks, while **halving the raw frame rate** (more noise preambles → more CRC rejects). "Best gain" therefore depends on the objective: ~35 dB for maximum frame throughput, **~42 dB for maximum range/reach**. Operational choice is **42.1 dB**.
- **Footgun (open item):** `GET /api/settings` reports `defaults{ gain:496, dc_filter:false, aggressive:false, crc_fix:false }` (`http_radar.c:258-264`) — **"Reset to Defaults" selects the worst-tested configuration** (max gain, all filters off). Not changed; noted for follow-up.
- **Live state after tuning:** 2048000 sps, **42.1 dB manual** (chosen for range), `dc_filter`/`aggressive`/`crc_fix` ON, persisted to NVS. `rf=OK`, `over=0 drops=0`, `eff≈2.045 MSPS` (99.9%).
- **Rule:** For a noise-limited receiver, gain is an *objective-dependent* trade-off, not a single optimum. Benchmark against the metric you actually care about (here: range/track coverage), not the one that is easiest to log (frames/s).

## L27. `/api/status` reported `phase:"idle"` and `programmed_sps:"0"` forever

- **Symptom:** While building a UI that gates alerts on `phase === "streaming"`, the raw `GET /api/status` showed `"phase":"idle"` and `"programmed_sps":"0"` even while the device was streaming at 2.048 MSPS with tracks on screen. The HUD's `RX:` field also always read `idle`.
- **Root cause:** `rtl_pipeline_get_status()` (`rtl_pipeline.c`) never assigned `out->phase` nor `out->programmed_sps`, so the `memset(out,0,…)` left `phase = RTL_PIPELINE_IDLE (0)` and `programmed_sps = 0`. The `RTL_PIPELINE_{OPTAINING,STREAMING,DEVICE_LOST,DEFUNCT}` enum and the `phase`→string switch in `http_radar.c` were effectively dead code — nothing ever populated the field.
- **Fix:** derive phase in `get_status()` from the real flags (`device_present`/`started_req`): idle → obtaining → streaming → device-lost; and set `programmed_sps = s_ctx.metrics.sample_rate_sps`.
- **Why it mattered:** any consumer that trusted `phase` silently mis-classified a healthy, streaming receiver as idle — e.g. it would have suppressed every "streaming-only" diagnostic warning.
- **Rule:** an enum + formatter that is never fed inputs is a latent trap. When adding a field to a status struct, grep for where it is *written*, not just where the struct type is declared — a zero-filled struct happily serialises as a plausible-looking default.


## L28. 2.048 MSPS decodes ZERO messages; 2.4 MSPS is the only working ADS-B rate (corrects L22, L25)

The receiver produced preambles but almost no CRC-valid frames. After ruling out gain,
antenna and USB, the same **Nooelec NESDR Nano 2+ + antenna was run through real `readsb`
on the Mac**:

| Sample rate | readsb preambles | correct CRC |
|---|---|---|
| 2.048 MSPS | 72k–140k | **0** (every gain 15–49.6 dB, ±10 ppm) |
| 2.4 MSPS | 96k | **780** |

**2.048 MSPS cannot decode Mode-S on this R820T2 with ANY decoder**, readsb included. So L22
("bit spacing must scale with sample rate") and L25 ("2.4 MSPS is still unusable") were both
incomplete/mistaken — the real blocker was the **detector algorithm**, not the geometry.

**Root cause:** `adsb_decoder.cpp` used a hand-rolled detector — `|I|+|Q|` magnitude, fixed
pulse indices `{0,2,7,9}`, a single fixed slicing phase searched over a 900 milli-sample
window, and a raw 2-point compare `interp(center) > interp(center+half_bit)`. Fixed sample
indices cannot track the arbitrary sub-sample phase of real IQ, so it found pulse-ish things
but sliced the wrong bits and CRC never passed.

**Fix:** ported readsb's proven `demodulate2400` into `adsb_decoder.cpp`:
- linear amplitude `sqrt((I-127.5)² + (Q-127.5)²)` normalised to 0..65535 (not `|I|+|Q|`)
- DC removal as a **1 Hz** single-pole high-pass (`dc_b = exp(-2π/sr)`), not a 1/4096 mean
- noise-adaptive preamble gate `ref_level = base_noise * preamble_threshold / 32`
- **5-phase** search with DC-balanced correlation slicers `slice_phase0..4` (`slice_byte`
  advances 19 samples/byte, 20 on wrap), picks the best-scoring phase
- skip `msgbits*8/4` samples after a hit so a frame is not re-decoded at every bit offset

Also: NVS already stored 2048000 on existing units, so `settings_load()` **migrates any
stored 2048000 → 2400000** (existing hardware self-heals). `/api/settings` now rejects
2048000 outright.

**Result:** 0 CRC/s and 0 tracks → ~5 CRC/s and **9 tracked aircraft (4 positioned)**.
Verified offline on the real capture before flashing: 650 CRC / 289 distinct aircraft vs
readsb's 883 / 419.

**Lesson:** a green `self_check()` proved nothing about reception — it only exercised
`parse()`/CRC on hardcoded bytes and explicitly *skipped* the live slicing path
("live_magnitudes test skipped"). Always validate the IQ→bits→frame path against a **real
capture**, and let a known-good reference decoder produce the expected number first.

## L29. RF health latched a permanent false "clipping" alarm and talked the user into killing reception

`metrics.sample_min/sample_max` were a **running cumulative min/max that `start()` never
reset** (only `bytes_total/blocks_total/overruns/consumer_drops` were cleared). One
transient spike during boot pinned the rails forever, so health reported
`sample=[0,255] / rf=CLIPPING / "RF clipping - reduce gain"` **while 9 aircraft were decoding
perfectly**. Following that advice drove gain to 0 dB, which genuinely produced zero aircraft.

Two more defects in the same check: the min/max scan used `i += 64` (32 samples per 64 KiB
block, **I channel only, Q never inspected**), and a single block tripping
`max>=250 && min<=8` was enough.

**Fix:** rolling window of the last 32 blocks (~0.4 s at 2.4 MSPS), full-block scan of I
and Q, window cleared in `start()`, and a verdict now needs **8 consecutive** rail-pinned
(or dead-flat) blocks.

**Lesson:** a health check whose advice is "reduce gain" must be near-impossible to false
trigger, and any *latched* metric must be reset when the stream restarts. Diagnostic output
that persists across restarts will eventually contradict reality and be believed.

## L30. `ring_free == 0` means IQ is being dropped before it is ever decoded

`/api/debug` showed `ring_free` pinned at 0 of 8 slots while bytes flowed at a full
4.8 MB/s. The pipeline ring (8 × 64 KiB) is filled by the driver callback and only drained
by `adsb_dsp`; when the decoder cannot keep up, `ring_take_free()` returns `UINT32_MAX` and
blocks are **silently discarded** — `overruns`/`drops` stay 0 because the driver thinks it
delivered fine.

Cause: firmware is built with `CONFIG_COMPILER_OPTIMIZATION_DEBUG=y` → **`-Og`** for every
translation unit, including the per-sample DSP loop. Deliberately left as-is for now
(whole-firmware optimisation is a separate, separately-measured change), but it is the
leading suspect for the decode-rate variance between ~0.7/s and ~5/s.

## L31. Per-sample sqrt was the throughput ceiling; AMB fixed it (+152% CRC rate)

After the 2.4 MSPS fix, `/api/debug` showed `adsb_dsp` processing only
`samples_per_s=576,717` against `required_sps=2,400,000` - **~24% of the stream**,
with the rest discarded by the ring (`busy_pct` ~40-88%). The per-sample
magnitude was the dominant cost:

```c
float magsq = fI*fI + fQ*fQ;
mag = std::sqrt(magsq) * 65535.0f;   // ~2.4 M sqrt/s
```

On x86 that is one `sqrtsd`; on the ESP32-P4's **RV32F it is a microcoded software
routine** (a `call sqrt`), so the cost is far higher there. Confirmed by compiling
for `rv32imafc`: the per-sample path compiles to a library call.

Replaced with **Alpha Max Plus Beta Min**, integer-only:

```
mag ~= max(|I|,|Q|) + min(|I|,|Q|)/4      scaled x415 -> full scale ~= 65535
```

The scale is chosen so the magnitude stays in the **same unit range** the readsb
`slice_phase0..4` correlations and the noise-adaptive preamble threshold are
expressed in - so **no slicer re-tuning was needed**.

**Measured on hardware** (192.168.1.6, 42.1 dB manual, 2-minute average):

| metric | before | after |
|---|---|---|
| CRC rate | 11.00/s | **27.72/s** (+152%) |
| samples_per_s | 576,717 | **1,139,780** (+98%) |
| share of stream | ~24% | ~47% |
| overruns / tracks | 0 / 19 | 0 / 19 (held) |

**Validate the magnitude path across WEAK signals, not just a strong capture.**
The earlier `int-sqrt` lookup table (L-less, reverted) passed on the full-strength
capture and then decoded **nothing** at weak signal - a coarse table quantised weak
blocks to ~5 distinct levels, so the integer peak tests `pa[1] > pa[7]` could not
resolve a preamble. Re-testing the same capture **amplitude-scaled** across the
levels the device really sees shows why AMB is safe and that table was not:

| sample swing | float distinct | AMB distinct | int-sqrt |
|---|---|---|---|
| 108 | 289 | **304** | 289 (full strength only) |
| 54 | 241 | **264** | - |
| 27 | 144 | **199** | - |
| 10 | 36 | **92** | **0 (device died)** |

AMB stays *continuous* in the input, so weak signals keep low-end magnitude
resolution; it is also strictly better than float at every level.

**Lesson:** a magnitude/magnitude-like change is only validated when it is exercised
at the signal levels that actually occur. Benchmark on the target ISA, not the
host - the sqrt ranking inverts between x86 and RV32F.

## L32 — Adding HTTP routes silently disables the whole server (`max_uri_handlers`)

Adding two routes to the radar web server (`/api/feeder` GET + POST) took the
total from 12 to 13. `httpd_config_t.max_uri_handlers` was set to exactly `12`,
so the 13th registration failed and **`httpd_start()` never ran at all**:

```
E http_radar: uri registration failed
E eth_netif: http server failed
```

The device booted, streamed IQ, decoded frames, and ran the feeder task
perfectly — but there was **no web UI and no API at all**, so from the outside it
looked like "the network is down". The feeder task had already connected and
was happily sending frames, which made the failure look even more like a network
problem than a firmware one.

**Lesson:** `max_uri_handlers` is a hard cap and exceeding it fails *at
registration*, taking down every route, not just the new one. Never size it at
today's route count.

**Rule:** set `max_uri_handlers` with headroom (16 for 13 routes here), and
treat `uri registration failed` as a route-count problem first. When the device
behaves normally over USB but the browser cannot connect, read the serial log
*before* blaming Ethernet or DHCP — the failure here was a one-line config
value, and the boot log named it outright.

## L33 — `getaddrinfo()` before lwIP is up = boot loop (feeder crash)

The BharatRadar feeder was started from `app_main()` **before**
`eth_netif_start()`. lwIP's `getaddrinfo()` posts a message to the tcpip thread
and asserts if that thread does not exist yet:

```
assert failed: tcpip_send_msg_wait_sem /lwip/src/api/tcpip.c:454 (Invalid mbox)
```

The chip panics, reboots, and on the next boot the feeder calls `getaddrinfo()`
again *before the network is up* -> panic again. **Unbreakable reboot loop**,
because the feeder config is persisted in NVS, so the path re-triggers on every
single boot. The unit looked dead: no web UI, no Ethernet, and it never stayed up
long enough to be rescued without a reflash.

**Why it survived testing:** the feeder was enabled at *runtime* over
`/api/feeder`, never at boot. Every test ran with `enabled=0` in NVS, so the
crash path was never exercised until a unit rebooted with the feeder already on.

**Fix:** wait for `IP_EVENT_ETH_GOT_IP` before resolving anything, and start the
feeder task after `eth_netif_start()`:

```c
if (!s_net_ready) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }  /* before connect */
...
esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ...);
```

**Rule:** anything touching lwIP (DNS, sockets) must be gated on the stack being
up. Treat boot-time behaviour of *persisted* config as a separate test from the
feature itself -- a change verified only at runtime has an entire untested class
of failures, all of which fire on the next power cycle.
