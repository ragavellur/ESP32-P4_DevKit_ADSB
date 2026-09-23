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