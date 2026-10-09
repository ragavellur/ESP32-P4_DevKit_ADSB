# AGENTS.md — Development & SDLC Guide

This document defines the working agreement for any human or AI agent contributing to
**ESP32-P4_DevKit_ADSB** (ESP-IDF project `adsb_radar`). Follow it for every change.

Project: dual-core ESP32-P4 (Waveshare ESP32-P4-ETH, **no WiFi / Ethernet-only**) driving an
RTL-SDR (Nooelec NESDR Nano 2+, R820T2) to decode 1090 MHz ADS-B / Mode-S, with an on-device
web radar UI and runtime-configurable receiver settings.

---

## 1. Golden Rules

1. **No trial-and-error flashing.** Prove a change builds and its static assets are valid
   *before* it touches the hardware. One deliberate flash beats five blind ones.
2. **One logical change per commit.** Small, reviewable, revertible.
3. **Every change is tagged.** Semver tag on `main` after a change is verified.
4. **Docs move with code.** A commit that changes behavior must update the docs below.
5. **Never commit secrets or generated artifacts.** See §7.

---

## 2. Repository Layout

```
main/                      app entry, pipeline, web server
components/
  adsb_decoder/            Mode-S / ADS-B demodulator + decoder
  esp_rtl_sdr/             RTL-SDR USB driver (R820T2)
docs/
  IMPLEMENTATION_PLAN.md   design / architecture
  TASKS_TRACKER.md         phase + task status, platform notes, open decisions
  SESSION_SUMMARY.md       running session narrative
  LEARNING.md              L1..Ln hard-won lessons (append-only)
AGENTS.md                  this file
```

---

## 3. Branching & Commit Model

- `main` is always the stable, hardware-verified branch.
- Work on short-lived branches:
  - `feat/<topic>` — new capability
  - `fix/<topic>` — bug fix
  - `docs/<topic>` — documentation only
  - `chore/<topic>` — build, tooling, deps
- Merge to `main` only after the §5 verification gate passes on hardware.
- **Commit messages — Conventional Commits:**
  ```
  <type>: <imperative summary>

  <optional body: what / why / measured effect>
  ```
  Types: `feat`, `fix`, `docs`, `chore`, `refactor`, `perf`, `test`.
  Example: `fix: Accept numeric gain_mode values (0/1) in API POST`

- **Releases:** tag `vMAJOR.MINOR.PATCH-<slug>` (e.g. `v0.2.2-gain-mode-fix`) after hardware
  verification. Tag format in this repo has historically been loose; prefer
  `vX.Y.Z-<slug>` going forward.

---

## 4. Change Workflow

1. `git checkout -b <type>/<topic>`
2. Implement the smallest change that satisfies the requirement.
3. Build locally (§6). Fix all warnings you introduced.
4. Run the **pre-flash verification gate** (§5).
5. Flash + test on hardware (§6). Record measured effect (frame counts, tracks, drops).
6. Update docs (§5.3) in the **same commit**.
7. Commit on the branch, merge to `main`, tag, push:
   ```
   git add ... && git commit -m "feat: ..."
   git checkout main && git merge --no-ff <branch>
   git tag vX.Y.Z-<slug> && git push origin main --tags
   ```

---

## 5. Verification Gate (run before every flash)

### 5.1 Build
```sh
source ~/esp/esp-idf/export.sh
idf.py build
```
A change is not "done" if the build fails or emits new warnings.

### 5.2 Static checks for web assets
The radar UI is embedded from `main/radar_data/radar.html` via `EMBED_TXTFILES`. Broken JS/HTML
has historically shipped and wasted flash cycles. Before flashing, always:
- **HTML:** parse with a real parser (no regex/regex-only "looks fine").
- **JS:** extract each `<script>` block and run `node --check` on it.
- **Duplicate identifiers:** grep for repeated `function` / `const` / `let` / `var` names —
  "Identifier 'x' has already been declared" is a classic regression here.
- Confirm every function referenced by inline `onclick=` / `addEventListener` is actually defined
  and, if defined inside an IIFE, is exposed on `window`.

### 5.3 Documentation (same commit)
Update, as applicable:
- `docs/LEARNING.md` — append a new `L<n>` entry for any non-obvious lesson (bug root cause,
  platform quirk, wasted-effort warning). **Append-only**, never renumber.
- `docs/TASKS_TRACKER.md` — task/phase status, open decisions, platform notes.
- `docs/SESSION_SUMMARY.md` — narrative of what changed and why.

---

## 6. Build, Flash & Monitor

```sh
# Environment (every new shell)
source ~/esp/esp-idf/export.sh

# Build
idf.py build

# Release the serial port if a monitor is holding it
lsof /dev/cu.usbmodem5B900947191      # find the serial-mon... PID
kill <PID>

# Flash + monitor
idf.py -p /dev/cu.usbmodem5B900947191 flash monitor
```

**Hardware facts (do not rediscover the hard way):**
- Board is silicon **rev v1.3** → must keep `ESP32P4_SELECTS_REV_LESS_V3=y` and CPU **360 MHz**.
- **No ESP32-C6 present** — WiFi is impossible; networking is Ethernet only.
- Device web UI: `http://<device-ip>/` (`/`, `/radar`, `/data/aircraft.json`, `/api/status`,
  `/api/settings`, `/api/settings/reset`). Device has been seen at `192.168.200.102`.
- Debugging metric of record: `crc_ok` frame counts, `tracks`, `effective_sps`, `overrun`/`drops`.

---

## 7. Secrets & Ignored Artifacts

**Never commit:**
- `build/`, `managed_components/`, `sdkconfig`, `sdkconfig.old`
- Logs, backups (`*.log`, `*.bak`), `.DS_Store`
- GitHub tokens, WiFi/AP credentials, SSH passwords, API keys

`.gitignore` already covers the generated/OS files. Secrets must never be written into the repo,
committed, or pasted into issue/PR text. Use environment variables or local-only scratch files.

---

## 8. Reference Documents

- `docs/IMPLEMENTATION_PLAN.md` — architecture and component design
- `docs/TASKS_TRACKER.md` — current phase/task status and platform notes
- `docs/LEARNING.md` — read this first; it will save you hours
