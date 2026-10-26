# ESP32-P4 ADS-B Radar

A self-contained ADS-B / Mode-S receiver and radar console built on the
**ESP32-P4**, decoding 1090 MHz transmissions from a cheap RTL-SDR USB dongle
and serving a live radar web UI over Ethernet.

It decodes aircraft locally (no internet needed for reception), plots them on a
browser radar, and can forward the raw frames to [BharatRadar](https://www.bharatradar.com)
so they show up on its public map.

```
   RTL-SDR dongle ──USB──▶ ESP32-P4 ──Ethernet──▶ browser radar UI
   (R820T2 tuner)            │                         (+ BharatRadar feed)
                            └── decodes 1090 MHz Mode-S: callsign, altitude,
                                speed, heading, vertical rate, lat/lon
```

---

## Table of contents

- [What it does](#what-it-does)
- [Hardware](#hardware)
- [Architecture](#architecture)
- [How the signal chain works](#how-the-signal-chain-works)
- [Building](#building)
- [Flashing](#flashing)
- [First-time setup](#first-time-setup)
- [HTTP API](#http-api)
- [BharatRadar feeder](#bharatradar-feeder)
- [Configuration](#configuration)
- [Repository layout](#repository-layout)
- [Testing](#testing)
- [Performance notes](#performance-notes)
- [Known issues](#known-issues)
- [Licensing and attribution](#licensing-and-attribution)
- [Project documents](#project-documents)

---

## What it does

- **Receives** 1090 MHz ADS-B Extended Squitter via any RTL2832U-class dongle
  (measured on a Nooelec NESDR Nano 2+ / R820T2).
- **Decodes** Mode-S DF11 and DF17 frames: ICAO address, callsign, altitude,
  ground speed, heading, vertical rate, and CPR-decoded latitude/longitude.
- **Tracks** aircraft, keeping position history so the UI can draw motion trails.
- **Serves** a single-page radar console over HTTP: range-scaled blips, trails,
  tooltips, callsign/flight-route lookup, and a live decode-stats readout.
- **Feeds** BharatRadar over the BEAST binary protocol (optional, off by default).
- **Configurable at runtime** from the web UI, persisted in NVS: gain, DC filter,
  gain mode, sample rate, receiver location, feeder settings.

Decoding is entirely on-device. The radar works with no internet connection;
only callsign route lookup and the optional BharatRadar feed need one.

---

## Hardware

| Item | Notes |
|---|---|
| **Board** | Waveshare **ESP32-P4-ETH** (silicon **rev v1.3**) |
| **SoC** | ESP32-P4, dual-core RISC-V @ 360 MHz, 32 MB PSRAM |
| **Tuner** | RTL2832U + R820T2 USB dongle (e.g. Nooelec NESDR Nano 2+) |
| **Network** | Onboard 100M Ethernet (P4 internal EMAC + IP101GRI RMII PHY) |
| **Antenna** | Any 1090 MHz antenna; a tuned one with reasonable gain matters a lot |

**Important hardware constraints**

- The board is silicon **rev v1.3**. Build with
  `ESP32P4_SELECTS_REV_LESS_V3=y` and CPU frequency **360 MHz**.
- **No ESP32-C6 module is populated**, so WiFi is unavailable. Networking is
  **Ethernet only**.
- Antenna placement dominates performance. A window-mounted or rooftop antenna
  typically outperforms any amount of DSP tuning.

---

## Architecture

Two custom components plus the application layer:

```
┌─────────────────────────────────────────────────────────────────┐
│ ESP32-P4 (dual core, 360 MHz)                                   │
│                                                                 │
│  components/esp_rtl_sdr/   (AGPL-3.0, vendored)                 │
│    ESP-IDF USB Host client for RTL2832U dongles                │
│      • enumerates the device, sets up bulk IN endpoints        │
│      • drives the R820T2 tuner over software I2C                │
│      • delivers 8-bit IQ in 8 x 64 KiB URBs                     │
│                        │  u8 IQ @ 2.4 MSPS                      │
│                        ▼                                        │
│  components/adsb_decoder/   (our code)                          │
│    demodulation + Mode-S decode                                 │
│      • magnitude (Alpha Max Plus Beta Min, integer)             │
│      • 1 Hz DC high-pass                                        │
│      • noise-adaptive preamble detector                         │
│      • 5-phase correlation slicers                              │
│      • Mode-S CRC, CPR position decoding, callsign/alt/speed    │
│                        │  adsb_frame_info_t (raw + decoded)      │
│                        ▼                                        │
│  main/   (our code)                                              │
│    rtl_pipeline.c   IQ task, track state, settings/NVS          │
│    adsb_state.c     aircraft tracks, trails, CPR resolution     │
│    http_radar.c     radar UI + JSON API                         │
│    beast_feeder.c   BharatRadar BEAST feed (optional)           │
│    eth_netif.c      Ethernet + DHCP                             │
│                                                                 │
│  core 1: rtl_driver(5) · adsb_dsp(6)                            │
│  core 0: rtl_stat(4) · beast_feed(3) · httpd(5) · USB host(19/20)│
└─────────────────────────────────────────────────────────────────┘
```

### Tasks

| Task | Core | Prio | Role |
|---|---|---|---|
| `rtl_usb_lib` / `rtl_usb_cli` | 0 | 20 / 19 | USB host stack (event driven) |
| `rtl_iq_del` | 1 | 18 | IQ delivery |
| `adsb_dsp` | 1 | **6** | The decoder — runs flat out |
| `rtl_driver` | 1 | 5 | Tuner/driver state machine |
| `httpd` | 0 | 5 | Web server |
| `rtl_stat` | 0 | 4 | Status + adaptive gain |
| `beast_feed` | 0 | 3 | BharatRadar upload (optional) |

The split matters: `adsb_dsp` saturates core 1, so all housekeeping that services
it lives on **core 0**. Co-locating them caused a real starvation bug in
development (see `docs/LEARNING.md` L24).

---

## How the signal chain works

1. **USB capture.** The dongle streams 8-bit interleaved I/Q. The driver fills
   8 × 64 KiB transfer buffers and hands each block to the pipeline.
2. **Magnitude.** Each I/Q pair becomes a magnitude via **Alpha Max Plus Beta
   Min** scaled by 415. This replaces a per-sample `std::sqrt` — which is a
   microcoded software call on this CPU and was discarding ~76% of the stream.
   AMB is *continuous* in the input, so weak signals keep low-end resolution.
3. **DC removal.** A single-pole 1 Hz high-pass removes the tuner's DC offset.
4. **Preamble detection.** Pulses are thresholded against a noise-adaptive
   floor derived from quiet sample quantiles.
5. **Slicing.** Five phase-offset correlators sample the pulse edges; the
   strongest phase yields the bit decisions (Phase 0–4, as in readsb).
6. **Mode-S decode.** CRC-validated DF11/DF17 messages yield ICAO, callsign,
   altitude, velocity, and CPR-encoded position.
7. **Position resolution.** CPR local/global decoding positions aircraft over
   successive frames.

Sample rate is **2.4 MSPS**. This is not arbitrary: on this R820T2, 2.048 MSPS
was measured to produce **zero** CRC-valid frames at every gain and ppm setting,
while 2.4 MSPS decodes reliably. Any stored 2.048 value is migrated to 2.4 on
boot.

---

## Building

**Requirements**

- ESP-IDF **v5.5 or newer** (developed against `v6.1-dev`)
- A checkout of this repository
- `espressif/usb` managed component (fetched automatically; needs network on
  first build)

```sh
source ~/esp/esp-idf/export.sh
idf.py set-target esp32p4
idf.py build
```

First build resolves the `espressif/usb` component registry dependency; later
builds are offline.

---

## Flashing

**Find the serial port**

```sh
ls /dev/cu.usbmodem*
```

**Flash and watch the boot log**

```sh
idf.py -p /dev/cu.usbmodemXXXXXX flash monitor
```

Expected on a healthy boot:

```
adsb_dsp: decoder self_check=1
rtl_pipe: stream started
esp_rtl_sdr: stream start freq=1090000000 exact_rate=2400000
http_radar: server up: / /radar /data/aircraft.json ...
```

If a serial monitor is holding the port, release it first:

```sh
lsof /dev/cu.usbmodemXXXXXX      # find the PID
kill <PID>
```

---

## First-time setup

1. Connect the antenna and place it as high and as clear as possible.
2. Flash, then open `http://<device-ip>/` in a browser. The device IP is printed
   on the serial boot log (`eth_netif: Ethernet IP: ...`).
3. Open **Settings** and set:
   - **Gain** — start at `42.1` (tenths of dB) and tune from there. Higher gain
     trades frame rate for range.
   - **DC Filter / Aggressive / CRC Fix** — all ON is a good starting point.
   - **Sample Rate** — leave at 2400000.
4. Set the **receiver location** so range rings and distances are correct.
   The web UI can fill it from your browser's geolocation.
5. Check reception via **Settings → decode stats**, or `/api/debug`. Healthy
   reception shows a non-zero CRC rate and a sample swing well above 20.

A quick sanity check with no browser:

```sh
curl http://<device-ip>/api/debug
```

`dsp.busy_pct` shows how much of the 2.4 MSPS stream the decoder is actually
processing — a useful regression signal for DSP changes.

---

## HTTP API

| Method | Path | Purpose |
|---|---|---|
| GET | `/`, `/radar` | Radar web UI (single page, self-contained) |
| GET | `/data/aircraft.json` | All tracked aircraft (also drives the map) |
| GET | `/api/status` | Phase, gain, RF health, sample range, tracks |
| GET | `/api/debug` | Decoder stats, DSP load, pipe health, config |
| GET | `/api/frames` | Recent **raw** Mode-S frames (hex) — diagnostics |
| GET/POST | `/api/location` | Receiver lat/lon |
| GET/POST | `/api/settings` | Gain, filters, sample rate |
| POST | `/api/settings/reset` | Restore defaults (also clears feeder config) |
| GET/POST | `/api/feeder` | BharatRadar feeder config and live status |

---

## BharatRadar feeder

Optional. Streams decoded frames to a BharatRadar hub so aircraft appear on the
public map at `map.bharatradar.com`. **Disabled by default** — a fresh flash
never transmits unless you turn it on.

Configure under **Settings → BharatRadar Feeder**: enable, paste your UUID
(from your BharatRadar profile), optionally a name. The host and port default to
`feed.bharatradar.com:30004`.

**Protocol** (BEAST binary over TCP):

- Handshake `0x1A 0xE4 <ASCII UUID>` on connect. No pre-registration is needed;
  the hub registers the UUID on first connect.
- Frames: `0x1A <type> <6B 12 MHz timestamp> <1B RSSI> <payload>`, where type is
  `0x32` for 7-byte DF11 and `0x33` for 14-byte DF17.
- Every literal `0x1A` byte in the frame is doubled (stereo bit-stuffing).
- RSSI is sent as `0xFF` (unknown). Location is **not** transmitted over BEAST.
- Location and aircraft identity come from the Mode-S frames themselves.

**Implementation notes**

- The decode hook only *enqueues* into a 256-slot lock-free ring and drops on
  overflow, so the priority-6 decoder never blocks on the network.
- All socket work happens on a separate core-0, priority-3 task with
  exponential reconnect backoff.
- The encoder is differential-tested byte-identical against an independent
  Python reference implementation.

The feeder waits for an IP before resolving DNS — calling `getaddrinfo()` before
the lwIP stack is up asserts and reboots the chip (this bit us once; see
`docs/LEARNING.md` L33).

---

## Configuration

Settings persist in NVS under the `radar` namespace and survive reboots.

| Key | Type | Default | Notes |
|---|---|---|---|
| `gain_tenth_db` | i32 | 496 | 42.1 dB is a good operating point |
| `sample_rate_sps` | u32 | 2400000 | 2.048 is migrated to 2.4 on load |
| `dc_filter` | u8 | 0 | DC high-pass |
| `aggressive` | u8 | 0 | More permissive preamble detection |
| `crc_fix` | u8 | 0 | Correct marginal frames |
| `gain_mode` | u8 | 0 | 0 = manual, 1 = adaptive |
| `lat` / `lon` | i32 | — | degrees × 1e7 |
| `feed_en` | u8 | 0 | Feeder enabled (0) |
| `feed_uuid` | str | — | Feeder UUID |
| `feed_name` | str | — | Feeder name |
| `feed_host` / `feed_port` | str / u32 | `feed.bharatradar.com` / 30004 | Hub |

---

## Repository layout

```
main/                     app entry, pipeline, web server, feeder
  app_main.c              boot order
  rtl_pipeline.c/.h       IQ task, driver control, settings/NVS
  adsb_state.c/.h         aircraft tracks, trails, CPR resolution
  http_radar.c/.h         web UI + JSON API
  beast_feeder.c/.h       BharatRadar BEAST feed
  eth_netif.c/.h          Ethernet + DHCP
  radar_data/radar.html   the entire web UI (embedded in the binary)
components/
  adsb_decoder/           demodulator + Mode-S decoder (C++)
  esp_rtl_sdr/            RTL-SDR USB host driver (vendored, AGPL-3.0)
tests/host/               host-side unit tests
docs/                     design, task log, lessons learned
```

The web UI is a **single self-contained HTML file** embedded into the firmware
with `EMBED_TXTFILES`. There is no CDN, no build step, and no external asset —
the device serves the complete interface on its own.

---

## Testing

**Host tests** (no hardware needed):

```sh
cd tests/host && make && /tmp/test_adsb_state
```

**Static checks for web assets** — the UI ships inside the firmware image, so
broken JS costs a full flash cycle. Before flashing UI changes, verify the HTML
parses, every `<script>` block passes `node --check`, and there are no duplicate
top-level identifiers. `AGENTS.md` §5.2 documents the procedure.

---

## Performance notes

Measured on the target hardware (P4 @ 360 MHz, 2.4 MSPS, gain 42.1 dB):

| Metric | Value |
|---|---|
| Effective sample rate | 2,400,000 sps (nominal, no overruns) |
| USB overruns / drops | 0 |
| CRC-valid frames | ~10–45 /s (varies with traffic) |
| Concurrent tracks | ~8–19 |
| USB throughput | ~14.4 MB/s |

The decoder processes roughly 40–50% of the sample stream; the remainder is
buffered. A known gap is that IQ crosses multiple PSRAM rings, and a
zero-copy handoff attempt regressed throughput and was reverted. The 16 KiB IQ
block size reduced buffering latency from ~328 ms to ~82 ms but was reverted to
keep the stable configuration.

**Gain is objective-dependent**, not universally optimal: higher gain increases
range and the number of positioned aircraft while reducing frame rate. Measure
for your site.

---

## Known issues

- **Sample-range collapse.** After long streaming sessions, sample min/max can
  narrow toward the 8-bit midpoint and decoding drops to zero; a reboot
  restores it. Under investigation — not yet root-caused.
- **No stall watchdog.** If the USB bulk path wedges, `eff` can read 0 with no
  automatic recovery.
- **No MLAT.** Aircraft are not triangulated from other receivers.
- Flash runs DIO at 40 MHz. QIO/80 MHz has not been attempted.
- Only DF11 and DF17 are decoded; DF18 (extended squitter, position) is
  filtered out.
- `crc_fix` may correct bits in marginal frames, so the reported `signal`
  value can read zero on a frame that decodes successfully.

---

## Licensing and attribution

This project combines **original code** with **third-party code under
different licenses**. Read this before redistributing.

| Component | Origin | License |
|---|---|---|
| `main/` | Original — this project (app, pipeline, web UI, feeder) | ⚠️ undetermined — see below |
| `components/adsb_decoder/` | Original code. The **algorithm** is ported from **readsb** (`demod_2400.c`, `convert.c`, `mode_s.c`) | ⚠️ see note |
| `components/esp_rtl_sdr/` | Vendored from a separate upstream project by the same author. Its header states it is a **clean-room implementation, not a librtlsdr port** | **AGPL-3.0** |
| `managed_components/espressif__usb` | Espressif, fetched at build time | Apache-2.0 |
| ESP-IDF, lwIP, FreeRTOS | Espressif / third party | Apache-2.0 / BSD |

**⚠️ Note on `main/`.** There is **no `LICENSE` file at the repository root**,
so the application layer currently has no explicit licence grant. Combined with
the AGPL-3.0 component below it, the tree as a whole is best treated as
**copyleft-encumbered** until you choose a licence and add one.

**⚠️ Note on the decoder.** The demodulation algorithm is a port of readsb,
which is **GPL-3.0**. Porting an algorithm is not the same as copying source,
but if you intend to distribute this firmware as a **binary** or a **closed
product**, the GPL-3.0 origin of `adsb_decoder` is the item to resolve first.
That decision is yours to make and is a reason to consult the licence texts and,
if it matters commercially, a qualified lawyer.

**AGPL note.** `components/esp_rtl_sdr/` is AGPL-3.0, which is a strong
copyleft licence with an explicit network-interaction clause (§13). AGPL
generally obliges you to offer corresponding source to users who interact with
the software over a network — which, for this project, includes users of the web
UI. That obligation is likely to apply to the combined firmware as a whole.

**Third-party references**

- **readsb** / **dump1090** (MIT/GPL family) — reference for the Mode-S and
  2.4 MSPS demodulation algorithms
- **librtlsdr** / **gr-osmosdr** — historical reference for RTL2832U device
  handling; *not* a source of code in this tree
- **ESP-IDF**, **espressif/usb** — Espressif Systems

---

## Project documents

| Document | Contents |
|---|---|
| [`AGENTS.md`](AGENTS.md) | Development workflow, verification gate, hardware facts |
| [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) | Architecture and component design |
| [`docs/TASKS_TRACKER.md`](docs/TASKS_TRACKER.md) | Phase/task history and platform notes |
| [`docs/LEARNING.md`](docs/LEARNING.md) | Hard-won lessons (L1…L33) — **start here** |
| [`docs/SESSION_SUMMARY.md`](docs/SESSION_SUMMARY.md) | Narrative of recent work |

`docs/LEARNING.md` records the non-obvious traps: task starvation, the 2.048 MSPS
dead end, a web-server route cap that silently killed the whole UI, and a lwIP
boot-order assert. Reading it first will save significant time.

## MLAT (BharatRadar)

The device now includes a native ESP-IDF MLAT client for [BharatRadar](https://bharatradar.com).

- **Server**: `feed.bharatradar.com:31090` (TCP, newline-delimited JSON)
- **Handshake**: Top-level JSON `{"version":3,"user":"<UUID>","uuid":"<UUID>","client_version":"esp32p4-mlat/1.0","lat":...,"lon":...,"alt":...,"compress":["none"],"clock_type":"radarcape_12mhz"}`
- **Timestamps**: Uses preamble sample offsets from the decoder for accurate reception timestamps (converted to 12 MHz units). Falls back to decode-time if unavailable.
- **Configuration**: HTTP API only (no USB needed after flashing). Persists in NVS.
- **Default**: Disabled. Auto-starts on boot if enabled.

### API

```bash
# Get MLAT status
curl http://<device-ip>/api/mlat

# Configure and enable MLAT (use same UUID as BharatRadar feeder)
curl -X POST http://<device-ip>/api/mlat \
  -H "Content-Type: application/json" \
  -d '{"enabled":true,"host":"feed.bharatradar.com","port":31090,"uuid":"<YOUR-UUID>","station_name":"pune-radar","lat":18.480856,"lon":73.898264,"alt_m":625.0,"privacy":false,"return_results":false}'
```

### Verification
- `/api/mlat` shows `connected`, `frames_submitted`, `frames_ts_preamble`, `frames_ts_fallback`
- `/api/frames` includes `t_sample_us`, `sample_idx`, `preamble_off` for debugging
- Check sync on BharatRadar: `https://mlat.bharatradar.com/synctable/feeder.html?0A&<UUID>`

### Notes
- Use the same feeder UUID as configured in `/api/feeder` (already feeding ADS-B to BharatRadar)
- USB only needed for firmware flashing; all config is HTTP-based
- After flashing, disconnect USB and move to terrace with antenna - verify over HTTP only
