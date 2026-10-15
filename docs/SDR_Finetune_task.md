# SDR Fine-Tuning Task — Reference Study

**Status:** Research complete, no code changes made.
**Created:** 2026-10-14
**Purpose:** Full teardown of the `jstockdale/T-Display-P4` ADS-B receiver (branch `adsb`)
as a reference for our memory / core / DSP work. Records what they do differently,
why it works for them, where their approach is *worse* than ours, and what (if
anything) is worth adopting.

> **This document is a reference, not a plan.** Nothing here has been implemented.
> Every "adopt" item is marked *not started*.

---

## 1. Reference sources

Primary repository:

- **Repo/branch:** <https://github.com/jstockdale/T-Display-P4/tree/adsb>

Files read in full (raw, branch `adsb`):

| File | Purpose | Permalink path |
|---|---|---|
| `main.cpp` | Boot order, task creation, memory strategy | `main/examples/adsb_receiver/main.cpp` |
| `class_driver.c` | `adsb_reader_task`, decode loop, adaptive gain, CPR | `main/examples/adsb_receiver/class_driver.c` |
| `esp_libusb.c` | USB transfer allocation, DMA reservation, bulk/control | `main/examples/adsb_receiver/esp_libusb.c` |
| `librtlsdr.c` | RTL2832/R82xx register + stream implementation | `main/examples/adsb_receiver/librtlsdr.c` |
| `mode-s.c` / `mode-s.h` | Mode-S demodulator + decoder | `main/examples/adsb_receiver/mode-s.c` |
| `rtl-sdr.h` | librtlsdr API surface | `main/examples/adsb_receiver/rtl-sdr.h` |
| `sdkconfig.defaults` | Build-time memory/peripheral configuration | `sdkconfig.defaults` |
| `partitions.csv` | Flash layout | `partitions.csv` |
| `CMakeLists.txt` | Build wiring | `main/examples/adsb_receiver/CMakeLists.txt` |

**Licence note.** `librtlsdr.c`, `rtl-sdr.h` and `mode-s.c` carry the upstream
**Steve Markgraf / librtlsdr GPL** header ("Copyright (C) 2012-2014 Steve Markgraf").
This project is therefore **GPL-encumbered**, not clean-room like ours. Relevant to
the licensing analysis already recorded in `README.md` (L34) — any technique we
*copy* from here inherits GPL; any technique we *reimplement from the published
behaviour* does not.

Our counterparts for side-by-side comparison:

- `components/esp_rtl_sdr/` — clean-room multi-URB USB host driver (AGPL-3.0)
- `components/adsb_decoder/` — readsb-derived demodulator
- `main/rtl_pipeline.c` — pipeline, tasks, settings

---

## 2. Headline comparison

| | T-Display-P4 (`adsb`) | Ours |
|---|---|---|
| USB driver | **librtlsdr port** (upstream, GPL) | clean-room multi-URB, async |
| Async stream engine | **disabled — commented out** | active, 8 × 64 KiB URBs |
| Read model | **blocking**, single in-flight transfer | async submit/resubmit |
| Sample rate | **2,000,000 (2.0 MSPS)** | 2,400,000 (2.4 MSPS) |
| Decoder | dump1090/multag `mode_s.c` | readsb `demodulate2400` |
| Magnitude | **129×129 precomputed LUT** × 360 | integer AMB × 415 |
| Bit slicing | 2 samples/bit, direct | **5 phase correlators**/bit |
| Ring buffer between USB→decode | **none** | multi-stage SPSC rings |
| IQ block size | 256 KiB (`DEFAULT_BUF_LENGTH`) | 8 × 64 KiB URBs |
| USB transfer buffer | 1 × 17,408 B, never freed | 8 × 64 KiB pool |
| Decode task | `adsb_reader` prio **3**, unpinned, 8 KB | `adsb_dsp` prio **6**, **core 1**, 8 KB |
| USB host task | `usb_host` prio **4**, **core 0** | prio 20/19, core 0 |
| LVGL task | prio **1** (lowest), PSRAM stack | n/a (no LVGL) |
| App task stacks | **all PSRAM** via `xTaskCreateWithCaps` | mostly internal RAM |
| Flash | 120 MHz, 16 MB | 40 MHz DIO |
| App partition | 15 MB single | 1.5 MB factory |
| Drop/overrun counters | **none at all** | overrun/drops on `/api/debug` |

---

## 3. Task & core topology

### Their tasks (exhaustive, from `main.cpp` + `class_driver.c`)

| Task | Stack | Prio | Core | Stack RAM | Created by |
|---|---|---|---|---|---|
| `usb_host` | 8192 | 4 | **pinned core 0** | internal | `rtlsdr_adsb_start()` |
| `class` (`class_driver_task`) | 16 KiB | 4 | unpinned | **PSRAM** | `rtlsdr_adsb_start()` |
| `usb_quit` | 2048 | 2 | unpinned | PSRAM | `rtlsdr_adsb_start()` |
| **`adsb_reader`** | 8192 | **3** | unpinned | internal | `client_event_cb` on NEW_DEV |
| `adsb_app_task` | 8 KiB | 2 | unpinned | PSRAM | `app_main` |
| `lvgl_ui_task` | 64 KiB | **1** | unpinned | PSRAM | `app_main` |
| `scope_app_task` | 16 KiB | 2 | unpinned | PSRAM | `app_main` |
| `meshy_app_task` | 8 KiB | 2 | unpinned | PSRAM | `app_main` |
| `gps_task` | 12 KiB | 3 | unpinned | PSRAM | `app_main` |
| `iis_tx_task` | 4 KiB | 6 | unpinned | PSRAM | `app_main` |
| ~8 more I2C/audio/UI tasks | 4–32 KiB | 2–3 | unpinned | PSRAM | `app_main` |

**Only two tasks in the entire project are core-pinned**, both to core 0.
There is no core-1 reference anywhere in `main.cpp`.

The `adsb_reader` task is created **from inside the USB client event callback**,
not from `app_main`:

```c
case USB_HOST_CLIENT_EVENT_NEW_DEV:
    // Launch the reader task — don't block in the callback.
    s_reader_owns_events = true;                 // set BEFORE creation
    xTaskCreate(adsb_reader_task, "adsb_reader", ADSB_READER_TASK_STACK,
                (void *)driver_obj, ADSB_READER_TASK_PRIO,
                &s_adsb_reader_task_hdl);
```

### Our tasks (for contrast)

| Task | Prio | Core |
|---|---|---|
| `rtl_usb_lib` / `rtl_usb_cli` | 20 / 19 | 0 |
| `rtl_iq_del` | 18 | 1 |
| **`adsb_dsp`** | **6** | **1** |
| `rtl_driver` | 5 | 1 |
| `httpd` | 5 | 0 |
| `rtl_stat` | 4 | 0 |
| `beast_feed` | 3 | 0 |

Our design deliberately saturates core 1 with the decoder and keeps all
housekeeping on core 0 (LEARNING.md L24 — a starving `rtl_stat` was a real bug).
Their design never dedicates a core; priorities are kept shallow (max 4 in the
ADS-B path) and nothing pins to core 1.

**Observation:** their approach works because their decode task is cheap enough
not to need isolation. Ours is 5× more expensive per sample, so it does.

---

## 4. Memory strategy

This is where they are genuinely more disciplined than us.

### 4.1 The DMA reservation pattern (the headline trick)

In `main.cpp`, **the very first** substantial allocation:

```c
// Reserve a contiguous block of DMA-capable internal RAM NOW, before
// peripheral init fragments the heap.  This block will be freed in
// init_adsb_dev() right before usb_host_transfer_alloc() needs it,
// guaranteeing a contiguous hole for the 17KB USB bulk transfer buffer.
g_usb_dma_reservation = heap_caps_malloc(20480, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
```

Then in `esp_libusb.c` `init_adsb_dev()`:

```c
if (g_usb_dma_reservation != NULL) {
    heap_caps_free(g_usb_dma_reservation);
    g_usb_dma_reservation = NULL;
}
// immediately realloc the USB bulk transfer into that exact hole
r = usb_host_transfer_alloc(RTLSDR_BUF_LEN + 512, 0, &adsbdev->transfer);
```

And the transfer buffer is **deliberately never freed**:

```c
void free_adsb_transfer(void) {
    // Intentionally empty — buffer stays allocated for reuse on reconnect
}
```

Their reasoning: a 17 KB contiguous DMA block may be unobtainable later once the
heap fragments. They also start the **USB host before WiFi** specifically so the
USB DMA allocation happens while the heap is still clean.

### 4.2 Pre-allocated control transfer buffers

```c
// Pre-allocate control transfer buffer once — eliminates the rapid
// alloc/free cycle during R820T tuner init that fragments internal
// heap and corrupts TLSF metadata.
usb_host_transfer_alloc(CTRL_TRANSFER_MAX_SIZE /*1024*/, 0, &adsbdev->ctrl_transfer);
adsbdev->response_buf = calloc(CTRL_TRANSFER_MAX_SIZE, sizeof(uint8_t));
```

### 4.3 PSRAM task stacks

All application task stacks go to PSRAM via a Zephyr-style helper:

```c
xTaskCreateWithCaps(device_adsb_app_task, "adsb_app_task", 8*1024, NULL, 2,
                    &Adsb_App_Task_Handle, MALLOC_CAP_SPIRAM);
```

Their stated budget: *"Internal RAM budget: ~100KB available at task creation.
With all app stacks in PSRAM, ~70KB+ remains free."*

Only `usb_host_lib_task` stays in internal RAM (ESP-IDF USB host requirement).

### 4.4 Boot-time heap instrumentation

22 `mem_checkpoint()` call sites logging free / **largest block** / min-ever-free
at each boot stage:

```c
static void mem_checkpoint(const char *label) {
    size_t free_int   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t large_int  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    size_t min_int    = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    ...
    // "The 'largest block' is critical — even if total free > 17KB,
    //  if it's fragmented into small pieces, DMA allocations fail."
}
```

Plus `heap_caps_check_integrity_all(true)` in the decode hot path:

```c
ESP_LOGI(TAG, "First read OK, stack HWM: %u free, heap: %s",
         uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
         heap_caps_check_integrity_all(true) ? "OK" : "CORRUPT");
```

### 4.5 `sdkconfig.defaults` memory settings

| Setting | Value | Effect |
|---|---|---|
| `SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY` | `y` | `.bss` → PSRAM (~18 KB saved) |
| `SPIRAM_MALLOC_ALWAYSINTERNAL` | `0` | PSRAM preferred for malloc |
| `SPIRAM_MALLOC_RESERVE_INTERNAL` | `32768` | 32 KB internal DMA reserve |
| `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` | `y` | lwIP buffers → PSRAM |
| `CACHE_L2_CACHE_256KB` | `y` | 256 KB L2 (ours: unspecified) |
| `ETH_DMA_RX/TX_BUFFER_NUM` | `4` / `4` | down from 20/10, saves ~11 KB |
| `ESPTOOLPY_FLASHFREQ_120M` | `y` | **120 MHz flash** (ours: 40 MHz DIO) |
| `ESPTOOLPY_FLASHSIZE_16MB` | `y` | 16 MB (ours: 4 MB) |

`partitions.csv` is a single 15 MB `factory` app partition — no OTA, no separate
data. Flash speed and L2 cache are the two settings where they likely have a
**real performance advantage** over us that has nothing to do with architecture.

---

## 5. The data path (and why they have no ring-buffer problems)

The entire chain, from `class_driver.c`:

```c
for (int i = 0; i < DEFAULT_BUF_LENGTH; i += MAX_PACKET_SIZE) {
    r = rtlsdr_read_sync(rtldev, tbuffer, MAX_PACKET_SIZE, &n_read);   // BLOCKING
    if (r < 0)              { read_ok = 0; break; }
    if (n_read < MAX_PACKET_SIZE) { read_ok = 0; break; }
    memcpy(&buffer[i], tbuffer, MAX_PACKET_SIZE);
}
if (read_ok && n_read > 0)
    demodulate(buffer, DEFAULT_BUF_LENGTH);                            // INLINE
```

```c
void demodulate(uint8_t *source, int length) {
    mode_s_compute_magnitude_vector(source, s_mag_buf, length);
    mode_s_detect(&state, s_mag_buf, length / 2, on_msg);
}
```

Three buffers, all allocated once, all PSRAM, all reused forever:

| Buffer | Size | Allocation |
|---|---|---|
| `buffer` | `DEFAULT_BUF_LENGTH` = 256 KiB | `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` |
| `tbuffer` | `MAX_PACKET_SIZE` | `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` |
| `s_mag_buf` | `(256KiB/2) × uint16_t` = 256 KiB | `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` |

**There is no ring buffer, no queue, no semaphore between USB and decoder.**

Because the USB read is **blocking**, back-pressure is automatic: if
`demodulate()` is slow, the next `rtlsdr_read_sync()` simply hasn't been issued
yet. There is no buffer to overflow, so there is no "ring full → drop → overrun"
failure mode. **This is the single biggest architectural difference between us
and them, and it is the direct cause of most of our pipeline symptoms.**

### 5.1 They disabled librtlsdr's async engine

`librtlsdr.c` retains upstream librtlsdr in full — but the entire async path is
**commented out** (~294 commented lines). Only this survives:

```c
int rtlsdr_read_sync(rtlsdr_dev_t *dev, void *buf, int len, int *n_read)
{
    return esp_libusb_bulk_transfer(dev->driver_obj, 0x81, buf, len, n_read, BULK_TIMEOUT);
}
```

with `#define DEFAULT_BUF_NUMBER 15` and `#define DEFAULT_BUF_LENGTH (16 * 32 * 512)`
left defined but unused by the async path. `rtlsdr_read_async`,
`rtlsdr_wait_async`, `_rtlsdr_alloc_async_buffers` and the `xfer_errors` /
`dev_lost` overflow accounting are all dead code.

**This explains their missing drop counters.** Upstream librtlsdr's async
callback is where `xfer_errors` and `dev_lost` were tracked. By deleting it they
also deleted the only place losses were ever visible.

---

## 6. Their decoder

`mode_s.c` is the classic dump1090 / Multilateration Mode-S demodulator, written
for **exactly 2 MHz**.

### Magnitude — lookup table, not arithmetic

```c
static uint16_t maglut[129 * 129 * 2];
// built once in mode_s_init():
for (i = 0; i <= 128; i++)
  for (q = 0; q <= 128; q++)
    maglut[i * 129 + q] = round(sqrt(i * i + q * q) * 360);

// per sample:
int i = data[j] - 127, q = data[j+1] - 127;
if (i < 0) i = -i;  if (q < 0) q = -q;
mag[j / 2] = maglut[i * 129 + q];
```

One subtraction, two abs, one multiply-add, one array load per sample. No
floating point at run time, no sqrt.

> **Scale mismatch warning:** their magnitude is scaled **× 360**, ours is
> **× 415**. Their preamble threshold `delta < 10 * 255` is tuned to ×360.
> These constants are **not** interchangeable between the two decoders.

### Detection — cheap fixed-offset comparisons

```c
if (!(mag[j]   > mag[j+1] && mag[j+1] < mag[j+2] && mag[j+2] > mag[j+3] &&
      mag[j+3] < mag[j]   && mag[j+4] < mag[j] && mag[j+5] < mag[j] &&
      mag[j+6] < mag[j]   && mag[j+7] > mag[j+8] && mag[j+8] < mag[j+9] &&
      mag[j+9] > mag[j+6])) continue;      // 10 magnitude comparisons, bail fast
```

Bits are then sliced directly off the magnitude vector, 2 samples/bit, with a
magic `delta < 256` no-change heuristic.

**Cost per bit: ours runs 5 phase-offset correlators; theirs runs 2 magnitude
reads and a comparison.** Combined with 20% less data (2.0 vs 2.4 MSPS), this is
the bulk of the throughput difference — not clock speed, not compiler flags.

---

## 7. Sample rate — the most important finding

```c
// class_driver.c
r = rtlsdr_set_sample_rate(rtldev, 2000000);   // hardcoded, not configurable
r = rtlsdr_set_center_freq(rtldev, 1090000000); // also hardcoded
```

They run **2.0 MSPS** with a 2 MHz-native decoder and decode successfully.

### This reframes our "2.048 decodes nothing" finding

Our measurement (readsb on the Mac, same dongle, same antenna) was:

| Rate | readsb CRC |
|---|---|
| 2,048,000 | **0** |
| 2,400,000 | **780** |

The conclusion we drew — *"2.048 MSPS does not decode ADS-B on this R820T2"* —
is **decoder-specific, not a tuner limitation.** Both decoders assume an *exact*
rate and compute preamble/bit offsets as fixed integer multiples of the sample
period:

- `demodulate2400` hard-codes 2.4 MHz.
- `mode_s.c` hard-codes 2.0 MHz.

At 2.048 MHz (2.4% above 2.0), bit positions drift by ~2.7 samples across 112
bits, which destroys correlation. At exactly 2.0 MHz with `mode_s.c`, there is no
drift. Likewise `librtlsdr.c` documents:

```c
/* sample loss is to be expected for rates > 2400000 */
```

**So the real rule is: the decoder and the sample rate must match exactly.**
There is no single "correct" rate — there are two well-supported ones, each with
its own detector.

**Implication for us (research, not action):** a 2.0 MSPS path using a
`mode_s.c`-class detector would cut our per-second sample volume by ~17% against
a materially cheaper detector. That is potentially a large CPU win. It is *not*
a drop-in change: different detector, different magnitude scale (×360 vs ×415),
different preamble thresholds, and a full re-tune of every constant.

Any such evaluation **must** be validated offline on our real capture
(`/tmp/.../dec_test/c24_42.1.bin`, recorded at 2.4 MSPS) before touching hardware
— though note that capture is 2.4 MSPS and would need re-recording at 2.0 MSPS
to test a 2.0 path properly.

---

## 8. Their problems — they are not issue-free

This section matters as much as the rest. They traded our problems for different
ones, and several are worse.

### 8.1 The aircraft display is completely disabled

```c
void adsb_update_display(lv_obj_t *table) {
    // LVGL table cell updates disabled — lv_table_set_cell_value calls realloc
    // which crashes on heap metadata corrupted by USB DMA.  Table shows headers
    // only.  Aircraft data is logged to SD card via sd_log_aircraft().
    (void)table;
    return;
}
```

Their on-screen aircraft list does not work. It was turned off because USB DMA
corrupted heap metadata and `realloc` in LVGL crashed on it.

### 8.2 A hand-rolled mutex over the USB client handle

```c
// When true, class_driver_task must NOT call usb_host_client_handle_events —
// the reader task becomes the sole USB event consumer to avoid concurrent
// event pumping on the same client handle (which corrupts the heap).
static volatile bool s_reader_owns_events = false;
```

A bare `volatile bool` is doing the work of a mutex. There is no atomicity, no
ordering guarantee, and the flag cannot retroactively stop `class_driver_task`
already inside `usb_host_client_handle_events()`.

### 8.3 A mutex taken with `portMAX_DELAY` inside the decode hot path

```c
void on_msg(mode_s_t *self, struct mode_s_msg *mm) {
    ...
    xSemaphoreTake(aircraft_mutex, portMAX_DELAY);   // blocks the whole read loop
    aircraft_t *ac = aircraft_get(icao);
    ...
    xSemaphoreGive(aircraft_mutex);
}
```

The same mutex is held by the UI task while formatting lists. **If the UI is slow,
the decode loop stalls** — and because the read is blocking, that stalls the USB
pipeline too, invisibly. Our design defers all UI/state work off the decode path
precisely to avoid this.

### 8.4 No loss observability at all

Zero overrun, zero drop, zero stall counters. There is no way for them to know
whether they are losing samples. We have `overruns` and `drops` on `/api/debug`.

### 8.5 Unresolved races and a busy-wait teardown

`s_new_dev_addr`, `s_adsb_reader_stop` and `s_adsb_reader_task_hdl` are plain
`volatile` globals shared across the reader task, the event callback, and the
deregister path. Teardown is a 5-second busy poll:

```c
s_adsb_reader_stop = true;
for (int i = 0; i < 50 && s_adsb_reader_task_hdl != NULL; i++)
    vTaskDelay(pdMS_TO_TICKS(100));
```

### 8.6 A latent tick-unit inconsistency

```c
ulTaskNotifyTake(false, 1000);        // 1000 TICKS ≈ 1 s
// vs the restart path:
ulTaskNotifyTake(false, pdMS_TO_TICKS(5000));   // 5000 ms
```

---

## 9. Recommendation: adopt the memory ideas, keep the architecture

### Adopt

| # | Idea | Why | Risk |
|---|---|---|---|
| A1 | **20 KB DMA reservation** — reserve contiguous internal DMA RAM first, release it exactly when USB needs it, never free the USB transfer buffer | Directly addresses USB/DMA allocation failure after heap fragmentation. We currently allocate URBs wherever the heap happens to allow. | Low — additive, no path change |
| A2 | **PSRAM task stacks** for non-DMA tasks | Frees internal RAM. `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` already exists in our config but is unused by our tasks. | Low–med — stack access cost; needs care for DMA-capable stacks |
| A3 | **`heap_caps_check_integrity_all()` + `largest_free_block` checkpoints at boot** | We have no equivalent. Would catch heap corruption before it becomes a mystery wedge. | Very low — diagnostics only |
| A4 | **Consider 120 MHz flash** | They run 120 MHz; we run 40 MHz DIO. Likely a real boot/flash-speed win. | Med — needs silicon verification |

### Do not adopt

| # | Their approach | Why not |
|---|---|---|
| R1 | Blocking reads, no ring buffer | Removes our overflow visibility and lets any decode stall silently wedge the USB path. Our async design reports `overruns`/`drops`; theirs reports nothing. |
| R2 | `mode_s.c` magnitude LUT ×360 | Our AMB ×415 is continuous in input amplitude and measured **better at every signal level** (L31). Switching to a LUT at ×360 would regress weak-signal decode — we tested exactly this failure mode with the earlier integer-sqrt table. |
| R3 | Mutex on the decode path | Directly contradicts the lesson in L24. |
| R4 | `volatile bool` as a mutex | Racy by construction. |

### Research

| # | Item |
|---|---|
| R5 | **2.0 MSPS decoder path** — evaluate a `mode_s.c`-class detector to cut sample volume ~17%. Offline validation on a freshly recorded 2.0 MSPS capture required. Not a drop-in: different scale (×360) and thresholds. |

---

## 10. Open gaps in this research

Recorded honestly — this study is thorough but **not** exhaustive:

1. **Runtime data.** We have no throughput, crash-rate, or measured CRC numbers
   from their hardware. Every performance comparison here is *structural*
   (instruction counts, buffer sizes, sample rates), not measured.
2. **`tuner_r82xx.c` not read.** Their R82xx register/gain implementation may
   contain tuner-specific handling relevant to our unresolved warm-up collapse
   (`docs/LEARNING.md` L35). **This is the highest-value unread file** for our
   open problems.
3. **`MAX_PACKET_SIZE` not resolved.** Defined outside the files read; it sets
   how many blocking reads fill the 256 KiB `buffer`. Affects latency estimates.
4. **`class_driver_task` creation not read** — lives in the `usb_host.c`
   submodule (`cpp_bus_driver_driver`), not in `main/examples/adsb_receiver/`.
5. **`xTaskCreateWithCaps` definition not read** — same submodule. Whether its
   stack argument is bytes or words is therefore unconfirmed.
6. **`mqtt_feeder.c`, `sd_logger.c`, `meshtastic_task.cpp` not read** — their
   network/logging path, potentially relevant to our feeder memory profile.
7. **`mode_s.c` is 2 MHz-native only**; we have not verified whether their build
   tolerates any ppm offset, which would matter for our crystal-error handling.

---

## 11. Follow-ups

All **not started**.

| ID | Task | Priority | Depends on |
|---|---|---|---|
| F1 | Read `tuner_r82xx.c` + `rtlsdr_i2c.h`; compare R82xx init/gain sequence against ours for warm-up-collapse clues | **High** | none |
| F2 | Implement A1 (DMA reservation) behind a config flag | Medium | — |
| F3 | Add A3 (heap integrity checkpoints) to our boot log | Medium | — |
| F4 | Evaluate A2 (PSRAM stacks) task by task, starting with `rtl_stat` | Low | F2 |
| F5 | Investigate A4 (120 MHz flash) — verify rev v1.3 tolerates it | Medium | — |
| F6 | Re-record a 2.0 MSPS capture for an R5 feasibility test | Low | — |

---

## 12. See also

- `docs/LEARNING.md` — L24 (core starvation), L31 (AMB vs integer-sqrt),
  L34 (licensing), L35 (warm-up collapse, unresolved)
- `docs/TASKS_TRACKER.md` — phase log
- `README.md` — architecture overview and licensing audit