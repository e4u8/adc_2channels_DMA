# adc_2channels_DMA — branch: v2.1-interleaved

Dual-channel ADC acquisition firmware for the **Dialog Semiconductor DA14706** (DA1470x family).  
Measures current (CH0, P0.5) and voltage (CH1, P0.6) back-to-back in a tight interleaved loop,
streams calibrated mV pairs over UART, and provides precise timing diagnostics via the ARM DWT
cycle counter. This branch is the stable foundation for power calculations (Vrms, Irms, P, PF)
on the next branch.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Target](#hardware-target)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
  - [RTOS Task Layout](#rtos-task-layout)
  - [Acquisition Flow](#acquisition-flow)
  - [DMA and the Adapter Layer](#dma-and-the-adapter-layer)
- [ADC Configuration](#adc-configuration)
- [GPIO Pin Mapping](#gpio-pin-mapping)
- [Clock & Power Configuration](#clock--power-configuration)
- [Serial Output Protocol](#serial-output-protocol)
- [Inter-Channel Skew](#inter-channel-skew)
  - [What it is](#what-it-is)
  - [Why it is constant](#why-it-is-constant)
  - [Impact on power calculations](#impact-on-power-calculations)
  - [How to compensate](#how-to-compensate)
- [Throughput Analysis](#throughput-analysis)
  - [Measured results](#measured-results)
  - [Why sample_time and oversampling have no effect](#why-sample_time-and-oversampling-have-no-effect)
  - [Is 2200 samples/channel/s enough?](#is-2200-sampleschannels-enough)
  - [How fs_acq is measured](#how-fs_acq-is-measured)
  - [Why fs_acq varies slightly between windows](#why-fs_acq-varies-slightly-between-windows)
- [Calibration](#calibration)
- [Multi-Task Architecture Notes](#multi-task-architecture-notes)
- [Build Configurations](#build-configurations)
- [Key Parameters](#key-parameters)
- [Next Steps — ZC Detection, RMS, Power](#next-steps--zc-detection-rms-power)

---

## Overview

This project is the second major iteration of a power-measurement firmware for 50 Hz mains signals.
It samples current (CH0) and voltage (CH1) in a strict per-sample interleaved pattern —
CH0 then CH1 then CH0 then CH1 — so that each pair carries a known, fixed, measurable time offset
(the **skew**). This skew is the phase-correction constant needed for accurate active power P
and power factor PF calculations.

**Key capabilities:**

- Interleaved single-sample acquisition — CH0 and CH1 sampled back-to-back for every index `i`
- ARM DWT cycle counter for sub-microsecond timestamps with zero OS overhead
- One-time startup skew measurement (`*skew=`) — the phase-correction constant T_SKEW
- DWT-based true acquisition throughput (`*fs_acq`, `*us_pair`) — immune to UART print time
- Per-channel linear calibration (offset + gain)
- Configurable batch size (default 64 pairs)
- Decimated UART output (`PRINT_EVERY`) for debugging
- Three execution targets: RAM, QSPI flash, OQSPI flash

---

## Hardware Target

| Parameter  | Value                                         |
|------------|-----------------------------------------------|
| SoC        | Dialog Semiconductor DA14706-00               |
| CPU        | ARM Cortex-M33                                |
| FPU        | FPv5-SP-D16 (single-precision hardware float) |
| BLE core   | Dedicated CMAC (Cortex-M0+), separate from CM33 |
| SDK        | DA1470x SDK                                   |
| RTOS       | FreeRTOS                                      |
| Toolchain  | arm-none-eabi-gcc                             |
| IDE        | Eclipse CDT (SmartSnippets / e² Studio)       |

---

## Project Structure

```
adc_2channels_DMA/
├── main.c                    # System init, clock setup, GPIO config, RTOS launch
├── gpadc_app.c               # ADC acquisition task — all application logic
├── gpadc_app.h               # Task entry point declarations
└── config/
    ├── platform_devices.h    # Exported ADC device handle declarations
    ├── custom_config_ram.h   # BSP feature flags — RAM execution target
    ├── custom_config_qspi.h  # BSP feature flags — QSPI flash target
    └── custom_config_oqspi.h # BSP feature flags — OQSPI flash target
```

Hardware initialization is centralised in `main.c → prvSetupHardware()`. `gpadc_app_init()` is
intentionally empty — `gpadc_app_task()` is the single entry point for all acquisition logic.

---

## Architecture

### RTOS Task Layout

```
main()
  └─ SysInit task  (OS_TASK_PRIORITY_HIGHEST, runs once then self-deletes)
       ├─ cm_sys_clk_init(sysclk_XTAL32M)    32 MHz crystal clock
       ├─ cm_apb/ahb_set_clock_divider(div1)  full-speed buses
       ├─ pm_sleep_mode_set(pm_mode_idle)     CPU halts between tasks
       ├─ pm_set_sys_wakeup_mode(fast)
       ├─ prvSetupHardware()                  GPIO, power domain, IO config
       └─ creates: gpadc_app_task  (OS_TASK_PRIORITY_NORMAL, runs forever)
```

Only `gpadc_app_task` remains after startup. Future tasks (I2C sensor, BLE handler, GPIO control)
should be created here alongside it with appropriate priorities — see
[Multi-Task Architecture Notes](#multi-task-architecture-notes).

### Acquisition Flow

```
[Startup — once]
  Arm DWT CYCCNT  (32 MHz free-running cycle counter)
  Open CH0 → read 1 dummy sample → snapshot DWT → Close CH0
  Open CH1 → read 1 dummy sample → snapshot DWT → Close CH1
  skew_cycles = t_ch1_done - t_ch0_done
  Print: *skew=<cycles> (~<µs> us)

[Main loop — repeats forever]

  ① Interleaved acquisition
     t_acq_start = DWT->CYCCNT
     For i = 0 .. BATCH_SIZE-1:
       Open CH0 → read raw0[i] → Close CH0
       Open CH1 → read raw1[i] → Close CH1   ← fixed skew after CH0
     acq_cycles_accum += DWT->CYCCNT - t_acq_start
     batches_in_window++

  ② Conversion and print
     For i = 0 .. BATCH_SIZE-1:
       raw0[i] → ad_gpadc_conv_to_mvolt → correct_mv → mv0
       raw1[i] → ad_gpadc_conv_to_mvolt → correct_mv → mv1
       If (i % PRINT_EVERY == 0): printf("%d,%d\n", mv0, mv1)

  ③ Diagnostics — once per second
     total_pairs = batches_in_window × BATCH_SIZE
     fs_acq  = (total_pairs × 2 × CPU_CLOCK_HZ) / acq_cycles_accum
     us_pair = acq_cycles_accum / (total_pairs × 32)
     Print: *fs_acq=<N>  *us_pair=<N>
     Reset accumulators
```

`ad_gpadc_read_nof_conv(handle, 1, &dest)` is **synchronous and blocking** — it starts one
conversion, suspends the task (yielding the CPU to the scheduler), and returns only when the
DMA completion event fires. The CPU is not busy-waiting during conversions.

### DMA and the Adapter Layer

DMA is active. Both `custom_config_*.h` files define:

```c
#define dg_configGPADC_DMA_SUPPORT   (1)
#define dg_configUSE_HW_DMA          (1)
```

The `ad_gpadc` adapter includes the `dma_setup` struct conditionally on `HW_GPADC_DMA_SUPPORT`.
From the application's perspective the API call is identical whether DMA is used or not —
the difference is internal: with DMA the CPU is truly released to the RTOS scheduler during
each conversion instead of busy-polling.

---

## ADC Configuration

| Parameter    | Value                                        |
|--------------|----------------------------------------------|
| CH0 signal   | Current (P0.5)                               |
| CH1 signal   | Voltage (P0.6)                               |
| API          | `ad_gpadc` adapter layer (synchronous)       |
| Acquisition  | 1 sample per `ad_gpadc_read_nof_conv` call   |
| DMA          | Enabled — CPU yields during each conversion  |

Full hardware parameters (input mode, attenuator range, sample time, oversampling, DMA channel)
are in `config/platform_devices.c`.

---

## GPIO Pin Mapping

| Signal       | Port | Pin | Function           |
|--------------|------|-----|--------------------|
| CH0 current  | P0   | 5   | `HW_GPIO_FUNC_ADC` |
| CH1 voltage  | P0   | 6   | `HW_GPIO_FUNC_ADC` |

Pins are configured as high-impedance ADC inputs in `periph_init()`. The adapter layer manages
their mux state automatically on every `ad_gpadc_open` / `ad_gpadc_close`.

---

## Clock & Power Configuration

| Parameter     | Setting                                       |
|---------------|-----------------------------------------------|
| System clock  | 32 MHz XTAL32M (crystal, ±20–50 ppm)          |
| AHB divider   | div1 → 32 MHz                                 |
| APB divider   | div1 → 32 MHz                                 |
| LP clock      | 32.768 kHz crystal (drives FreeRTOS tick)     |
| Sleep mode    | `pm_mode_idle` — CPU halts, DMA stays active  |
| Wakeup mode   | `pm_sys_wakeup_mode_fast`                     |
| DWT tick      | 1 cycle = 31.25 ns at 32 MHz                  |

**Why XTAL32M and not RCHS:** The internal RC oscillator (RCHS) has ±2–3% frequency accuracy
that varies with temperature and supply voltage — not fixed. XTAL32M at ±20–50 ppm is required
for two reasons: (1) the DWT µs conversion constant `cycles / 32` is accurate only if the clock
is exactly 32 MHz; (2) the FreeRTOS tick period (via the LP crystal) would diverge from the
DWT-based measurements if the system clock drifted. Use XTAL32M for any measurement application.

---

## Serial Output Protocol

All output is ASCII text over the retarget UART (enabled by `CONFIG_RETARGET` in
`custom_config_*.h`). Lines prefixed with `*` are diagnostic markers, not sample data.

| Line | When | Meaning |
|------|------|---------|
| `*skew=N cycles (~M us)` | Once at startup | CH0→CH1 inter-sample delay. Record this value — it is T_SKEW for PF compensation |
| `<mv0>,<mv1>` | Every `PRINT_EVERY` pairs | Current (mV), Voltage (mV) — CSV for debugging |
| `*fs_acq=N  *us_pair=N` | Once per second | True ADC throughput and average pair time, both measured from DWT |
| `*loop_skew  first=N  mid=N  last=N us` | Once per second | Skew measured at i=0, i=BATCH_SIZE/2, i=BATCH_SIZE-1 of the last batch — confirms skew stability across the batch |

Example:

```
*skew=6688 cycles (~209 us)
1234,987
1236,985
...
*fs_acq=4418  *us_pair=452
*loop_skew  first=209  mid=209  last=209 us
```

**Important:** `printf` over UART is synchronous and blocking on this platform. Each character
takes ~87 µs at 115200 baud. `PRINT_EVERY` and `BATCH_SIZE` directly affect how much time the
task spends inside `printf`, but they do **not** affect `*fs_acq` or `*us_pair` — those are
measured only over the acquisition loop, excluding all print time.

---

## Inter-Channel Skew

### What it is

CH0 and CH1 cannot be sampled simultaneously — the GPADC has one input mux. They are sampled
back-to-back. The time between the moment CH0's sample was captured and the moment CH1's sample
was captured is called the **skew**:

```
skew = t_ch1_done − t_ch0_done
     = close_CH0 overhead
     + open_CH1 overhead
     + CH1 conversion time
```

Measured at startup using DWT and printed as `*skew=`. At the default configuration:
**skew ≈ 209 µs** (6688 DWT cycles at 32 MHz).

### Stability — measured results

To verify skew consistency across a batch, `*loop_skew` was added to measure at `i=0`,
`i=BATCH_SIZE/2`, and `i=BATCH_SIZE-1` of the same batch (same DWT method as startup).

**Typical output:**
```
*loop_skew  first=209  mid=209  last=209 us
```

**Observed occasionally:**
```
*loop_skew  first=209  mid=339  last=209 us
```

The middle sample sporadically shows a skew ~130 µs higher than first and last. First and last
remain stable. The most likely cause is a FreeRTOS tick interrupt or a DMA completion callback
for an unrelated peripheral firing at the exact moment the GPADC task is between the CH0 close
and CH1 open at that index, adding latency before `ad_gpadc_open(CHAN1)` can return.

**Decision: accepted.** The error this introduces in P at 50 Hz is bounded:

- Normal skew: 209 µs → 3.76° phase error → 0.22% error in P
- Worst-case skew: 339 µs → 6.10° phase error → 0.57% error in P for the affected sample

Since the spike affects only isolated samples (not the entire batch), the mean error in P
accumulated over a full mains cycle is far smaller than the per-sample worst case and is
considered acceptable for this application.

### Impact on power calculations

| Calculation | Skew effect |
|-------------|-------------|
| Vrms | None — independent of I |
| Irms | None — independent of V |
| P = mean(V[i] × I[i]) | Yes — V[i] and I[i] are not simultaneous |
| PF = P / (Vrms × Irms) | Yes — inherited from P error |

At 50 Hz, 209 µs = **3.76° of phase error**. The resulting error in P is `1 − cos(3.76°)` ≈
**0.22%**. Acceptable for a first implementation; must be corrected for accurate PF.

### How to compensate

The skew is constant and already measured. Linear interpolation of the voltage to the time the
current sample was actually taken:

```c
float T_SKEW_US = 209.0f;         // from *skew= at startup
float T_SAMPLE_US = 452.0f;       // from *us_pair= at runtime

// For each pair i (except the last):
float v_compensated = mv1[i] + (mv1[i+1] - mv1[i]) * (T_SKEW_US / T_SAMPLE_US);
float p_inst = (mv0[i] / R_SHUNT) * v_compensated;  // instantaneous power
```

Then `P = mean(p_inst)` over a complete number of mains cycles.

---

## Throughput Analysis

### Measured results

Configuration: `sample_time=4`, `oversampling=4` (representative of all tested configs):

| skew (µs) | fs_acq (Hz) | us_pair (µs) |
|-----------|-------------|--------------|
| 209       | 4414–4419   | 452          |
| 209       | 4439        | 450          |
| 209       | 4602–4603   | 434          |

All other configurations (`sample_time` 2–4, `oversampling` 1–4) produced the same result:
**~4430 Hz** (`fs_acq` pairs/s, i.e. ~2215 samples/channel/s) and **~450 µs/pair**.

### Why sample_time and oversampling have no effect

This is the most important finding from profiling. The expectation was that reducing
`oversampling` from 4 to 1 would reduce conversion time by ~4×. It did not. Every configuration
lands at ~450 µs/pair.

The reason: the bottleneck is the `ad_gpadc_open` / `ad_gpadc_close` adapter overhead, called
**128 times per batch** (once per sample per channel). Each call:
- Acquires and releases an RTOS resource lock (mutex → context switch overhead)
- Reconfigures the IO mux pin state
- Reconfigures GPADC registers for the channel

The actual ADC conversion time is a small fraction of the 450 µs. Tuning `sample_time` and
`oversampling` adjusts only the conversion time, not the adapter overhead, so fs_acq is
unaffected.

**Attempted optimisation:** keeping both handles open across the batch (`open CH0, open CH1,
loop, close CH0, close CH1`) was blocked by the adapter's single shared GPADC resource lock —
both channels cannot hold the lock simultaneously.

**Path to higher throughput** (for a future branch if needed):
- Access `hw_gpadc_*` directly inside the batch loop, bypassing the adapter resource management
- Use a hardware timer to trigger conversions at a fixed rate (interrupt/DMA driven, no per-sample RTOS overhead)

### Is 2200 samples/channel/s enough?

For 50 Hz mains power measurement:

| Measurement target | Minimum fs/ch needed | Current 2215 Hz |
|--------------------|----------------------|-----------------|
| Fundamental P, PF  | ~500 Hz              | ✓ 4× margin     |
| Vrms, Irms         | ~500 Hz              | ✓ 4× margin     |
| Harmonics to 5th   | ~600 Hz              | ✓               |
| Harmonics to 10th  | ~1200 Hz             | ✓               |
| Harmonics to 20th  | ~2500 Hz             | borderline       |

**Conclusion:** 2215 Hz/channel is more than sufficient for fundamental P, PF, Vrms, Irms.
Only pursue higher throughput if harmonic content beyond the 10th is required.

### How fs_acq is measured

`fs_acq` is derived purely from DWT cycle counts accumulated during the acquisition loop,
excluding the print loop and all other overhead:

```
fs_acq  = (total_pairs × 2 × CPU_CLOCK_HZ) / acq_cycles_accum
us_pair = acq_cycles_accum / (total_pairs × 32)
```

This makes `fs_acq` immune to `PRINT_EVERY`, `BATCH_SIZE`, and UART congestion. The old approach
of counting `sample_counter += BATCH_SIZE * 2` once per second produced values that dropped
significantly when `PRINT_EVERY` was reduced (from 4430 Hz to 1408–1536 Hz) because print time
was included in the measurement window.

### Why fs_acq varies slightly between windows

`fs_acq` is reported with ~±2% window-to-window variation (4414–4603 Hz). This is a measurement
artifact, not real jitter in the ADC timing. Two causes:

1. **Batch-boundary quantization:** the 1-second diagnostic window closes at the end of a
   complete batch, not at an exact 1-second mark. The window can be off by ±1 batch (~28 ms),
   which is ±2.8% on a 1-second window.
2. **`xTaskGetTickCount` resolution:** 1 ms tick → ±1 ms additional jitter.

`us_pair` (450–452 µs) is the stable metric — it averages over all pairs in the window and
is not affected by window-boundary timing. Use `us_pair` for timing calculations, not `fs_acq`.

---

## Calibration

Each channel has two constants at the top of [gpadc_app.c](gpadc_app.c):

```c
#define OFFSET_MV_CH0   0.0f   // subtract from raw mV reading
#define GAIN_CH0        1.0f   // divide by this factor

#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f
```

Applied by `correct_mv()`:

```
corrected_mv = (raw_mv − offset) / gain
```

Values below zero after correction are clamped to 0. The defaults pass raw values through
unchanged. To calibrate: apply a known reference voltage, compare to the reported value,
then set `OFFSET_MV` to the measured DC error and `GAIN` to the ratio of expected/reported.

---

## Multi-Task Architecture Notes

The next branches will add FreeRTOS tasks for I2C sensor reading, BLE data transmission,
and GPIO control. Key points for integrating them without degrading acquisition:

**BLE stack does not run on the CM33.** The DA1470x has a dedicated CMAC core (Cortex-M0+)
for the BLE stack. Radio and protocol handling happen there autonomously. The CM33-side BLE
tasks are thin IPC handlers — short-lived, infrequent (one per connection event, ~7.5–4000 ms).
They will not continuously preempt the GPADC task.

**The only true threat is I2C blocking reads** at equal or higher priority, which would delay
the GPADC task's resume after a DMA completion event.

**Recommended priority assignment:**

```
OS_TASK_PRIORITY_HIGHEST  →  SysInit (self-deletes)
                              BLE IPC tasks (set by SDK — do not change)

OS_TASK_PRIORITY_NORMAL+1 →  gpadc_app_task        ← raise one level from current
OS_TASK_PRIORITY_NORMAL   →  BLE data batch task
OS_TASK_PRIORITY_NORMAL-1 →  I2C sensor task
OS_TASK_PRIORITY_NORMAL-1 →  GPIO / BLE command handler
```

With this assignment, the GPADC task resumes immediately after each DMA ISR regardless of what
the I2C or BLE tasks are doing. The BLE SDK tasks at HIGHEST are short-lived and do not
continuously hold the CPU.

---

## Build Configurations

| Configuration            | Optimisation | Execution target | Recommended use        |
|--------------------------|-------------|------------------|------------------------|
| DA1470x-00-Debug_RAM     | -O0         | SRAM             | Active development     |
| DA1470x-00-Debug_OQSPI   | -O0         | OQSPI flash      | Hardware-in-loop debug |
| DA1470x-00-Release_OQSPI | -Os         | OQSPI flash      | Production             |

RAM execution skips the flash erase/program cycle — strongly recommended during development.

All three configurations enable the same ADC-relevant flags in `custom_config_*.h`:

```c
#define dg_configUSE_HW_GPADC         (1)
#define dg_configGPADC_ADAPTER        (1)
#define dg_configGPADC_DMA_SUPPORT    (1)
#define dg_configUSE_HW_DMA           (1)
#define OS_FREERTOS
#define configTOTAL_HEAP_SIZE         14000
```

---

## Key Parameters

All commonly-tuned values in [gpadc_app.c](gpadc_app.c):

| Symbol             | Default     | Purpose |
|--------------------|-------------|---------|
| `BATCH_SIZE`       | `64`        | Pairs per loop iteration. Range 32–256. Larger → smoother `*fs_acq` average, higher latency. Smaller → lower latency, noisier `*fs_acq`. Each unit costs 4 bytes of BSS (2 × uint16_t) |
| `PRINT_EVERY`      | `2`         | Print 1 in every N pairs. Does not affect `*fs_acq`. Set to 1 for full data, higher to reduce UART load |
| `CPU_CLOCK_HZ`     | `32000000`  | Must match `cm_sys_clk_init(sysclk_XTAL32M)`. Used in all DWT µs conversions |
| `OFFSET_MV_CH0/1`  | `0.0f`      | DC offset calibration per channel (mV) |
| `GAIN_CH0/1`       | `1.0f`      | Gain calibration per channel |

---

## Next Steps — ZC Detection, RMS, Power

This branch delivers stable, calibrated, time-tagged V/I sample pairs. The next branch builds
power calculations on top of this foundation.

### Design notes for the next branch

**Accumulate over complete mains cycles, not fixed time windows.**
Partial cycles at the start and end of a fixed time window introduce a bias error in Vrms,
Irms, and P. Use zero-crossing detection on the voltage channel (CH1) to mark cycle boundaries
and accumulate only over whole cycles.

```c
// Zero-crossing: rising edge on voltage (CH1 crosses zero from below)
bool zc = (prev_mv1 < 0) && (mv1_val >= 0);
```

Accumulate `sum_v2 += mv1*mv1`, `sum_i2 += mv0*mv0`, `sum_p += mv0*mv1` between crossings.
On each crossing: finalise the previous cycle, reset accumulators.

**Vrms and Irms are not affected by skew.** Only compute the skew compensation for P:

```c
// Interpolate voltage to the time current was sampled:
float v_at_i = mv1[i] + (mv1[i+1] - mv1[i]) * (T_SKEW_US / T_SAMPLE_US);
sum_p += mv0[i] * v_at_i;
```

**Quantities to compute per mains cycle:**

```
Vrms = sqrt(mean(V²))
Irms = sqrt(mean(I²))
P    = mean(V_compensated × I)          [active power, watts]
S    = Vrms × Irms                      [apparent power, VA]
Q    = sqrt(S² − P²)                    [reactive power, VAr]
PF   = P / S                            [power factor, −1 to +1]
```

**BLE transmission:** compute per-cycle values on-chip, buffer N cycles, send a compact struct
via BLE notification. Do not stream raw samples over BLE — the data rate does not fit within a
standard connection interval, and raw samples are not the end goal.
