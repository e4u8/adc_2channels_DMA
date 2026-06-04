# adc_2channels_DMA — branch: v2.1-interleaved_RMS

Dual-channel ADC acquisition firmware for the **Dialog Semiconductor DA14706** (DA1470x family).  
Measures current (CH0, P0.5) and voltage (CH1, P0.6) back-to-back in a tight interleaved loop,
computes **AC RMS for both channels** using a numerically stable two-pass algorithm, and outputs
calibrated Vrms / Irms values as scaled integers over UART. Precise timing diagnostics are
provided via the ARM DWT cycle counter.

This branch adds RMS computation on top of the stable interleaved acquisition foundation from
`v2.1-interleaved` and is the direct predecessor to power calculations (P, S, Q, PF).

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
- [RMS Calculation](#rms-calculation)
  - [Why mean subtraction is required](#why-mean-subtraction-is-required)
  - [Two-pass algorithm](#two-pass-algorithm)
  - [Windowed accumulation across batches](#windowed-accumulation-across-batches)
  - [Why ZCD is not required at this stage](#why-zcd-is-not-required-at-this-stage)
  - [Output as scaled integers](#output-as-scaled-integers)
- [Calibration](#calibration)
  - [Signal conditioning constants](#signal-conditioning-constants)
  - [Calibration procedure](#calibration-procedure)
  - [Measurement results](#measurement-results)
- [Serial Output Protocol](#serial-output-protocol)
- [Inter-Channel Skew](#inter-channel-skew)
  - [What it is](#what-it-is)
  - [Why it is constant](#why-it-is-constant)
  - [Impact on power calculations](#impact-on-power-calculations)
  - [How to compensate](#how-to-compensate)
- [Throughput Analysis](#throughput-analysis)
  - [Measured results](#measured-results-1)
  - [Why sample_time and oversampling have no effect](#why-sample_time-and-oversampling-have-no-effect)
  - [Is 2200 samples/channel/s enough?](#is-2200-sampleschannels-enough)
  - [How fs_acq is measured](#how-fs_acq-is-measured)
  - [Why fs_acq varies slightly between windows](#why-fs_acq-varies-slightly-between-windows)
- [Multi-Task Architecture Notes](#multi-task-architecture-notes)
- [Build Configurations](#build-configurations)
- [Key Parameters](#key-parameters)
- [Next Steps — ZCD, Active Power, Power Factor](#next-steps--zcd-active-power-power-factor)

---

## Overview

This project is the third major iteration of a power-measurement firmware for 50 Hz mains signals.
It builds directly on `v2.1-interleaved` (stable interleaved acquisition with DWT timing) and adds
a complete AC RMS pipeline for both channels.

**Key capabilities:**

- Interleaved single-sample acquisition — CH0 and CH1 sampled back-to-back for every index `i`
- ARM DWT cycle counter for sub-microsecond timestamps with zero OS overhead
- One-time startup skew measurement (`*skew=`) — the phase-correction constant T_SKEW
- DWT-based true acquisition throughput (`*fs_acq`, `*us_pair`) — immune to UART print time
- **AC RMS computation** using a two-pass mean-subtraction algorithm (numerically stable)
- **1-second windowed RMS** — variance accumulated per batch, sqrt taken once per second
- **Scaled integer output** — no float `printf`, shared format between UART and future BLE payload
- Per-channel linear calibration (offset + gain) plus signal-chain scaling constants K_V and K_I
- Configurable batch size (default 64 pairs)
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
├── gpadc_app.c               # ADC acquisition + RMS task — all application logic
├── gpadc_app.h               # Task entry point declarations
└── config/
    ├── platform_devices.h    # Exported ADC device handle declarations
    ├── custom_config_ram.h   # BSP feature flags — RAM execution target
    ├── custom_config_qspi.h  # BSP feature flags — QSPI flash target
    └── custom_config_oqspi.h # BSP feature flags — OQSPI flash target
```

Hardware initialization is centralised in `main.c → prvSetupHardware()`. `gpadc_app_init()` is
intentionally empty — `gpadc_app_task()` is the single entry point for all acquisition and
computation logic.

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

  ② Per-batch RMS accumulation
     rms_mv0 = compute_ac_rms_mv(raw0)   ← two-pass, returns AC RMS in mV
     rms_mv1 = compute_ac_rms_mv(raw1)
     rms2_accum_ch0 += rms_mv0²          ← accumulate variance across batches
     rms2_accum_ch1 += rms_mv1²

  ③ Per-sample print  [disabled — #if 0]
     raw → mV conversion + CSV printf (kept for debugging, re-enable by changing #if 0 → #if 1)

  ④ Diagnostics — once per second
     total_pairs = batches_in_window × BATCH_SIZE
     fs_acq  = (total_pairs × 2 × CPU_CLOCK_HZ) / acq_cycles_accum
     us_pair = acq_cycles_accum / (total_pairs × 32)

     rms_mv1_w = sqrt( rms2_accum_ch1 / batches )   ← windowed Vrms in mV
     rms_mv0_w = sqrt( rms2_accum_ch0 / batches )   ← windowed Irms in mV

     v_rms = K_V × rms_mv1_w / 1000                 ← mains Volts
     i_rms = K_I × rms_mv0_w / HALL_SENSITIVITY     ← Amperes

     v_rms_cV = (int32_t)(v_rms × 100)              ← centivolts
     i_rms_mA = (int32_t)(i_rms × 1000)             ← milliamps

     Print: *fs_acq=<N>  *us_pair=<N>
     Print: *Vrms=<cV/100>.<cV%100> V  *Irms=<mA/1000>.<mA%1000> A
     Reset all accumulators
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

## RMS Calculation

### Why mean subtraction is required

Both analog signals are **bipolar AC** (mains voltage and Hall sensor current), but the signal
conditioning circuit adds a DC offset to each so the full swing fits within the ADC's positive
input range. The ADC therefore always reads a positive value:

```
adc_mv[i]  =  true_AC_signal[i]  +  DC_offset
```

The DC offset is an artifact of the signal conditioning circuit — it is **not** part of the
original signal. If the raw ADC values were squared directly:

```
RMS_wrong = sqrt( (1/N) × Σ adc_mv[i]² )
```

the DC offset term (typically ~1650 mV for a 3.3 V supply) dominates the sum and the AC
content is nearly lost to rounding. The correct approach removes the DC offset first.

### Two-pass algorithm

Implemented in `compute_ac_rms_mv()` in [gpadc_app.c](gpadc_app.c):

**Pass 1 — compute the batch mean** (estimates the DC offset):

```
mean = (1/N) × Σ adc_mv[i]
```

**Pass 2 — compute RMS of the zero-centered residuals:**

```
rms_mv = sqrt( (1/N) × Σ (adc_mv[i] − mean)² )
```

This is mathematically equivalent to the standard deviation (square root of variance). Squaring
small residuals `(adc_mv − mean)` instead of large absolute values keeps float32 precision well
within acceptable bounds even when `DC >> AC amplitude`.

The function is **intentionally isolated** — it takes only a raw buffer and its length. When
Zero-Crossing Detection is added in the next branch, only the call site changes (what buffer and
length are passed in), not this function.

### Windowed accumulation across batches

A single 64-sample batch covers ~1.44 mains cycles at the current sample rate — not an integer
number. Rather than computing RMS once per batch, variance is accumulated across all batches
within a 1-second window and the square root is taken once:

```
// per batch:
rms2_accum += compute_ac_rms_mv(...)²    ← accumulate variance

// once per second:
rms_window = sqrt( rms2_accum / batches_in_window )
```

This is mathematically correct because averaging variances from multiple batches of a stationary
signal converges to the true variance, and the sqrt is applied only once at the boundary.

### Why ZCD is not required at this stage

Zero-Crossing Detection (ZCD) aligns the accumulation window to complete mains cycles, eliminating
the partial-cycle truncation bias at the window boundary. At the current sample rate:

```
fs per channel ≈ 2220 samples/s
samples per mains cycle (50 Hz) ≈ 44.4
cycles in 1-second window ≈ 44.4   →   partial cycle = 0.4 / 44.4 ≈ 0.9%
```

The worst-case RMS error from the partial-cycle truncation is well below 1% over a 1-second
window — acceptable for Vrms and Irms at this stage.

**ZCD becomes mandatory** for active power P and power factor PF, because those require the
voltage and current samples to be aligned to the same complete cycles. ZCD is deferred to the
next branch.

### Output as scaled integers

The DA1470x SDK uses **newlib-nano**, which strips float `printf` support by default to save
flash. Rather than enabling it with `-u _printf_float` (~6–8 KB extra), results are converted
to scaled integers before printing. The same integers are used directly in the future BLE
payload — one conversion, two uses, no float format specifiers needed.

| Constant       | Value | Unit          | Example              |
|----------------|-------|---------------|----------------------|
| `V_RMS_SCALE`  | 100   | centivolts    | 230.45 V → `23045`   |
| `I_RMS_SCALE`  | 1000  | milliamps     |   1.500 A → `1500`   |

`int32_t` is used for both. `int16_t` would cover up to 327.67 V and 32.767 A respectively —
switch to `int16_t` for the BLE payload if range permits.

---

## Calibration

### Signal conditioning constants

Four constants in [gpadc_app.c](gpadc_app.c) describe the full signal chain from MCU ADC pin
back to the original physical quantity:

```c
#define K_V                       (289.269f)  // mains V per ADC V  [V/V]
#define K_I                       (1.298f)    // Hall mV per ADC mV [mV/mV]
#define HALL_SENSITIVITY_MV_PER_A (80.0f)     // Hall sensor datasheet [mV/A]
```

The conversion in code:

```c
float v_rms = K_V * rms_mv1_w / 1000.0f;                     // Volts
float i_rms = (K_I * rms_mv0_w) / HALL_SENSITIVITY_MV_PER_A; // Amps
```

There are also per-channel ADC-level calibration constants (applied before RMS computation):

```c
#define OFFSET_MV_CH0   0.0f   // subtract from raw mV reading
#define GAIN_CH0        1.0f   // divide by this factor
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f
```

### Calibration procedure

To derive K_V and K_I from a reference instrument:

1. Temporarily enable the raw calibration print in the diagnostics section:

```c
printf("*rms_mv_CH1=%"PRId32"  *rms_mv_CH0=%"PRId32"\n",
       (int32_t)(rms_mv1_w * 10), (int32_t)(rms_mv0_w * 10));
```

The printed integers are in units of **0.1 mV** (tenths of millivolt). Divide by 10 to get mV.

2. Record `rms_mv1_w` (CH1 voltage channel, mV) and `rms_mv0_w` (CH0 current channel, mV)
   alongside the reference instrument's Vrms and Irms readings (stable resistive load, no
   TRIAC switching):

```
K_V = PA_Vrms [V] × 1000 / rms_mv1_w [mV]
K_I = PA_Irms [A] × HALL_SENSITIVITY_MV_PER_A / rms_mv0_w [mV]
```

3. Update the `#define` values and recompile.

### Measurement results

Calibration was performed against a power analyser on a resistive heat fan load at three
operating modes. Before recalibration a **systematic ~3.4% low bias** was observed on both
channels across all modes, indicating a uniform scaling error in the previous K_V and K_I
values. After updating to `K_V = 289.269` and `K_I = 1.298`:

| Mode | PA Vrms (V) | MCU Vrms (V)    | Error | PA Irms (A) | MCU Irms (A)  | Error |
|------|-------------|-----------------|-------|-------------|---------------|-------|
| 1    | 231.8       | 223.7 – 224.1   | ~3.3% | 4.4         | 4.17 – 4.19   | ~5%   |
| 2    | 227.8       | 219.9 – 220.3   | ~3.4% | 8.4         | 8.10 – 8.14   | ~3.4% |
| OFF  | 235.9       | 227.8 – 228.1   | ~3.4% | 0           | 0.23 – 0.24   | —     |

**Notes:**
- Residual ~3.4% error after first calibration was a uniform scaling factor — consistent with
  a single measurement session at slightly different mains voltage than calibration conditions.
  ZCD absence (< 1% truncation error) and skew (< 0.3% on Vrms/Irms) are not responsible.
- The small Irms offset at OFF (~0.23 A) is the Hall sensor zero-current noise floor — the
  sensor has a residual output at zero load that the signal conditioning does not fully suppress.
  This is below the noise floor of the measurement and is acceptable for this application.
- The heat fan uses a TRIAC phase-cutting controller. Vrms and Irms are correctly computed for
  non-sinusoidal (phase-cut) waveforms by the definition-based RMS formula — no special handling
  is needed.

---

## Serial Output Protocol

All output is ASCII text over the retarget UART (enabled by `CONFIG_RETARGET` in
`custom_config_*.h`). Lines prefixed with `*` are diagnostic markers, not sample data.

| Line | When | Meaning |
|------|------|---------|
| `*skew=N cycles (~M us)` | Once at startup | CH0→CH1 inter-sample delay. Record this value — it is T_SKEW for power factor compensation |
| `*fs_acq=N  *us_pair=N` | Once per second | True ADC throughput and average pair time, measured from DWT |
| `*Vrms=N.NN V  *Irms=N.NNN A` | Once per second | Windowed AC RMS, scaled integers printed as fixed-point |
| `*rms_mv_CH1=N  *rms_mv_CH0=N` | Once per second (calibration mode) | Raw ADC channel RMS in 0.1 mV units — enable during calibration only |

Example output (normal operation):

```
*skew=6711 cycles (~209 us)
*fs_acq=4421  *us_pair=452
*Vrms=230.45 V  *Irms=4.380 A
*fs_acq=4420  *us_pair=452
*Vrms=230.47 V  *Irms=4.381 A
```

**Important:** `printf` over UART is synchronous and blocking on this platform. Each character
takes ~87 µs at 115200 baud. `BATCH_SIZE` directly affects how much time the task spends inside
`printf`, but does **not** affect `*fs_acq` or `*us_pair` — those are measured only over the
acquisition loop, excluding all print time.

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

This makes `fs_acq` immune to `BATCH_SIZE` and UART congestion. The old approach of counting
`sample_counter += BATCH_SIZE * 2` once per second produced values that dropped significantly
when print density increased because print time was included in the measurement window.

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

| Symbol                      | Default      | Purpose |
|-----------------------------|--------------|---------|
| `BATCH_SIZE`                | `64`         | Pairs per loop iteration. Larger → smoother RMS average, higher latency. Each unit costs 4 bytes of BSS (2 × uint16_t) |
| `CPU_CLOCK_HZ`              | `32000000`   | Must match `cm_sys_clk_init(sysclk_XTAL32M)`. Used in all DWT µs conversions |
| `OFFSET_MV_CH0/1`           | `0.0f`       | DC offset calibration per channel (mV), applied before RMS |
| `GAIN_CH0/1`                | `1.0f`       | Gain calibration per channel, applied before RMS |
| `K_V`                       | `289.269`    | Mains Volts per ADC Volt — signal chain scaling for voltage channel |
| `K_I`                       | `1.298`      | Hall sensor mV per ADC mV — signal chain scaling for current channel |
| `HALL_SENSITIVITY_MV_PER_A` | `80.0`       | Hall sensor sensitivity from datasheet [mV/A] |
| `V_RMS_SCALE`               | `100`        | Output scaling: result in centivolts (230.45 V → `23045`) |
| `I_RMS_SCALE`               | `1000`       | Output scaling: result in milliamps (1.500 A → `1500`) |

---

## Next Steps — ZCD, Active Power, Power Factor

This branch delivers stable, calibrated, windowed Vrms and Irms. The next branch builds
active power and power factor on top.

### Zero-Crossing Detection

ZCD replaces the fixed 1-second accumulation window with cycle-aligned windows. For Vrms and
Irms this improves the < 1% truncation bias to zero. For P and PF it is **mandatory** — partial
cycles introduce a systematic bias that cannot be averaged away.

```c
// Rising zero-crossing on voltage channel (after mean subtraction):
bool zc = (prev_mv1_centered < 0.0f) && (mv1_centered >= 0.0f);
```

The `compute_ac_rms_mv()` function in this branch is already designed for this: when ZCD is
added, only the call site changes (passing a cycle-aligned buffer instead of a fixed-length
batch). The function itself is unchanged.

### Active Power, Apparent Power, Reactive Power, Power Factor

With cycle-aligned V and I samples and skew compensation:

```
Vrms = sqrt( mean(V²) )
Irms = sqrt( mean(I²) )
P    = mean(V_compensated × I)          [active power, watts]
S    = Vrms × Irms                      [apparent power, VA]
Q    = sqrt(S² − P²)                    [reactive power, VAr]
PF   = P / S                            [power factor, −1 to +1]
```

### BLE Transmission

The scaled integers (`v_rms_cV`, `i_rms_mA`) introduced in this branch are already in the
correct format for BLE payload packing. The next branch will add `p_cW` (centiwatts) and
`pf_scaled` (PF × 1000) using the same pattern, and send a compact struct via a BLE
notification characteristic — no float encoding, no `-u _printf_float`, consistent with the
approach established here.
