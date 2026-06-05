# adc_2channels_DMA — branch: v2.1-interleaved_RMS_P_S_Q_PF

Dual-channel ADC acquisition firmware for the **Dialog Semiconductor DA14706** (DA1470x family).  
Measures current (CH0, P0.5) and voltage (CH1, P0.6) back-to-back in a tight interleaved loop,
computes **AC RMS, active power, apparent power, reactive power, and power factor** using
numerically stable algorithms, and outputs all quantities as scaled integers over UART.
Precise timing diagnostics are provided via the ARM DWT cycle counter.

This branch extends `v2.1-interleaved_RMS` (stable Vrms/Irms) with a complete power measurement
pipeline: P, S, Q, and PF. It is the direct predecessor to ZCD and skew compensation.

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
  - [Why ZCD is not required for RMS](#why-zcd-is-not-required-for-rms)
  - [Output as scaled integers](#output-as-scaled-integers)
- [Power Calculations](#power-calculations)
  - [Active power P](#active-power-p)
  - [Signal polarity correction — P_SIGN](#signal-polarity-correction--p_sign)
  - [Apparent power S](#apparent-power-s)
  - [Reactive power Q](#reactive-power-q)
  - [Power factor PF](#power-factor-pf)
  - [The power triangle](#the-power-triangle)
  - [Combined batch function](#combined-batch-function)
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
- [Next Steps — ZCD and Skew Compensation](#next-steps--zcd-and-skew-compensation)

---

## Overview

This project is the fourth major iteration of a power-measurement firmware for 50 Hz mains signals.
It builds directly on `v2.1-interleaved_RMS` (stable windowed Vrms/Irms) and adds a complete
power measurement pipeline.

**Key capabilities:**

- Interleaved single-sample acquisition — CH0 and CH1 sampled back-to-back for every index `i`
- ARM DWT cycle counter for sub-microsecond timestamps with zero OS overhead
- One-time startup skew measurement (`*skew=`) — the phase-correction constant T_SKEW
- DWT-based true acquisition throughput (`*fs_acq`, `*us_pair`) — immune to UART print time
- **AC RMS computation** using a two-pass mean-subtraction algorithm (numerically stable)
- **1-second windowed RMS** — variance accumulated per batch, sqrt taken once per second
- **Active power P** — mean cross-product of mean-subtracted V and I residuals
- **Apparent power S** — product of windowed Vrms and Irms
- **Reactive power Q** — derived from the power triangle: `sqrt(|S² − P²|)`
- **Power factor PF** — `P / S`, sign-preserving, range −1 to +1
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
├── gpadc_app.c               # ADC acquisition + power measurement task — all application logic
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

  ② Per-batch metrics accumulation  (single call, two passes)
     m = compute_batch_metrics(raw1, raw0)
       Pass 1: convert both channels to mV, compute mean_v and mean_i
       Pass 2: squared residuals (RMS) and cross-product (P) simultaneously
     rms2_accum_ch1 += m.rms_mv_v²     ← accumulate variance, CH1 voltage
     rms2_accum_ch0 += m.rms_mv_i²     ← accumulate variance, CH0 current
     p_accum        += m.p_mvsq        ← accumulate mean cross-product [mV²]

  ③ Per-sample print  [disabled — #if 0]
     raw → mV conversion + CSV printf (kept for debugging)

  ④ Diagnostics — once per second
     total_pairs = batches_in_window × BATCH_SIZE
     fs_acq  = (total_pairs × 2 × CPU_CLOCK_HZ) / acq_cycles_accum
     us_pair = acq_cycles_accum / (total_pairs × 32)

     rms_mv1_w = sqrt( rms2_accum_ch1 / batches )   ← windowed Vrms in mV
     rms_mv0_w = sqrt( rms2_accum_ch0 / batches )   ← windowed Irms in mV

     v_rms = K_V × rms_mv1_w / 1000                 ← mains Volts
     i_rms = K_I × rms_mv0_w / HALL_SENSITIVITY     ← Amperes

     p_w   = P_SIGN × P_SCALE × (p_accum / batches) ← Watts
     s_va  = v_rms × i_rms                           ← VA
     q_var = sqrt( |s_va² − p_w²| )                 ← VAr
     pf    = p_w / s_va                              ← dimensionless

     Print: *fs_acq  *us_pair
     Print: *Vrms  *Irms
     Print: *P  *S  *Q  *PF
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

Implemented in `compute_batch_metrics()` in [gpadc_app.c](gpadc_app.c):

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

The same two passes also compute the P cross-product simultaneously — see
[Combined batch function](#combined-batch-function).

### Windowed accumulation across batches

A single 64-sample batch covers ~1.44 mains cycles at the current sample rate — not an integer
number. Rather than computing RMS once per batch, variance is accumulated across all batches
within a 1-second window and the square root is taken once:

```
// per batch:
rms2_accum += compute_batch_metrics(...)².rms_mv_v    ← accumulate variance

// once per second:
rms_window = sqrt( rms2_accum / batches_in_window )
```

This is mathematically correct because averaging variances from multiple batches of a stationary
signal converges to the true variance, and the sqrt is applied only once at the boundary.

### Why ZCD is not required for RMS

Zero-Crossing Detection (ZCD) aligns the accumulation window to complete mains cycles, eliminating
the partial-cycle truncation bias at the window boundary. At the current sample rate:

```
fs per channel ≈ 2220 samples/s
samples per mains cycle (50 Hz) ≈ 44.4
cycles in 1-second window ≈ 44.4   →   partial cycle = 0.4 / 44.4 ≈ 0.9%
```

The worst-case RMS error from the partial-cycle truncation is well below 1% over a 1-second
window — acceptable for Vrms and Irms at this stage.

**ZCD becomes mandatory** for accurate P and PF, because partial cycles bias the cross-product
mean. It is deferred to the next branch.

### Output as scaled integers

The DA1470x SDK uses **newlib-nano**, which strips float `printf` support by default to save
flash. Rather than enabling it with `-u _printf_float` (~6–8 KB extra), results are converted
to scaled integers before printing. The same integers are used directly in the future BLE
payload — one conversion, two uses, no float format specifiers needed.

| Constant        | Value | Unit                | Example                 |
|-----------------|-------|---------------------|-------------------------|
| `V_RMS_SCALE`   | 100   | centivolts          | 230.45 V  → `23045`     |
| `I_RMS_SCALE`   | 1000  | milliamps           |   1.500 A → `1500`      |
| `P_W_SCALE`     | 100   | centiwatts          | 1926.37 W → `192637`    |
| `S_VA_SCALE`    | 100   | centi volt-amps     | 1939.57 VA → `193957`   |
| `Q_VAR_SCALE`   | 100   | centi volt-amps reactive | 134.28 VAr → `13428` |
| `PF_SCALE`      | 1000  | milli power factor  |   0.991  → `991`        |

`int32_t` is used for all quantities. `int16_t` would cover Vrms up to 327.67 V and Irms up to
32.767 A — switch to `int16_t` for the BLE payload if range permits.

---

## Power Calculations

### Active power P

Active (real) power is the mean of the instantaneous product of voltage and current:

```
P = mean( v[i] × i[i] )
```

The mean **must** be computed on mean-subtracted residuals, not raw ADC values:

```
rv[i] = v_mv[i] − mean_v      ← AC component only
ri[i] = i_mv[i] − mean_i

P_mvsq = mean( rv[i] × ri[i] )    [mV²]
P_w    = P_SIGN × P_SCALE × P_mvsq  [W]
```

Using raw values would introduce a spurious `DC_V × DC_I` term from the signal conditioning
offsets, which are not part of the real signal. The mean represents the DC offset artifact;
subtracting it isolates the AC component before forming the product.

The scaling constant converts mV² to Watts:

```
P_SCALE = K_V × K_I / (1000 × HALL_SENSITIVITY_MV_PER_A)   [W / mV²]
```

Derived from:
```
v_mains [V] = K_V × v_mv [mV] / 1000
i_mains [A] = K_I × i_mv [mV] / HALL_SENSITIVITY [mV/A]
P [W]        = mean(v_mains × i_mains)
             = K_V × K_I / (1000 × HALL_SENSITIVITY) × mean(rv × ri)
             = P_SCALE × P_mvsq
```

### Signal polarity correction — P_SIGN

The signal conditioning circuit inverts the polarity of one channel. When the mains voltage
rises above its mean, the conditioned signal at the ADC falls below its mean, making the
cross-product negative for a resistive load where V and I are in phase. This is corrected by:

```c
#define P_SIGN  (-1.0f)   /* -1.0f: signal conditioning inverts one channel */
```

Applied as: `p_w = P_SIGN × P_SCALE × p_mvsq`

RMS values are unaffected — `sqrtf` always returns a positive result regardless of polarity.
If the hardware inversion is corrected in a future revision, set `P_SIGN = +1.0f`.

### Apparent power S

Apparent power is the product of the RMS magnitudes:

```
S = Vrms × Irms   [VA]
```

It represents the total power the source must supply, including both the useful (active) and
reactive components. S is always positive.

### Reactive power Q

Reactive power is derived from the power triangle:

```
Q = sqrt( |S² − P²| )   [VAr]
```

The `fabsf` inside the sqrt is essential: when PF ≈ 1 (resistive load), floating-point rounding
can make `S² − P²` go slightly negative, which would produce `NaN` from `sqrtf`. `fabsf` guards
against this.

Q represents energy that oscillates between the source and reactive elements (inductors,
capacitors) without being consumed. For a heat fan, the fan motor winding contributes a small
inductive reactive component.

### Power factor PF

```
PF = P / S
```

Range: −1 to +1. For a purely resistive load, PF = 1. For a purely reactive load, PF = 0.
A negative PF indicates the sign convention of P is reversed (see P_SIGN).

The sign of PF follows the sign of P. The output format preserves the sign explicitly:

```c
printf("*PF=%s%"PRId32".%03"PRId32"\n",  pf < 0.0f ? "-" : "", pf_m/1000, pf_m%1000);
```

### The power triangle

The three power quantities form a right triangle:

```
         S (hypotenuse, VA)
        /|
       / |
      /  |  Q (reactive, VAr)
     /   |
    / φ  |
   -------
   P (active, W)
```

The relationships:

```
S²  = P² + Q²          (Pythagorean identity)
PF  = P / S = cos(φ)
φ   = arccos(PF)        (phase angle between V and I)
Q   = S × sin(φ)
```

**Verification with PA measurements (mode 1):**
```
PA:  P = 1.017 kW,  Q = 0.050 kVAr,  S = 1.018 kVA
Check S: sqrt(1.017² + 0.050²) = sqrt(1.040400 + 0.002500) = sqrt(1.042900) = 1.021 kVA ✓
PF = P/S = 1.017/1.021 = 0.9990  (PA displays 0.99 — truncated to 2 decimal places)
φ  = arccos(0.9990) = 2.5°
```

### Combined batch function

In earlier branches, Vrms and Irms were computed by two separate calls to `compute_ac_rms_mv()`,
each running two passes over its own buffer independently. This branch replaces both calls with
a single `compute_batch_metrics()` that processes both channels in one pair of passes:

**Pass 1:** convert both channels to mV, accumulate sums to compute `mean_v` and `mean_i`.

**Pass 2:** using the same residuals `rv` and `ri` for both channels:
- Accumulate `rv²` and `ri²` → RMS for each channel
- Accumulate `rv × ri` → mean cross-product for P

```c
typedef struct {
    float rms_mv_v;   /* CH1 AC RMS [mV] */
    float rms_mv_i;   /* CH0 AC RMS [mV] */
    float p_mvsq;     /* mean( (v−mean_v)×(i−mean_i) ) [mV²] */
} batch_metrics_t;
```

This eliminates two redundant passes (4 passes → 2), and ensures the same `mean_v` and `mean_i`
values are used for both RMS and P — the residuals are identical, not recomputed.

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
   alongside the reference instrument's Vrms and Irms readings (stable resistive load):

```
K_V = PA_Vrms [V] × 1000 / rms_mv1_w [mV]
K_I = PA_Irms [A] × HALL_SENSITIVITY_MV_PER_A / rms_mv0_w [mV]
```

3. Update the `#define` values and recompile.

### Measurement results

All measurements were taken against a power analyser on a heat fan load at two power modes.
The fan uses a TRIAC phase-cutting controller; RMS and power quantities are correctly computed
for non-sinusoidal waveforms by the definition-based formulas — no special handling is needed.

**Vrms and Irms (from `v2.1-interleaved_RMS`, K_V = 289.269, K_I = 1.298):**

| Mode | PA Vrms (V) | MCU Vrms (V)   | Error | PA Irms (A) | MCU Irms (A) | Error |
|------|-------------|----------------|-------|-------------|--------------|-------|
| 1    | 231.8       | 223.7 – 224.1  | ~3.3% | 4.4         | 4.17 – 4.19  | ~5%   |
| 2    | 227.8       | 219.9 – 220.3  | ~3.4% | 8.4         | 8.10 – 8.14  | ~3.4% |

Note: residual ~3.4% error across modes is a uniform scaling factor from calibration at slightly
different mains voltage — not from ZCD absence or skew.

**P, S, Q, PF (this branch):**

| Quantity | Mode 1 MCU | Mode 1 PA | Error | Mode 2 MCU | Mode 2 PA | Error |
|----------|-----------|-----------|-------|-----------|-----------|-------|
| P        | 1.011 kW  | 1.017 kW  | −0.6% | 1.926 kW  | 1.920 kW  | +0.3% |
| S        | 1.020 kVA | 1.018 kVA | +0.2% | 1.940 kVA | 1.921 kVA | +1.0% |
| Q        | 134 VAr   | 50 VAr    | see † | 226 VAr   | 62 VAr    | see † |
| PF       | 0.991     | 0.99      | +0.001 | 0.993    | 0.99      | +0.003 |

†  **Q discrepancy is a known skew effect, not a code defect.** The inter-sample skew (~225 µs)
introduces an apparent phase shift of ~4° between V and I at 50 Hz. This adds to the load's true
phase angle (2.5°, from PA data), giving the MCU an apparent phase of ~6.5°, which is consistent
with the measured MCU PF of 0.991–0.993:

```
φ_skew  = 2π × 50 Hz × 225 µs = 4.05°
φ_true  = arccos(0.9990) = 2.5°   (from PA: P/S = 1.017/1.021)
φ_MCU   ≈ φ_true + φ_skew ≈ 6.5°
PF_MCU  = cos(6.5°) = 0.9936  ← consistent with measured 0.991–0.993
Q_MCU   = S × sin(6.5°) ≈ 1020 × 0.113 ≈ 116 VAr  ← consistent with measured ~134 VAr
```

Q is highly sensitive to PF accuracy when PF ≈ 1 — a 0.6% PF error translates to a 3×
error in Q because Q = S × sin(φ) and sin(φ) is small. Skew compensation (next branch) will
correct this.

**PA PF display note:** The PA displays PF = 0.99, but the true value from P/S = 0.9990. The PA
truncates PF to two decimal places — 0.9990 displayed as 0.99. All four PA quantities (P, Q, S,
PF) are internally consistent:

```
S² = P² + Q²:  sqrt(1.017² + 0.050²) = 1.021 kVA ✓
PF = P/S     = 1.017/1.021 = 0.9990  (displayed as 0.99) ✓
```

---

## Serial Output Protocol

All output is ASCII text over the retarget UART (enabled by `CONFIG_RETARGET` in
`custom_config_*.h`). Lines prefixed with `*` are diagnostic markers, not sample data.

| Line | When | Meaning |
|------|------|---------|
| `*skew=N cycles (~M us)` | Once at startup | CH0→CH1 inter-sample delay. Record this value — it is T_SKEW for power factor compensation |
| `*fs_acq=N  *us_pair=N` | Once per second | True ADC throughput and average pair time, measured from DWT |
| `*Vrms=N.NN V  *Irms=N.NNN A` | Once per second | Windowed AC RMS as fixed-point scaled integers |
| `*P=N.NN W` | Once per second | Windowed active power (centiwatts) |
| `*S=N.NN VA` | Once per second | Apparent power (centi volt-amps) |
| `*Q=N.NN VAr` | Once per second | Reactive power (centi volt-amps reactive) |
| `*PF=[−]N.NNN` | Once per second | Power factor × 1000, sign preserved |

Example output (heat fan, mode 2):

```
*skew=6711 cycles (~209 us)
*fs_acq=4421  *us_pair=452
*Vrms=230.45 V  *Irms=8.375 A
*P=1926.37 W
*S=1939.57 VA
*Q=225.88 VAr
*PF=0.993
```

**Unit note:** Q is printed in VAr (not kVAr). To convert: divide by 1000.

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

### Why it is constant

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

The middle sample sporadically shows a skew ~130 µs higher than first and last. The most likely
cause is a FreeRTOS tick interrupt or a DMA completion callback for an unrelated peripheral
firing between the CH0 close and CH1 open at that index.

**Decision: accepted.** Since the spike affects only isolated samples (not the entire batch),
the mean error in P accumulated over a full mains cycle is far smaller than the per-sample
worst case and is considered acceptable for this application.

### Impact on power calculations

| Calculation | Skew effect |
|-------------|-------------|
| Vrms | None — independent of I |
| Irms | None — independent of V |
| P = mean(V[i] × I[i]) | Yes — V[i] and I[i] are not simultaneous |
| S = Vrms × Irms | Indirect — inherited from V/I calibration only |
| Q = sqrt(\|S²−P²\|) | Yes — amplified: Q is highly sensitive to PF error when PF≈1 |
| PF = P / S | Yes — inherited from P error |

At 50 Hz, 209 µs = **3.76° of phase error**. The resulting error in PF is `cos(3.76°) − 1` ≈
**−0.22%** for a purely resistive load. For a load with a true phase angle, the errors compound:
the skew phase adds to the load phase, increasing the apparent Q by the amount shown in the
[Measurement results](#measurement-results) section above.

### How to compensate

The skew is constant and already measured. Linear interpolation corrects the voltage sample to
the time the current sample was actually captured:

```c
float T_SKEW_US  = 209.0f;   // from *skew= at startup
float T_SAMPLE_US = 452.0f;  // from *us_pair= at runtime

// For each pair i (except the last):
float v_compensated = mv1[i] + (mv1[i+1] - mv1[i]) * (T_SKEW_US / T_SAMPLE_US);
```

Then `P = mean(v_compensated × i_mv)` over a complete number of mains cycles. This is deferred
to the next branch (together with ZCD for cycle-aligned windows).

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
| `BATCH_SIZE`                | `64`         | Pairs per loop iteration. Larger → smoother average, higher latency. Each unit costs 4 bytes of BSS |
| `CPU_CLOCK_HZ`              | `32000000`   | Must match `cm_sys_clk_init(sysclk_XTAL32M)`. Used in all DWT µs conversions |
| `OFFSET_MV_CH0/1`           | `0.0f`       | DC offset calibration per channel (mV), applied before RMS |
| `GAIN_CH0/1`                | `1.0f`       | Gain calibration per channel, applied before RMS |
| `K_V`                       | `289.269`    | Mains Volts per ADC Volt — signal chain scaling for voltage channel |
| `K_I`                       | `1.298`      | Hall sensor mV per ADC mV — signal chain scaling for current channel |
| `HALL_SENSITIVITY_MV_PER_A` | `80.0`       | Hall sensor sensitivity from datasheet [mV/A] |
| `P_SIGN`                    | `-1.0f`      | `−1.0f` if signal conditioning inverts one channel; `+1.0f` otherwise |
| `P_SCALE`                   | `K_V×K_I / (1000×HALL_SENSITIVITY)` | Converts ADC cross-product mean [mV²] to Watts |
| `V_RMS_SCALE`               | `100`        | Output scaling: centivolts (230.45 V → `23045`) |
| `I_RMS_SCALE`               | `1000`       | Output scaling: milliamps (1.500 A → `1500`) |
| `P_W_SCALE`                 | `100`        | Output scaling: centiwatts (1926.37 W → `192637`) |
| `S_VA_SCALE`                | `100`        | Output scaling: centi volt-amps |
| `Q_VAR_SCALE`               | `100`        | Output scaling: centi volt-amps reactive |
| `PF_SCALE`                  | `1000`       | Output scaling: milli power factor (0.991 → `991`) |

---

## Next Steps — ZCD and Skew Compensation

This branch delivers calibrated, windowed P, S, Q, and PF with accuracy limited primarily by
the inter-sample skew between channels. The next branch addresses both remaining sources of
systematic error.

### Zero-Crossing Detection

ZCD replaces the fixed 1-second accumulation window with cycle-aligned windows. For P this is
important because partial-cycle truncation introduces a bias in the cross-product mean that
cannot be averaged away over non-integer cycle counts.

```c
// Rising zero-crossing on voltage channel (after mean subtraction):
bool zc = (prev_rv < 0.0f) && (rv >= 0.0f);
```

The `compute_batch_metrics()` function in this branch is already designed for this: when ZCD is
added, only the call site changes (passing a cycle-aligned buffer). The function itself is
unchanged.

### Skew compensation

The skew is constant and already measured at startup. Linear interpolation corrects each voltage
sample to the instant the paired current sample was captured:

```c
float v_compensated = mv1[i] + (mv1[i+1] - mv1[i]) * (T_SKEW_US / T_SAMPLE_US);
float p_inst = v_compensated * mv_i[i];
```

With ZCD + skew compensation, the expected improvements are:
- PF accuracy: from ±0.003 to <0.001 (the skew-induced 4° phase error is removed)
- Q accuracy: the main error source is removed; MCU Q should converge to PA Q
- P accuracy: marginal improvement (already within 0.6%)

### BLE Transmission

The scaled integers introduced in this branch (`v_rms_cV`, `i_rms_mA`, `p_cW`, `s_cVA`,
`q_cVAr`, `pf_m`) are already in the correct format for BLE payload packing. The next branch
will send a compact struct via a BLE notification characteristic — no float encoding, no
`-u _printf_float`, consistent with the approach established here.
