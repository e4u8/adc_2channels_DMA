# adc_2channels_DMA

Dual-channel ADC acquisition firmware for the **Dialog Semiconductor DA14706** (DA1470x family). Samples current (CH0) and voltage (CH1) back-to-back using the GPADC adapter layer, streams mV pairs over UART, and measures the precise inter-channel time skew using the ARM DWT cycle counter.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Target](#hardware-target)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
  - [RTOS Task Layout](#rtos-task-layout)
  - [Acquisition Flow](#acquisition-flow)
- [ADC Configuration](#adc-configuration)
- [GPIO Pin Mapping](#gpio-pin-mapping)
- [Clock & Power Configuration](#clock--power-configuration)
- [Serial Output Protocol](#serial-output-protocol)
- [Inter-Channel Skew](#inter-channel-skew)
- [Calibration](#calibration)
- [Build Configurations](#build-configurations)
- [Key Parameters](#key-parameters)

---

## Overview

This project samples two analog signals — current (CH0, P0.5) and voltage (CH1, P0.6) — in a tight interleaved loop on the DA14706 BLE SoC. The acquisition uses Dialog's `ad_gpadc` adapter layer (synchronous blocking API) inside a FreeRTOS task. Results are streamed as ASCII CSV over UART.

**Key capabilities:**

- Interleaved single-sample acquisition — CH0 and CH1 sampled back-to-back per pair
- ARM DWT cycle counter for sub-microsecond inter-channel skew measurement
- One-time startup skew print (`*skew=N cycles (~M us)`) as the phase-correction constant for power-factor calculations
- Per-channel linear calibration (offset + gain)
- Configurable batch size (default 64 pairs per loop)
- Decimated UART output (`PRINT_EVERY = 2`) to reduce terminal load
- Firmware-side sample rate diagnostics (`*fs=N` once per second)
- Three execution targets: RAM, QSPI flash, OQSPI flash

---

## Hardware Target

| Parameter  | Value                                          |
|------------|------------------------------------------------|
| SoC        | Dialog Semiconductor DA14706-00                |
| CPU        | ARM Cortex-M33                                 |
| FPU        | FPv5-SP-D16 (single-precision hardware float)  |
| SDK        | DA1470x SDK                                    |
| RTOS       | FreeRTOS                                       |
| Toolchain  | arm-none-eabi-gcc                              |
| IDE        | Eclipse CDT (SmartSnippets / e² Studio)        |

---

## Project Structure

```
adc_2channels_DMA/
├── main.c                    # System init, clock setup, GPIO config, RTOS launch
├── gpadc_app.c               # ADC acquisition task — core application logic
├── gpadc_app.h               # Task entry point declarations
└── config/
    ├── platform_devices.h    # Exported ADC device handle declarations
    ├── custom_config_ram.h   # BSP feature flags for RAM execution target
    ├── custom_config_qspi.h  # BSP feature flags for QSPI flash target
    └── custom_config_oqspi.h # BSP feature flags for OQSPI flash target
```

---

## Architecture

### RTOS Task Layout

```
main()
  └─ SysInit task (highest priority, runs once)
       ├─ cm_sys_clk_init(sysclk_XTAL32M)   — 32 MHz system clock
       ├─ pm_sleep_mode_set(pm_mode_idle)
       ├─ prvSetupHardware()                 — GPIO and power domain init
       └─ creates: gpadc_app_task (Normal priority, runs forever)
```

`main()` creates only the `SysInit` task and starts the scheduler. `system_init()` handles all hardware setup and then deletes itself, leaving only the GPADC task running.

### Acquisition Flow

```
[Startup — once]
  Enable DWT CYCCNT (32 MHz tick, sub-µs resolution)
  Open CH0 → read 1 dummy → timestamp t0 → Close CH0
  Open CH1 → read 1 dummy → timestamp t1 → Close CH1
  Print: *skew = (t1 - t0) cycles and µs

[Main loop — repeats forever]
  For i = 0 .. BATCH_SIZE-1:
    1. Open CH0 → read raw0[i] → Close CH0
    2. Open CH1 → read raw1[i] → Close CH1   ← fixed skew after step 1

  For i = 0 .. BATCH_SIZE-1:
    Convert raw0[i] → mV (with calibration)
    Convert raw1[i] → mV (with calibration)
    If (i % PRINT_EVERY == 0): print "<mv0>,<mv1>"

  sample_counter += BATCH_SIZE * 2
  Every 1 second: print "*fs=<sample_counter>", reset counter
```

`ad_gpadc_read_nof_conv(handle, 1, &dest)` is **synchronous/blocking** — it triggers one ADC conversion and waits for the result before returning. The CPU yields to the RTOS scheduler during each conversion.

---

## ADC Configuration

The GPADC adapter configuration is defined in `config/platform_devices.h` (and the corresponding `.c` file). The adapter manages opening/closing the hardware resource and applying IO mux settings per handle.

| Parameter       | Value                        |
|-----------------|------------------------------|
| CH0 signal      | Current                      |
| CH1 signal      | Voltage                      |
| API             | `ad_gpadc` (adapter layer)   |
| Acquisition     | Synchronous, 1 sample/call   |

---

## GPIO Pin Mapping

| Signal    | Port | Pin | GPIO Function      |
|-----------|------|-----|--------------------|
| CH0 (current) | P0 | 5 | `HW_GPIO_FUNC_ADC` |
| CH1 (voltage) | P0 | 6 | `HW_GPIO_FUNC_ADC` |

Pins are configured in `periph_init()` (called from `prvSetupHardware()`) as high-impedance ADC inputs. The `ad_gpadc` adapter layer manages their mux state automatically when a handle is opened/closed.

---

## Clock & Power Configuration

| Parameter      | Setting                                        |
|----------------|------------------------------------------------|
| System clock   | 32 MHz XTAL32M                                 |
| AHB divider    | div1 (32 MHz)                                  |
| APB divider    | div1 (32 MHz)                                  |
| Sleep mode     | `pm_mode_idle` (CPU halts between tasks)       |
| Wakeup mode    | `pm_sys_wakeup_mode_fast`                      |
| DWT clock      | 32 MHz → 1 DWT tick = 31.25 ns                 |

`pm_mode_idle` keeps peripherals active while letting the CPU sleep between task activations, avoiding busy-wait loops.

---

## Serial Output Protocol

All output is ASCII over the retarget UART:

| Line                         | Meaning                                                   |
|------------------------------|-----------------------------------------------------------|
| `*skew=N cycles (~M us)`     | One-time CH0→CH1 inter-sample delay measured at startup   |
| `<mv0>,<mv1>`                | Current mV, Voltage mV — printed for 1 in every `PRINT_EVERY` pairs |
| `*fs=N`                      | Total samples (both channels) acquired in the last second |

Example output:

```
*skew=6400 cycles (~200 us)
1234,987
1235,986
...
*fs=12800
```

---

## Inter-Channel Skew

Because CH0 and CH1 are sampled sequentially (not simultaneously), there is a fixed time offset between `raw0[i]` and `raw1[i]`:

```
skew = t_ch1_done - t_ch0_done
     = CH0_close_overhead + CH1_open_overhead + CH1_conversion_time
```

This is measured once at startup using the DWT cycle counter and printed as `*skew=`. It is **constant** across all sample pairs and is the **phase-correction constant T_SKEW** needed when computing power factor from the voltage and current waveforms.

---

## Calibration

Each channel has two constants at the top of [gpadc_app.c](gpadc_app.c):

```c
#define OFFSET_MV_CH0   0.0f   // subtract from raw mV
#define GAIN_CH0        1.0f   // divide by this

#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f
```

Applied by `correct_mv()`:

```
corrected_mv = (raw_mv - offset) / gain
```

Negative results are clamped to 0. The defaults pass raw values through unchanged. To calibrate, apply a known reference voltage, compare to the reported value, and adjust `OFFSET_MV` / `GAIN` accordingly.

---

## Build Configurations

| Configuration              | Optimization | Execution target |
|----------------------------|-------------|------------------|
| DA1470x-00-Debug_RAM       | -O0         | SRAM             |
| DA1470x-00-Debug_OQSPI     | -O0         | OQSPI flash      |
| DA1470x-00-Release_OQSPI   | -Os         | OQSPI flash      |

RAM execution is recommended during development (no flash erase cycle). OQSPI Release is the production target.

---

## Key Parameters

All commonly-tuned values in [gpadc_app.c](gpadc_app.c):

| Symbol           | Default         | Purpose                                              |
|------------------|-----------------|------------------------------------------------------|
| `BATCH_SIZE`     | `64`            | (CH0, CH1) pairs collected per loop iteration        |
| `PRINT_EVERY`    | `2`             | Print 1 in every N pairs over UART                   |
| `CPU_CLOCK_HZ`   | `32000000`      | Must match `cm_sys_clk_init(sysclk_XTAL32M)`         |
| `OFFSET_MV_CH0`  | `0.0f`          | DC offset calibration for CH0 (mV)                   |
| `GAIN_CH0`       | `1.0f`          | Gain calibration for CH0                             |
| `OFFSET_MV_CH1`  | `0.0f`          | DC offset calibration for CH1 (mV)                   |
| `GAIN_CH1`       | `1.0f`          | Gain calibration for CH1                             |
