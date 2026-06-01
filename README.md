# adc_2channels_DMA

Dual-channel ADC acquisition firmware for the **Dialog Semiconductor DA14706** (DA1470x family), using DMA-driven transfers and FreeRTOS. The two channels are sampled in an interleaved fashion — voltage (CH0) and current (CH1) back-to-back — at ~2.5 kHz per channel, with time-aligned pairs streamed over UART for power-factor analysis.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Target](#hardware-target)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
  - [RTOS Task Layout](#rtos-task-layout)
  - [Acquisition Flow](#acquisition-flow)
  - [DMA Pipeline](#dma-pipeline)
- [ADC Configuration](#adc-configuration)
- [GPIO Pin Mapping](#gpio-pin-mapping)
- [Clock & Power Configuration](#clock--power-configuration)
- [UART Output Protocol](#uart-output-protocol)
- [Calibration](#calibration)
- [Build Configurations](#build-configurations)
- [Building the Project](#building-the-project)
- [Key Parameters](#key-parameters)

---

## Overview

This project demonstrates efficient dual-channel analog acquisition on the DA14706 BLE SoC. The General Purpose ADC (GPADC) is driven via DMA, meaning the CPU is free to run other tasks while the hardware moves conversion results directly into memory. FreeRTOS coordinates the acquisition task with system initialization.

**Key capabilities:**
- Two time-aligned channels: CH0 = voltage (P0.5), CH1 = current (P0.6)
- Interleaved single-sample acquisition — V and I sampled back-to-back per pair
- Fixed ~200 µs skew between paired samples (constant, correctable in PF calc)
- Effective rate: ~2.5 kHz per channel (50 samples/cycle at 50 Hz)
- 4× oversampling and chopping enabled for improved accuracy
- Input range up to 3.6 V with built-in attenuator
- Per-channel calibration (offset + gain correction)
- Raw pair streaming every `PRINT_EVERY` pairs for waveform verification
- Firmware-side sample rate diagnostics (`*fs=<count>` every second)
- Three execution targets: RAM, QSPI flash, OQSPI flash

---

## Hardware Target

| Parameter | Value |
|-----------|-------|
| SoC | Dialog Semiconductor DA14706-00 |
| CPU | ARM Cortex-M33 |
| FPU | FPv5-SP-D16 (single-precision hardware float) |
| SDK | DA1470x SDK (symlinked as `sdk/`) |
| RTOS | FreeRTOS |
| Toolchain | arm-none-eabi-gcc |
| IDE | Eclipse CDT (SmartSnippets / e² Studio) |

---

## Project Structure

```
adc_2channels_DMA/
├── main.c                      # System init, clock setup, GPIO config, RTOS launch
├── gpadc_app.c                 # ADC acquisition task — core application logic
├── gpadc_app.h                 # Task entry point declarations
├── config/
│   ├── platform_devices.c      # ADC and DMA hardware configuration structs
│   ├── platform_devices.h      # Exported device handle declarations
│   ├── custom_config_ram.h     # BSP config for RAM execution target
│   ├── custom_config_qspi.h    # BSP config for QSPI flash execution target
│   └── custom_config_oqspi.h   # BSP config for OQSPI flash execution target
└── sdk/                        # Symlinks to DA1470x SDK modules
    ├── FreeRTOS/               # RTOS kernel + DA1470x portable layer
    ├── adapters/               # GPADC adapter (ad_gpadc API)
    ├── peripherals/            # Low-level hardware drivers (hw_gpadc, hw_dma, …)
    ├── bsp_include/            # Board support package headers
    ├── osal/                   # OS abstraction layer
    ├── sys_man/                # System management (clock, power)
    └── ldscripts/              # Linker scripts for each memory target
```

---

## Architecture

### RTOS Task Layout

```
main()
  └─ SysInit task (highest priority)
       ├─ cm_sys_clk_init()       — 32 MHz XTAL32M system clock
       ├─ pm_sleep_mode_set()     — Idle sleep mode
       ├─ prvSetupHardware()      — GPIO and power domain init
       └─ gpadc_app_task (Normal priority)
            └─ Acquisition loop (runs forever)
```

`main()` creates a single SysInit task that runs at elevated priority to complete all hardware initialization before handing off to the lower-priority acquisition task.

### Acquisition Flow

Each iteration of `gpadc_app_task` performs the following sequence:

```
[Warmup]
  Open CH0 → read 1 sample → Close CH0   (lets S&H capacitor settle)
  Open CH1 → read 1 sample → Close CH1

[Main loop — repeats forever]
  For i = 0 .. BATCH_SIZE-1:
    1. Open CH0 → single DMA read → raw0[i] → Close CH0
    2. Open CH1 → single DMA read → raw1[i] → Close CH1   ← ~200 µs after step 1

  Convert raw0[i] and raw1[i] to millivolts with calibration.
  Print every PRINT_EVERY-th pair over UART.
  Every 1 second: print "*fs=<total_samples>" diagnostic.
```

CH0 (voltage) and CH1 (current) are sampled **back-to-back for every index i**,
giving a fixed time skew of ~200 µs between `raw0[i]` and `raw1[i]`.
This skew is constant across all pairs and will be corrected by a single
phase-offset term in the PF calculation (Step 2).

### DMA Pipeline

```
GPADC hardware register
        │  (fires after each conversion)
        ▼
   DMA Channel 0
        │  (1 transfer, terminal-count IRQ)
        ▼
   raw0[i] or raw1[i]   (uint16_t in retained RAM)
        │  (DMA complete → unblocks task)
        ▼
   gpadc_app_task        (advances i, opens next channel)
```

The DMA is configured in **one-shot mode** with `irq_nr_of_trans = 0` (terminal-count
only), making it valid for `nof_conv = 1`. Because the task blocks on the DMA
completion callback, the CPU is yielded to the RTOS scheduler during each
conversion — zero busy-waiting.

---

## ADC Configuration

| Parameter | Value |
|-----------|-------|
| Peripheral | HW_GPADC_1 |
| Input mode | Single-ended |
| Input range | 0 – 3.6 V (attenuator enabled) |
| Sample time | 4 (= 32 ADC clock cycles) |
| Oversampling | 4× (HW_GPADC_OVERSAMPLING_4_SAMPLES) |
| Chopping | Enabled (reduces offset drift) |
| Continuous mode | Enabled (free-running during a DMA burst) |
| Result mode | Normal (16-bit unsigned, left-aligned) |
| DMA channel | HW_DMA_CHANNEL_0 (must be even — hardware constraint) |
| DMA priority | HW_DMA_PRIO_2 (medium) |
| DMA mode | One-shot (non-circular) |
| DMA IRQ trigger | After `ADC_NOF_CONV` transfers |

Oversampling averages 4 consecutive conversions per result, improving SNR at the cost of proportionally lower throughput. Chopping periodically reverses the input polarity to cancel systematic offset errors.

---

## GPIO Pin Mapping

| Signal | Port | Pin | GPIO Function | Voltage rail |
|--------|------|-----|---------------|--------------|
| ADC Channel 0 | P0 | 5 | HW_GPIO_FUNC_ADC | VDD1V8P |
| ADC Channel 1 | P0 | 6 | HW_GPIO_FUNC_ADC | VDD1V8P |

Pins are configured as high-impedance inputs with the ADC mux function while a channel is open, and reconfigured as plain GPIO inputs (no pull) when the channel handle is closed. This is managed automatically by the `ad_gpadc` adapter layer.

---

## Clock & Power Configuration

| Parameter | Setting |
|-----------|---------|
| System clock | 32 MHz XTAL32M |
| AHB divider | div1 (32 MHz bus) |
| APB divider | div1 (32 MHz bus) |
| Low-power clock | 32.768 kHz XTAL |
| Sleep mode | `pm_mode_idle` (CPU halts, peripherals and DMA remain active) |
| Wakeup mode | `pm_sys_wakeup_mode_fast` |

`pm_mode_idle` is the appropriate choice here: the CPU sleeps between task activations while DMA and the GPADC continue running, keeping acquisition throughput high without burning CPU cycles polling.

---

## UART Output Protocol

The firmware prints ASCII text lines over the default UART (configured by the SDK's retarget layer):

```
<mv0>,<mv1>
```

Where `<mv0>` and `<mv1>` are floating-point millivolt values for CH0 and CH1 respectively, printed for every 10th sample pair in each batch. Example:

```
1234.5,987.2
1235.1,986.8
...
*fs=6400
```

The `*fs=<N>` line is emitted once per second and reports the cumulative number of samples acquired by the firmware. This is the ground-truth sample rate counter, useful for verifying that the host Python analyzer is receiving and counting samples at the same rate.

The `PRINT_EVERY` decimation reduces UART load without losing ADC data — all pairs are acquired into the buffers; only the print rate is reduced.

---

## Calibration

Each channel has two calibration coefficients defined at the top of [gpadc_app.c](gpadc_app.c):

```c
#define OFFSET_MV_CH0   0.0f   // Subtract this from raw mV
#define GAIN_CH0        1.0f   // Then multiply by this

#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f
```

These are applied by `correct_mv()`:

```
corrected_mv = (raw_mv - offset) * gain
```

The function also clamps negative results to 0.0 (dead-band for near-zero inputs). To calibrate a channel, measure a known voltage, compare to the reported value, and adjust `OFFSET_MV` and `GAIN` accordingly. The defaults (0 offset, 1.0 gain) pass raw converted values through unchanged.

---

## Build Configurations

Three Eclipse CDT build configurations are provided:

| Configuration | Optimization | Execution target | `dg_configEXEC_MODE` |
|--------------|-------------|------------------|-----------------------|
| DA1470x-00-Debug_RAM | -O0 | SRAM | `NON_VOLATILE_IS_NONE` |
| DA1470x-00-Debug_OQSPI | -O0 | OQSPI flash (cached) | `MODE_IS_CACHED` |
| DA1470x-00-Release_OQSPI | -Os | OQSPI flash (cached) | `MODE_IS_CACHED` |

**RAM** execution is fastest to iterate on (no flash erase/program cycle) and is recommended during development. **OQSPI Release** is the production target.

All three configurations enable the same set of SDK feature flags:

```c
#define dg_configUSE_HW_GPADC        (1)   // GPADC peripheral driver
#define dg_configGPADC_ADAPTER       (1)   // ad_gpadc adapter layer
#define dg_configGPADC_DMA_SUPPORT   (1)   // DMA support in GPADC adapter
#define dg_configUSE_HW_DMA          (1)   // DMA peripheral driver
#define OS_FREERTOS                         // FreeRTOS kernel
#define configTOTAL_HEAP_SIZE        14000  // 14 KB FreeRTOS heap
```

---

## Building the Project

1. Open the project in **SmartSnippets Studio** or **e² Studio** (Eclipse-based IDEs for DA1470x).
2. Ensure the `sdk/` symlinks resolve to a valid DA1470x SDK installation.
3. Select the desired build configuration from the Build Configurations menu.
4. Build (`Ctrl+B`). The Makefile is auto-generated by Eclipse CDT Managed Build.
5. Flash the resulting `.bin` / `.elf` to the target using the SDK's flash programmer or a J-Link.

The linker script generation step (`generate_ldscripts`) runs automatically as a pre-build command and produces `mem.ld` and `sections.ld` for the selected memory layout.

---

## Key Parameters

All commonly-tuned values are collected here for quick reference:

| Symbol | File | Default | Purpose |
|--------|------|---------|---------|
| `BATCH_SIZE` | gpadc_app.c | `64` | (V, I) pairs collected per loop cycle |
| `PRINT_EVERY` | gpadc_app.c | `8` | Print one raw pair every N pairs to UART |
| `OFFSET_MV_CH0/1` | gpadc_app.c | `0.0f` | Per-channel mV offset calibration |
| `GAIN_CH0/1` | gpadc_app.c | `1.0f` | Per-channel gain calibration |
| `ZC_HYST_MV` | gpadc_app.c | `50` | Hysteresis band (mV) for zero-crossing detector |
| `ZC_CYCLES_AVG` | gpadc_app.c | `5` | Complete cycles averaged per frequency update |
| `CPU_CLOCK_HZ` | gpadc_app.c | `32000000` | Must match sysclk_XTAL32M in main.c |
| `irq_nr_of_trans` | platform_devices.c | `0` | DMA midpoint IRQ disabled; terminal-count only |
| `HW_GPADC_OVERSAMPLING_4_SAMPLES` | platform_devices.c | 4× | Oversampling ratio |
| `HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6` | platform_devices.c | 3.6 V | Input attenuator range |
| `configTOTAL_HEAP_SIZE` | custom_config_*.h | `14000` | FreeRTOS heap in bytes |
