/**
 ****************************************************************************************
 *
 * @file platform_devices.c
 *
 * @brief Platform Devices — GPADC configuration for DA14706
 *        Two single-ended channels: CH0 on P0_5, CH1 on P0_6.
 *        DMA mode, one-shot burst, chopping enabled, no oversampling.
 *
 * Copyright (C) 2015-2022 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "ad_gpadc.h"
#include "hw_dma.h"
#include "platform_devices.h"

#if dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC

/* -------------------------------------------------------------------------
 * DMA configuration
 *
 * One shared struct is correct: the GPADC is a single hardware block and
 * ch0 / ch1 are never active at the same time. The driver copies this into
 * its own internal DMA_setup on every hw_gpadc_configure() call (which
 * happens inside ad_gpadc_open()), so sharing is both safe and required.
 *
 * Rules enforced by read_dma_mode() in hw_gpadc.c:
 *   - channel MUST be an even number (GPADC trigger mux hardware constraint)
 *   - irq_nr_of_trans = 0  →  DMA fires its IRQ after the full transfer
 *                              length, automatically. This avoids the
 *                              constraint that irq_nr_of_trans <= nof_conv,
 *                              which would otherwise cause a silent failure
 *                              if nof_conv ever differs from irq_nr_of_trans.
 *   - circular = false with irq_nr_of_trans = 0 is the safest combination.
 * ------------------------------------------------------------------------- */
#if HW_GPADC_DMA_SUPPORT
static gpadc_dma_cfg dma_cfg_adc = {
        .channel         = HW_DMA_CHANNEL_0, /* must be even; change if ch0/1 taken elsewhere */
        .prio            = HW_DMA_PRIO_2,
        .circular        = false,            /* one-shot: DMA stops after nof_conv transfers  */
        .irq_nr_of_trans = 0,               /* 0 → fire IRQ at end of full transfer           */
};
#endif /* HW_GPADC_DMA_SUPPORT */

/* -------------------------------------------------------------------------
 * GPIO / IO configurations
 *
 * input1 is unused (single-ended mode), so port/pin are set to NONE.
 * Pins are driven as ADC inputs while the adapter is open (.on) and
 * released back to high-impedance GPIO when closed (.off).
 * voltage_level must match the supply rail of the signal source.
 * ------------------------------------------------------------------------- */
const ad_gpadc_io_conf_t io_conf_ch0 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_5,
                .on   = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true  },
                .off  = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false },
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P,
};

const ad_gpadc_io_conf_t io_conf_ch1 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_6,
                .on   = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true  },
                .off  = { HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false },
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P,
};

/* -------------------------------------------------------------------------
 * GPADC driver configurations
 *
 * Tuned for ~5 kHz per-channel throughput with chopping kept enabled:
 *
 *   sample_time  = 1  → acquisition window = 1 × 8 = 8 ADC clock cycles
 *   oversampling = 1  → single conversion per result (no hardware averaging)
 *   chopping     = true → two sub-conversions per result (cancels DC offset);
 *                         this halves the raw rate but is kept for accuracy.
 *   continuous   = true → REQUIRED for DMA and for nof_conv > 1; the hardware
 *                          keeps firing conversions back-to-back until the DMA
 *                          transfer completes and the adapter clears the flag.
 *   interval     = 0  → no inter-conversion delay in continuous mode
 *   attenuator   = UP_TO_3V6 → full-scale = 3.6 V (matches your signal range)
 *   result_mode  = NORMAL → 16-bit left-aligned raw result
 *   dma_setup    = &dma_cfg_adc → activates DMA path in hw_gpadc_read()
 *
 * With chopping=true and sample_time=1 the effective conversion time is
 * approximately 2 × 8 = 16 ADC clock cycles. At a 1 MHz ADC clock that
 * gives ~62 kHz raw rate. Sequential ch0+ch1 acquisition halves this to
 * ~31 kHz per channel — well above the 5 kHz target.
 * ------------------------------------------------------------------------- */
const ad_gpadc_driver_conf_t drv_conf_ch0 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_5,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 1,
        .continuous       = true,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_1_SAMPLE,
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = &dma_cfg_adc,
#endif
};

const ad_gpadc_driver_conf_t drv_conf_ch1 = {
        .input_mode       = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive         = HW_GPADC_INP_P0_6,
        .temp_sensor      = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time      = 1,
        .continuous       = true,
        .interval         = 0,
        .input_attenuator = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping         = true,
        .oversampling     = HW_GPADC_OVERSAMPLING_1_SAMPLE,
        .result_mode      = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup        = &dma_cfg_adc,
#endif
};

/* -------------------------------------------------------------------------
 * Controller configurations and exported device handles
 * ------------------------------------------------------------------------- */
const ad_gpadc_controller_conf_t conf_ch0 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch0,
        .drv = &drv_conf_ch0,
};

const ad_gpadc_controller_conf_t conf_ch1 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch1,
        .drv = &drv_conf_ch1,
};

gpadc_device ADC_CH0_DEVICE = &conf_ch0;
gpadc_device ADC_CH1_DEVICE = &conf_ch1;

#endif /* dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC */
