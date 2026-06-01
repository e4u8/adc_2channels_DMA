/**
 ****************************************************************************************
 *
 * @file platform_devices.c
 *
 * @brief Platform Devices
 *
 * Copyright (C) 2015-2022 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "ad_gpadc.h"
#include "hw_dma.h"           // ADD: for HW_DMA_CHANNEL_x / HW_DMA_PRIO_x types
#include "platform_devices.h"

/*
 * PLATFORM PERIPHERALS GPIO CONFIGURATION
 *****************************************************************************************
 */

#if dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC

/* ADC_NOF_CONV removed: irq_nr_of_trans is now 0 (terminal-count only),
 * decoupled from gpadc_app.c batch size. */

/* ---------------------------------------------------------------
 * DMA config � ONE struct shared by both channels.
 * The GPADC block is a single hardware resource; ch0 and ch1
 * are never active simultaneously, so sharing is safe and
 * actually required (there is only one internal gpadc_dma_setup).
 *
 * RULES enforced by read_dma_mode() in hw_gpadc.c:
 *   - channel must be EVEN (hardware constraint on GPADC trigger mux)
 *   - irq_nr_of_trans must be <= nof_conv passed to read_nof_conv()
 *   - circular=true with irq_nr_of_trans>0 is NOT supported (returns false)
 * ---------------------------------------------------------------- */
#if HW_GPADC_DMA_SUPPORT
static gpadc_dma_cfg dma_cfg_adc = {
    .channel         = HW_DMA_CHANNEL_0,  // must be even; change if ch0/1 used elsewhere
    .prio            = HW_DMA_PRIO_2,
    .circular        = false,             // one-shot: DMA stops after nof_conv transfers
    .irq_nr_of_trans = 0,                 // no midpoint IRQ; relies on DMA terminal count only
                                          // 0 satisfies irq_nr_of_trans<=nof_conv for any nof_conv
};
#endif

/* GPADC IO configurations */
const ad_gpadc_io_conf_t io_conf_ch0 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_5,
                .on   = {HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true},
                .off  = {HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false}
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P
};

const ad_gpadc_io_conf_t io_conf_ch1 = {
        .input0 = {
                .port = HW_GPIO_PORT_0,
                .pin  = HW_GPIO_PIN_6,
                .on   = {HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_ADC, true},
                .off  = {HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO, false}
        },
        .input1 = {
                .port = HW_GPIO_PORT_NONE,
                .pin  = HW_GPIO_PIN_NONE,
        },
        .voltage_level = HW_GPIO_POWER_VDD1V8P
};

/* GPADC driver configurations */
const ad_gpadc_driver_conf_t drv_conf_ch0 = {
        .input_mode             = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive               = HW_GPADC_INP_P0_5,
        .temp_sensor            = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time            = 4,      /* multiplier x 8 x ADC_CLK */
        .continuous             = true,
        .interval               = 0,      /* unused if continuous = false */
        .input_attenuator       = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping               = true,
        .oversampling           = HW_GPADC_OVERSAMPLING_4_SAMPLES,
        .result_mode            = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup              = &dma_cfg_adc,
#endif
};

const ad_gpadc_driver_conf_t drv_conf_ch1 = {
        .input_mode             = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .positive               = HW_GPADC_INP_P0_6,
        .temp_sensor            = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time            = 4,      /* multiplier x 8 x ADC_CLK */
        .continuous             = true,
        .interval               = 0,      /* unused if continuous = false */
        .input_attenuator       = HW_GPADC_INPUT_VOLTAGE_UP_TO_3V6,
        .chopping               = true,
        .oversampling           = HW_GPADC_OVERSAMPLING_4_SAMPLES,
        .result_mode            = HW_GPADC_RESULT_NORMAL,
#if HW_GPADC_DMA_SUPPORT
        .dma_setup              = &dma_cfg_adc,
#endif
};

/*  External device/module configurations */
const ad_gpadc_controller_conf_t conf_ch0 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch0,
        .drv = &drv_conf_ch0
};

const ad_gpadc_controller_conf_t conf_ch1 = {
        .id  = HW_GPADC_1,
        .io  = &io_conf_ch1,
        .drv = &drv_conf_ch1
};

gpadc_device ADC_CH0_DEVICE = &conf_ch0;
gpadc_device ADC_CH1_DEVICE = &conf_ch1;

#endif /* dg_configGPADC_ADAPTER || dg_configUSE_HW_GPADC */
