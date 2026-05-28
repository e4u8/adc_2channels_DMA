/*
 * gpadc_app.c
 *
 *  Created on: Feb 6, 2026
 *      Author: User
 *      Application logic of GPADC
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "gpadc_app.h"

#define CHAN0_DEVICE   ADC_CH0_DEVICE
#define CHAN1_DEVICE   ADC_CH1_DEVICE

#define ADC_NOF_CONV    16    // must match platform_devices.c

/* Gain and Offset error coefficients for each ADC */
//#define OFFSET_MV_CH0    340.0f         // For P05
//#define GAIN_CH0         0.798f
//#define OFFSET_MV_CH1    170.0f         // For P06
//#define GAIN_CH1         0.845f

#define OFFSET_MV_CH0    0.0f         // For P05
#define GAIN_CH0         1.0f
#define OFFSET_MV_CH1    0.0f         // For P06
#define GAIN_CH1         1.0f

/*  The hardware configuration happens at main.c inside prvSetupHardware()
 *  because I chose centralized hardware init.
 *  So, this function will be empty!
 */
void gpadc_app_init(void) {}

/* Calibrates mV measurements of ADC for 3.6V attenuation. For values that are up to
 * OFFLINE_OFFSET_MV_3V6, it outputs 0V, that's because under no input there are
 * "wrong" measurements up to that value. An offset value. Also a GAIN_ERROR_3V6 is
 * used to correct the results. For every attenuation level there is a different
 * GAIN_ERROR_PARAMETER and it is measured empirically.
 * Input:  uncalibrated mV from ad_gpadc_conv_to_mvolt(), offset and gain coeffs for each channel
 * Output: (float) corrected mV value from offset and gain error of the MCU
 */
float correct_mv(uint32_t mv_uncalibrated, float offset, float gain) {
    if ((float)mv_uncalibrated < offset) return 0.0f;
    return ((float)mv_uncalibrated - offset) / gain;
}

/* The GPADC Adapter is used following the pattern of
 *      [  --> Open - Read - Close <--  ]
 * this happens for both channels. printf() happens in batches, not
 * for every conversion and the time for a 100 sample reading period is also printed.
 * NOTE: The driver prevents using one handler for two different channels, [with reconfig()]
 * that's why this methodology was selected.
 */

void gpadc_app_task(void *pvParameters)
{
    static uint16_t raw_buf0[ADC_NOF_CONV];  // DMA writes directly here
    static uint16_t raw_buf1[ADC_NOF_CONV];
    uint16_t dummy = 0;

    /* Warmup dummy reads — keep as-is, nof_conv=1 is fine here */
    ad_gpadc_handle_t h_warm = ad_gpadc_open(CHAN0_DEVICE);
    if (h_warm) { ad_gpadc_read_nof_conv(h_warm, 1, &dummy); ad_gpadc_close(h_warm, false); }
    h_warm = ad_gpadc_open(CHAN1_DEVICE);
    if (h_warm) { ad_gpadc_read_nof_conv(h_warm, 1, &dummy); ad_gpadc_close(h_warm, false); }

    for (;;) {

        /* ---- CH0 ---- */
        ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
        if (!h0) { printf("[GPADC] open ch0 failed\n"); continue; }
        int ret0 = ad_gpadc_read_nof_conv(h0, ADC_NOF_CONV, raw_buf0);
        // DMA fills raw_buf0[] autonomously; task blocks on OS_EVENT until DMA IRQ fires
        ad_gpadc_close(h0, false);

        /* ---- CH1 ---- */
        ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
        if (!h1) { printf("[GPADC] open ch1 failed\n"); continue; }
        int ret1 = ad_gpadc_read_nof_conv(h1, ADC_NOF_CONV, raw_buf1);
        ad_gpadc_close(h1, false);

        if (ret0 == AD_GPADC_ERROR_NONE && ret1 == AD_GPADC_ERROR_NONE) {
            uint32_t sum0 = 0, sum1 = 0;
            for (int i = 0; i < ADC_NOF_CONV; i++) {
                sum0 += raw_buf0[i];
                sum1 += raw_buf1[i];
            }
            uint16_t avg0 = (uint16_t)(sum0 / ADC_NOF_CONV);
            uint16_t avg1 = (uint16_t)(sum1 / ADC_NOF_CONV);

            float mv0 = correct_mv(ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, avg0), OFFSET_MV_CH0, GAIN_CH0);
            float mv1 = correct_mv(ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, avg1), OFFSET_MV_CH1, GAIN_CH1);
            printf("%d,%d\n", (int)mv0, (int)mv1);
        }
    }
}
