/**
 ****************************************************************************************
 *
 * @file gpadc_app.c
 *
 * @brief GPADC application logic — DA14706
 *        Interleaved single-sample acquisition (CH0=current, CH1=voltage).
 *
 *        • DWT cycle counter — sub-microsecond timestamps, no OS overhead.
 *        • Startup skew print (*skew=) — exact CH0→CH1 inter-sample delay.
 *        • *fs_acq / *us_pair — true ADC throughput from DWT timestamps,
 *          independent of printf / UART overhead.
 *        • *Vrms / *Irms — 1-second windowed AC RMS (mean-subtracted per batch).
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "gpadc_app.h"

/* If CoreDebug / DWT are not visible, add:  #include "hw_cpm.h"  */

/* ── Channel aliases ─────────────────────────────────────────────────────── */
#define CHAN0_DEVICE    ADC_CH0_DEVICE   /* current channel */
#define CHAN1_DEVICE    ADC_CH1_DEVICE   /* voltage channel */

/* ── Acquisition ─────────────────────────────────────────────────────────── */
#define BATCH_SIZE      64
// #define PRINT_EVERY      2

/* ── Calibration ─────────────────────────────────────────────────────────── */
#define OFFSET_MV_CH0   0.0f
#define GAIN_CH0        1.0f
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f

/* ── RMS scaling ─────────────────────────────────────────────────────────── *
 * K_V  : mains Volts per ADC millivolt   (V/V)  — derived from transformer  *
 *         ratio and signal conditioning attenuation.  *
 * K_I  : Hall sensor mV per ADC millivolt (mV/mV) — signal conditioning gain.*
 * HALL_SENSITIVITY_MV_PER_A : Hall sensor output sensitivity from datasheet.  *
 * ─────────────────────────────────────────────────────────────────────────  */
#define K_V                       (289.269f)    /* PowerAnalyzer / MCU [V/V]    */
#define K_I                       (1.298f)      /* PowerAnalyzer / MCU [mV/mV] */
#define HALL_SENSITIVITY_MV_PER_A (80.0f)       /* [mV/A] — 80 mV @ 1 A */

/* ── Output scaling ──────────────────────────────────────────────────────── *
 * Fixed-point representation shared by printf and future BLE payload.        *
 * V_RMS_SCALE 100  → centivolts  (230.45 V  = 23045)  fits int16_t          *
 * I_RMS_SCALE 1000 → milliamps   (  1.500 A =  1500)  fits int16_t up to    *
 *                                  32.767 A — use int32_t if range exceeded. *
 * ─────────────────────────────────────────────────────────────────────────  */
#define V_RMS_SCALE  100
#define I_RMS_SCALE  1000

/* ── CPU clock ───────────────────────────────────────────────────────────── *
 * DWT->CYCCNT increments every CPU clock cycle.                               *
 * Must match sysclk_XTAL32M configured in main.c.                            *
 * ─────────────────────────────────────────────────────────────────────────  */
#define CPU_CLOCK_HZ    32000000UL

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_init(void) {}

static float correct_mv(uint32_t mv_raw, float offset, float gain)
{
        if ((float)mv_raw < offset) return 0.0f;
        return ((float)mv_raw - offset) / gain;
}

/*
 * Compute the AC RMS of a raw ADC buffer, returned in millivolts.
 *
 * Two-pass mean-subtraction:
 *   pass 1 — compute the batch mean (= DC offset from signal conditioning)
 *   pass 2 — compute sqrt( (1/n) * sum( (x[i] - mean)^2 ) )
 *
 * Squaring small residuals (x - mean) instead of raw mV values keeps
 * float32 precision well within range even when DC >> AC amplitude.
 *
 * The function is intentionally isolated from the window/accumulation logic:
 * when ZCD is added later, only the caller changes — not this function.
 */
static float compute_ac_rms_mv(const uint16_t *raw, int n,
                                const ad_gpadc_driver_conf_t *drv,
                                float offset_mv, float gain)
{
        float mv[BATCH_SIZE];

        /* Pass 1: convert to mV and accumulate sum for mean */
        float sum = 0.0f;
        for (int i = 0; i < n; i++) {
                mv[i] = correct_mv(
                        (uint32_t)ad_gpadc_conv_to_mvolt(drv, raw[i]),
                        offset_mv, gain);
                sum += mv[i];
        }
        const float mean = sum / (float)n;

        /* Pass 2: sum of squared residuals around the mean */
        float sum_sq = 0.0f;
        for (int i = 0; i < n; i++) {
                const float r = mv[i] - mean;
                sum_sq += r * r;
        }

        return sqrtf(sum_sq / (float)n);
}

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_task(void *pvParameters)
{
        static uint16_t   raw0[BATCH_SIZE];
        static uint16_t   raw1[BATCH_SIZE];
        static uint32_t   acq_cycles_accum  = 0;
        static uint32_t   batches_in_window = 0;
        static TickType_t t_last            = 0;
        static float      rms2_accum_ch0    = 0.0f;  /* sum of per-batch variance, CH0 */
        static float      rms2_accum_ch1    = 0.0f;  /* sum of per-batch variance, CH1 */

        /* ── Enable DWT cycle counter ────────────────────────────────────── *
         * DEMCR.TRCENA gates all DWT/ITM/ETM units — must be set first.       *
         * CTRL.CYCCNTENA starts CYCCNT.                                        *
         * ──────────────────────────────────────────────────────────────────  */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT       = 0;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

        /* ── Warmup + inter-channel skew measurement ─────────────────────── *
         * We read one dummy sample from each channel back-to-back and          *
         * timestamp immediately after each read_nof_conv returns.              *
         *                                                                      *
         * t_skew = t_ch1_done - t_ch0_done                                    *
         *        = close_CH0 + open_CH1 + CH1_conversion_time                 *
         *                                                                      *
         * This is the fixed delay between every raw0[i] and raw1[i] pair.     *
         * Write down this value — it is the phase-correction constant T_SKEW  *
         * needed in Step 3 (PF calculation).                                   *
         * ──────────────────────────────────────────────────────────────────  */
        {
                uint16_t dummy      = 0;
                uint32_t t_ch0_done = 0;
                uint32_t t_ch1_done = 0;
                bool     both_ok    = false;

                ad_gpadc_handle_t h = ad_gpadc_open(CHAN0_DEVICE);
                if (h) {
                        ad_gpadc_read_nof_conv(h, 1, &dummy);
                        t_ch0_done = DWT->CYCCNT;   /* CH0 sample just arrived */
                        ad_gpadc_close(h, false);

                        h = ad_gpadc_open(CHAN1_DEVICE);
                        if (h) {
                                ad_gpadc_read_nof_conv(h, 1, &dummy);
                                t_ch1_done = DWT->CYCCNT; /* CH1 sample just arrived */
                                ad_gpadc_close(h, false);
                                both_ok = true;
                        }
                }

                if (both_ok) {
                        uint32_t skew_cy = t_ch1_done - t_ch0_done;
                        uint32_t skew_us = skew_cy / (CPU_CLOCK_HZ / 1000000UL);
                        printf("*skew=%u cycles (~%u us)\n",
                               (unsigned)skew_cy, (unsigned)skew_us);
                }
        }

        /* ── Main loop ───────────────────────────────────────────────────── */
        for (;;) {

                /* ── 1. Interleaved acquisition ──────────────────────────── */
                uint32_t t_acq_start   = DWT->CYCCNT;
                // uint32_t t_ch0_done    = 0;
                // uint32_t t_ch1_done    = 0;
                // uint32_t skew_first_cy = 0;   /* skew at i=0            */
                // uint32_t skew_mid_cy   = 0;   /* skew at i=BATCH_SIZE/2 */
                // uint32_t skew_last_cy  = 0;   /* skew at i=BATCH_SIZE-1 */

                for (int i = 0; i < BATCH_SIZE; i++) {
                        ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
                        if (h0) {
                                ad_gpadc_read_nof_conv(h0, 1, &raw0[i]);
                                // t_ch0_done = DWT->CYCCNT;   /* CH0 sample just arrived */
                                ad_gpadc_close(h0, false);
                        } else {
                                raw0[i] = 0;
                        }
                        ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                        if (h1) {
                                ad_gpadc_read_nof_conv(h1, 1, &raw1[i]);
                                // t_ch1_done = DWT->CYCCNT;   /* CH1 sample just arrived */
                                ad_gpadc_close(h1, false);
                        } else {
                                raw1[i] = 0;
                        }

                        // if      (i == 0)              skew_first_cy = t_ch1_done - t_ch0_done;
                        // else if (i == BATCH_SIZE / 2) skew_mid_cy   = t_ch1_done - t_ch0_done;
                        // else if (i == BATCH_SIZE - 1) skew_last_cy  = t_ch1_done - t_ch0_done;
                }

                acq_cycles_accum  += DWT->CYCCNT - t_acq_start;   // Measures the acquisition time for [ch0-ch1] x 64samples -> total od 128samples
                batches_in_window++;                              // variable that meaasures the completed batches

                /* ── 2. Per-batch RMS accumulation ──────────────────────── *
                 * compute_ac_rms_mv() returns the AC RMS of this batch in mV.*
                 * We accumulate rms² (= variance) across batches so that at  *
                 * the 1-second boundary we can recover the windowed RMS as:  *
                 *   rms_window = sqrt( sum(rms²) / batches )                 *
                 * ──────────────────────────────────────────────────────────  */
                {
                        float rms_mv0 = compute_ac_rms_mv(raw0, BATCH_SIZE,
                                                           CHAN0_DEVICE->drv,
                                                           OFFSET_MV_CH0, GAIN_CH0);
                        float rms_mv1 = compute_ac_rms_mv(raw1, BATCH_SIZE,
                                                           CHAN1_DEVICE->drv,
                                                           OFFSET_MV_CH1, GAIN_CH1);
                        rms2_accum_ch0 += rms_mv0 * rms_mv0;
                        rms2_accum_ch1 += rms_mv1 * rms_mv1;
                }

                /* ── 3. Per-sample print ─────────────────────────────────── */
#if 0
                for (int i = 0; i < BATCH_SIZE; i++) {
                        int mv0_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN0_DEVICE->drv, raw0[i]),
                                OFFSET_MV_CH0, GAIN_CH0);
                        int mv1_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN1_DEVICE->drv, raw1[i]),
                                OFFSET_MV_CH1, GAIN_CH1);

                        if (i % PRINT_EVERY == 0) {
                                printf("%d,%d\n", mv0_val, mv1_val);
                        }
                }
#endif

                /* ── 4. Diagnostics (once per second) ────────────────────── *
                 * fs_acq  — pairs/s derived purely from DWT cycle count of    *
                 *           the acquisition loop. Printf and all other task    *
                 *           overhead are excluded. Multiply by 2 for total     *
                 *           samples/s across both channels.                    *
                 *                                                              *
                 * us_pair — average µs per (CH0 + CH1) pair: open/read/close  *
                 *           for CH0, then open/read/close for CH1. Stable      *
                 *           value reveals true ADC conversion + adapter cost.  *
                 *                                                              *
                 * Vrms / Irms — windowed AC RMS over all batches in this 1 s. *
                 * ──────────────────────────────────────────────────────────  */
                TickType_t now = xTaskGetTickCount();
                if ((now - t_last) >= pdMS_TO_TICKS(1000)) {
                        if (acq_cycles_accum > 0 && batches_in_window > 0) {
                                uint32_t total_pairs = batches_in_window * (uint32_t)BATCH_SIZE;

                                /* fs_acq = total_samples / t_acq_seconds
                                 *        = (total_pairs * 2) / (acq_cycles / CPU_CLOCK_HZ)
                                 *        = (total_pairs * 2 * CPU_CLOCK_HZ) / acq_cycles
                                 * uint64_t cast prevents overflow before the division. */
                                uint32_t fs_acq  = (uint32_t)(
                                        (uint64_t)total_pairs * 2UL * CPU_CLOCK_HZ
                                        / acq_cycles_accum);

                                /* us_pair = t_acq_seconds / total_pairs * 1e6
                                 *         = (acq_cycles / CPU_CLOCK_HZ) / total_pairs * 1e6
                                 *         = acq_cycles / (total_pairs * (CPU_CLOCK_HZ / 1e6))
                                 * CPU_CLOCK_HZ / 1000000 = 32 cycles/µs at XTAL32M. */
                                uint32_t us_pair = acq_cycles_accum
                                        / (total_pairs * (CPU_CLOCK_HZ / 1000000UL));
                                printf("*fs_acq=%u  *us_pair=%u\n",
                                       (unsigned)fs_acq, (unsigned)us_pair);
                                // printf("*loop_skew  first=%u  mid=%u  last=%u us\n",
                                //        (unsigned)(skew_first_cy / (CPU_CLOCK_HZ / 1000000UL)),
                                //        (unsigned)(skew_mid_cy   / (CPU_CLOCK_HZ / 1000000UL)),
                                //        (unsigned)(skew_last_cy  / (CPU_CLOCK_HZ / 1000000UL)));

                                /* Windowed RMS: sqrt( mean_of_batch_variances ) */
                                float rms_mv0_w = sqrtf(rms2_accum_ch0 / (float)batches_in_window);
                                float rms_mv1_w = sqrtf(rms2_accum_ch1 / (float)batches_in_window);

                                float v_rms = K_V * rms_mv1_w / 1000.0f;                    /* V  */
                                float i_rms = (K_I * rms_mv0_w) / HALL_SENSITIVITY_MV_PER_A; /* A  */

                                /* Scaled integers — used for both printf and BLE payload */
                                int32_t v_rms_cV = (int32_t)(v_rms * V_RMS_SCALE);   /* centivolts */
                                int32_t i_rms_mA = (int32_t)(i_rms * I_RMS_SCALE);   /* milliamps  */

                                printf("*Vrms=%"PRId32".%02"PRId32" V  *Irms=%"PRId32".%03"PRId32" A\n",
                                       v_rms_cV / 100,  v_rms_cV % 100,
                                       i_rms_mA / 1000, i_rms_mA % 1000);
                        }
                        acq_cycles_accum  = 0;
                        batches_in_window = 0;
                        rms2_accum_ch0    = 0.0f;
                        rms2_accum_ch1    = 0.0f;
                        t_last            = now;
                }

        } /* end for(;;) */
}
