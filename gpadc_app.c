/**
 ****************************************************************************************
 *
 * @file gpadc_app.c
 *
 * @brief GPADC application logic — DA14706  (Step 2)
 *        Step 1: interleaved single-sample acquisition (CH0=current, CH1=voltage).
 *        Step 2: zero-crossing detection on CH1 → frequency measurement.
 *
 *        New in Step 2
 *        ─────────────
 *        • DWT cycle counter — sub-microsecond timestamps, no OS overhead.
 *        • Startup skew print (*skew=) — exact CH0→CH1 inter-sample delay.
 *        • ZC detector on CH1 — hysteresis state machine on the batch DC mean.
 *        • Frequency update every ZC_CYCLES_AVG complete cycles (*freq= line).
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
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
#define PRINT_EVERY      2

/* ── Calibration ─────────────────────────────────────────────────────────── */
#define OFFSET_MV_CH0   0.0f
#define GAIN_CH0        1.0f
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f

/* ── Zero-crossing tuning ────────────────────────────────────────────────── *
 * ZC_HYST_MV   — dead-band around the threshold.                             *
 *   Signal must exceed (threshold + HYST) for an upward crossing, and drop   *
 *   below (threshold - HYST) before the next crossing can be detected.        *
 *   Increase if *freq= is unstable; decrease if crossings are missed.         *
 *                                                                             *
 * ZC_CYCLES_AVG — number of complete cycles per frequency update.             *
 *   Frequency error ≈ ±1 sample / (ZC_CYCLES_AVG × samples_per_cycle).       *
 *   At 50 Hz / ~1800 sps: 5 cycles → ±0.28 Hz error, 100 ms update rate.    *
 * ─────────────────────────────────────────────────────────────────────────  */
#define ZC_HYST_MV      50
#define ZC_CYCLES_AVG    5

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

/* ── Zero-crossing state ─────────────────────────────────────────────────── *
 * File-scope so zc_update() can access it without passing pointers.           *
 * All fields are zero-initialised by the C runtime (static storage).          *
 *                                                                             *
 * State diagram:                                                              *
 *                                                                             *
 *   BELOW ──(mv > threshold + HYST)──► ABOVE   ← upward crossing event      *
 *   ABOVE ──(mv < threshold − HYST)──► BELOW                                 *
 *                                                                             *
 * On every upward crossing:                                                   *
 *   • First ever   → anchor the measurement window (save t_window_start).    *
 *   • Subsequent   → increment zc_cycle_cnt.                                  *
 *                    When zc_cycle_cnt == ZC_CYCLES_AVG:                      *
 *                      freq = ZC_CYCLES_AVG × CPU_CLOCK_HZ / elapsed_cycles   *
 *                      reset window to this crossing.                          *
 * ─────────────────────────────────────────────────────────────────────────  */
static bool     zc_above        = false;
static bool     zc_first_seen   = false;
static uint32_t zc_sample_start = 0;   /* global CH1 sample index at window-open ZC  */
static uint32_t zc_global_idx   = 0;   /* total CH1 samples acquired across all batches */
static int      zc_cycle_cnt    = 0;
static float    freq_hz         = 0.0f;   /* 0 until first measurement */

/*
 * zc_update — call once per voltage sample.
 *
 * mv              mV value of the voltage sample
 * threshold_mv    DC offset estimate (batch mean) used as crossing baseline
 * sample_time_cy  estimated DWT timestamp when this sample was acquired
 */
/*
 * zc_update — call once per CH1 sample.
 *
 * mv             mV value of the voltage sample
 * threshold_mv   batch mean used as crossing baseline
 * dt_sample_cy   within-batch inter-sample time in DWT cycles
 *                (= (t_acq_end - t_acq_start) / BATCH_SIZE for the current batch)
 *
 * Why sample counting instead of DWT timestamps:
 *   t_acq_start includes the previous batch's UART+processing time (~29 ms).
 *   If the ZC window straddles batch boundaries that gap inflates elapsed_cy and
 *   makes the measured frequency too low (e.g. 31 Hz instead of 50 Hz).
 *   Counting acquired samples excludes inter-batch idle time entirely.
 *
 *   freq = ZC_CYCLES_AVG × f_acq / elapsed_samples
 *   where f_acq = CPU_CLOCK_HZ / dt_sample_cy  (within-batch acquisition rate)
 */
static void zc_update(int mv, int threshold_mv, uint32_t dt_sample_cy)
{
        if (!zc_above && mv > threshold_mv + ZC_HYST_MV) {

                zc_above = true;

                if (!zc_first_seen) {
                        zc_sample_start = zc_global_idx;
                        zc_first_seen   = true;
                } else {
                        zc_cycle_cnt++;

                        if (zc_cycle_cnt >= ZC_CYCLES_AVG) {
                                uint32_t elapsed_samples = zc_global_idx - zc_sample_start;
                                if (elapsed_samples > 0 && dt_sample_cy > 0) {
                                        freq_hz = (float)ZC_CYCLES_AVG
                                                  * ((float)CPU_CLOCK_HZ / (float)dt_sample_cy)
                                                  / (float)elapsed_samples;
                                }
                                zc_sample_start = zc_global_idx;
                                zc_cycle_cnt    = 0;
                        }
                }

        } else if (zc_above && mv < threshold_mv - ZC_HYST_MV) {
                zc_above = false;
        }
}

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_task(void *pvParameters)
{
        static uint16_t   raw0[BATCH_SIZE];
        static uint16_t   raw1[BATCH_SIZE];
        static uint32_t   sample_counter = 0;
        static TickType_t t_last         = 0;

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

                /* ── 1. Interleaved acquisition ──────────────────────────── *
                 * Timestamp the batch start and end so we can estimate each    *
                 * sample's acquisition time in step 3.                         *
                 * ──────────────────────────────────────────────────────────  */
                uint32_t t_acq_start = DWT->CYCCNT;

                for (int i = 0; i < BATCH_SIZE; i++) {
                        ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
                        if (h0) {
                                ad_gpadc_read_nof_conv(h0, 1, &raw0[i]);
                                ad_gpadc_close(h0, false);
                        } else {
                                raw0[i] = 0;
                        }
                        ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                        if (h1) {
                                ad_gpadc_read_nof_conv(h1, 1, &raw1[i]);
                                ad_gpadc_close(h1, false);
                        } else {
                                raw1[i] = 0;
                        }
                }

                uint32_t t_acq_end = DWT->CYCCNT;

                /* ── 2. Compute ZC threshold from batch mean ─────────────── *
                 * The voltage signal is DC-biased; "zero crossing" means       *
                 * crossing its own mean, not 0 V.                              *
                 *                                                              *
                 * Since ad_gpadc_conv_to_mvolt is linear:                     *
                 *   convert(mean(raw)) == mean(convert(raw))                   *
                 * so summing raw integers (fast) then converting once is       *
                 * mathematically identical to converting every sample first.   *
                 * ──────────────────────────────────────────────────────────  */
                uint32_t sum_raw1 = 0;
                for (int i = 0; i < BATCH_SIZE; i++) {
                        sum_raw1 += (uint32_t)raw1[i];
                }
                uint16_t mean_raw1    = (uint16_t)(sum_raw1 / (uint32_t)BATCH_SIZE);
                int      threshold_mv = (int)correct_mv(
                        (uint32_t)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv,
                                                          mean_raw1),
                        OFFSET_MV_CH1, GAIN_CH1);

                /* ── 3. Per-sample: ZC detection + print ─────────────────── *
                 * Estimated acquisition time for sample i:                    *
                 *                                                              *
                 *   t_sample[i] = t_acq_start + i × dt_sample_cy             *
                 *                                                              *
                 * where dt_sample_cy = (t_acq_end - t_acq_start) / BATCH_SIZE *
                 *                                                              *
                 * Assumes uniform spacing between samples — valid because the  *
                 * open/close overhead is nearly constant for every iteration.  *
                 * ──────────────────────────────────────────────────────────  */
                uint32_t dt_sample_cy = (t_acq_end - t_acq_start)
                                        / (uint32_t)BATCH_SIZE;

                for (int i = 0; i < BATCH_SIZE; i++) {
                        int mv0_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN0_DEVICE->drv, raw0[i]),
                                OFFSET_MV_CH0, GAIN_CH0);
                        int mv1_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN1_DEVICE->drv, raw1[i]),
                                OFFSET_MV_CH1, GAIN_CH1);

                        zc_global_idx++;
                        zc_update(mv1_val, threshold_mv, dt_sample_cy);

                        if (i % PRINT_EVERY == 0) {
                                printf("%d,%d\n", mv0_val, mv1_val);
                        }
                }

                /* ── 4. Diagnostics (once per second) ────────────────────── */
                sample_counter += (uint32_t)(BATCH_SIZE * 2);

                TickType_t now = xTaskGetTickCount();
                if ((now - t_last) >= pdMS_TO_TICKS(1000)) {
                        printf("*fs=%u\n", (unsigned)sample_counter);
                        if (freq_hz > 0.0f) {
                                /* newlib-nano has no %f — scale to centihz and print as int */
                                uint32_t fq = (uint32_t)(freq_hz * 100.0f + 0.5f);
                                printf("*freq=%u.%02u\n",
                                       (unsigned)(fq / 100U),
                                       (unsigned)(fq % 100U));
                        }
                        sample_counter = 0;
                        t_last         = now;
                }

        } /* end for(;;) */
}
