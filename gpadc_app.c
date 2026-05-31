/**
 ****************************************************************************************
 *
 * @file gpadc_app.c
 *
 * @brief GPADC application logic — DA14706
 *        Reads two ADC channels via DMA in batches, accumulates sum-of-squares
 *        across multiple batches, and computes AC-RMS, real power P, apparent
 *        power S, power factor PF, and reactive power Q once per RMS window.
 *        Results are sent over UART as a CSV line once per window.
 *
 *        Acquisition architecture
 *        ────────────────────────
 *        The adapter (ad_gpadc) holds a single hardware mutex for the one
 *        GPADC block. Opening two handles simultaneously deadlocks, so the
 *        task uses the open → read → close pattern per channel per batch.
 *        With BATCH_SIZE = 164 the open/close overhead is paid once per
 *        ~one 50 Hz period — negligible at 8192 Hz per channel.
 *
 *        Phase alignment note
 *        ────────────────────
 *        CH0 and CH1 are acquired sequentially. The time offset between the
 *        two datasets equals exactly one batch duration:
 *            offset = BATCH_SIZE / fs = 164 / 8192 ≈ 20 ms = one 50 Hz period
 *        One full period offset is equivalent to zero phase error for a
 *        periodic 50 Hz signal, so power factor calculation is valid as long
 *        as BATCH_SIZE remains at 164 (or any integer multiple of 164).
 *
 *        RMS window
 *        ──────────
 *        PERIODS_PER_RMS batches are accumulated before each RMS computation.
 *        With BATCH_SIZE = 164 and PERIODS_PER_RMS = 50, the window covers
 *        164 × 50 = 8200 samples ≈ 1 second at 8192 Hz. This equals exactly
 *        50 complete 50 Hz periods, giving zero fractional-period RMS error.
 *
 *        Sample-rate diagnostic
 *        ──────────────────────
 *        Every second the task emits "*fs=<n>" which uart_analyzer.py ignores
 *        (lines starting with '*' are filtered by parse_line()). This is the
 *        firmware-side ground truth for sample rate, independent of UART
 *        buffering jitter on the Python side.
 *        Expected value: ~16384 (8192 Hz × 2 channels).
 *
 *        Physical calibration
 *        ─────────────────────
 *        CH0 — LEM HLSR 10P Hall current sensor:
 *            sensitivity = 80 mV/A, calibration gain K_i = 1.1486
 *            I_rms [A] = (AC_RMS_mV × K_i) / 80
 *
 *        CH1 — BEL DPC12 100 voltage transformer:
 *            calibration gain K_v = 267.487
 *            V_rms [V] = (AC_RMS_mV × K_v) / 1000
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "gpadc_app.h"

/* ── Channel aliases ─────────────────────────────────────────────────────── */
#define CHAN0_DEVICE    ADC_CH0_DEVICE   /* CH0: P0_5 — current (Hall sensor) */
#define CHAN1_DEVICE    ADC_CH1_DEVICE   /* CH1: P0_6 — voltage (transformer) */

/* ── Acquisition parameters ─────────────────────────────────────────────── *
 * BATCH_SIZE must equal fs / f_signal = 8192 / 50 = 163.84 → rounded to    *
 * 164 so that one batch ≈ one complete 50 Hz period.                         *
 *                                                                            *
 * PERIODS_PER_RMS controls how many batches are accumulated before each     *
 * RMS result is computed and sent. 50 batches × 164 samples = 8200 samples  *
 * ≈ exactly 50 complete periods of 50 Hz → zero fractional-period error.    *
 * ─────────────────────────────────────────────────────────────────────────  */
#define BATCH_SIZE          164
#define PERIODS_PER_RMS     50

/* ── Physical calibration constants ─────────────────────────────────────── *
 * Calibrated on 21.05.2026 (K_i) and 25.05.2026 (K_v).                     *
 * Update after re-calibration with the HAMEG HM8115-2.                      *
 * ─────────────────────────────────────────────────────────────────────────  */
#define K_I             1.1486f     /* Current sensor gain correction         */
#define SENSITIVITY_I   80.0f      /* LEM HLSR 10P: 80 mV per 1 A            */
#define K_V             267.487f   /* Voltage transformer gain correction     */

/* ── Hardware init ───────────────────────────────────────────────────────── *
 * Centralised in main.c → prvSetupHardware(). Nothing to do here.           *
 * ─────────────────────────────────────────────────────────────────────────  */
void gpadc_app_init(void) {}

/* ── gpadc_app_task ──────────────────────────────────────────────────────── *
 * Main FreeRTOS task. Never returns.                                         *
 *                                                                            *
 * All persistent state is declared static so that it lives in retained RAM  *
 * rather than on the task stack. This prevents stack overflow when           *
 * sqrtf() / fabsf() are called, which consume significant stack internally  *
 * on Cortex-M33.                                                             *
 * ─────────────────────────────────────────────────────────────────────────  */
void gpadc_app_task(void *pvParameters)
{
        /* ── DMA destination buffers (retained RAM) ─────────────────────── */
        static uint16_t raw0[BATCH_SIZE];
        static uint16_t raw1[BATCH_SIZE];

        /* ── RMS / power accumulators (retained RAM) ─────────────────────── *
         * sum_sq0/1   — sum of (x - mean)² for current and voltage           *
         * sum_p       — sum of (i_ac × v_ac) for instantaneous power         *
         * total_sum0/1 — cumulative mV sums used to compute the window mean  *
         * mean0/1     — window mean (DC offset), updated each batch          *
         * batches     — how many batches accumulated in the current window   *
         * first_window_done — discard window that may predate signal connect *
         * ──────────────────────────────────────────────────────────────────  */
        static float    sum_sq0          = 0.0f;
        static float    sum_sq1          = 0.0f;
        static float    sum_p            = 0.0f;
        static float    total_sum0       = 0.0f;
        static float    total_sum1       = 0.0f;
        static float    mean0            = 0.0f;
        static float    mean1            = 0.0f;
        static int      batches          = 0;
        static bool     first_window_done = false;

        /* ── Sample-rate diagnostic (retained RAM) ───────────────────────── */
        static uint32_t   sample_counter = 0;
        static TickType_t t_last         = 0;

        /* ── Warmup reads ────────────────────────────────────────────────── *
         * One dummy conversion per channel lets the sample-and-hold          *
         * capacitor settle after the ADC mux first switches to each input.   *
         * nof_conv = 1 is safe because irq_nr_of_trans = 0 in dma_cfg_adc.  *
         * ──────────────────────────────────────────────────────────────────  */
        uint16_t dummy = 0;

        ad_gpadc_handle_t h_warm = ad_gpadc_open(CHAN0_DEVICE);
        if (h_warm) {
                ad_gpadc_read_nof_conv(h_warm, 1, &dummy);
                ad_gpadc_close(h_warm, false);
        }
        h_warm = ad_gpadc_open(CHAN1_DEVICE);
        if (h_warm) {
                ad_gpadc_read_nof_conv(h_warm, 1, &dummy);
                ad_gpadc_close(h_warm, false);
        }

        /* ── Main acquisition loop ───────────────────────────────────────── */
        for (;;) {

                /* ── CH0: open → DMA read → close ───────────────────────── */
                ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
                if (!h0) {
                        printf("[GPADC] open ch0 failed\n");
                        continue;
                }
                int ret0 = ad_gpadc_read_nof_conv(h0, BATCH_SIZE, raw0);
                ad_gpadc_close(h0, false);
                // TEMPORARY — remove after diagnosis
                if (ret0 != AD_GPADC_ERROR_NONE) {
                    printf("*ch0 err=%d\n", ret0);
                }

                /* ── CH1: open → DMA read → close ───────────────────────── */
                ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                if (!h1) {
                        printf("[GPADC] open ch1 failed\n");
                        continue;
                }

                int ret1 = ad_gpadc_read_nof_conv(h1, BATCH_SIZE, raw1);
                ad_gpadc_close(h1, false);
                // TEMPORARY — remove after diagnosis
                if (ret1 != AD_GPADC_ERROR_NONE) {
                    printf("*ch1 err=%d\n", ret1);
                }

                /* ── Accumulate sum-of-squares for this batch ────────────── *
                 * Skip if either read failed to avoid corrupting totals.     *
                 * ──────────────────────────────────────────────────────────  */
                if (ret0 == AD_GPADC_ERROR_NONE && ret1 == AD_GPADC_ERROR_NONE) {

                        /* Pass 1 — batch sums for mean update.
                         * The cumulative mean tracks DC offset so that pass 2
                         * accumulates true AC variance. */
                        float bsum0 = 0.0f, bsum1 = 0.0f;
                        for (int i = 0; i < BATCH_SIZE; i++) {
                                bsum0 += (float)ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw0[i]);
                                bsum1 += (float)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw1[i]);
                        }

                        total_sum0 += bsum0;
                        total_sum1 += bsum1;
                        mean0 = total_sum0 / (float)((batches + 1) * BATCH_SIZE);
                        mean1 = total_sum1 / (float)((batches + 1) * BATCH_SIZE);

                        /* Pass 2 — accumulate (x - mean)² and i*v. */
                        for (int i = 0; i < BATCH_SIZE; i++) {
                                float v0 = (float)ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw0[i]) - mean0;
                                float v1 = (float)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw1[i]) - mean1;
                                sum_sq0 += v0 * v0;
                                sum_sq1 += v1 * v1;
                                sum_p   += v0 * v1;
                        }

                        batches++;

                        /* ── Compute and output once per RMS window ──────── */
                        if (batches >= PERIODS_PER_RMS) {

                                float n = (float)(PERIODS_PER_RMS * BATCH_SIZE);

                                if (!first_window_done) {
                                        /* Discard — window may contain pre-signal noise */
                                        first_window_done = true;
                                } else {
                                        /* Guard sqrtf against tiny negative rounding errors */
                                        float rms0_mv = sqrtf(sum_sq0 > 0.0f ? sum_sq0 / n : 0.0f);
                                        float rms1_mv = sqrtf(sum_sq1 > 0.0f ? sum_sq1 / n : 0.0f);

                                        float mean_p_mv2 = sum_p / n;

                                        float I_rms = (rms0_mv * K_I) / SENSITIVITY_I;
                                        float V_rms = (rms1_mv * K_V) / 1000.0f;

                                        /* Real power: scale mean instantaneous power
                                         * from mV² to W using both sensor functions. */
                                        float P  = mean_p_mv2 * (K_I / SENSITIVITY_I) * (K_V / 1000.0f);
                                        float S  = V_rms * I_rms;
                                        float PF = (S > 0.001f) ? (P / S) : 0.0f;
                                        float Q  = sqrtf(fabsf(S * S - P * P));

                                        /* Scaled-integer output — avoids float printf
                                         * stack consumption on Cortex-M33 newlib-nano.
                                         * Python side: divide each field by 1000.0     */
                                        printf("%ld,%ld,%ld,%ld,%ld,%ld\n",
                                               (long)(V_rms * 1000),
                                               (long)(I_rms * 1000),
                                               (long)(P     * 1000),
                                               (long)(S     * 1000),
                                               (long)(PF    * 1000),
                                               (long)(Q     * 1000));
                                }

                                /* Reset accumulators — always, even on discard */
                                sum_sq0    = 0.0f;
                                sum_sq1    = 0.0f;
                                sum_p      = 0.0f;
                                total_sum0 = 0.0f;
                                total_sum1 = 0.0f;
                                mean0      = 0.0f;
                                mean1      = 0.0f;
                                batches    = 0;
                        }
                }

                /* ── Sample-rate diagnostic (once per second) ────────────── *
                 * Counts every batch. Expected: *fs=16400 (≈8200 Hz × 2).   *
                 * ──────────────────────────────────────────────────────────  */
                sample_counter += (uint32_t)(BATCH_SIZE * 2);

                TickType_t now = xTaskGetTickCount();
                if ((now - t_last) >= pdMS_TO_TICKS(1000)) {
                        printf("*fs=%lu\n", (unsigned long)sample_counter);
                        sample_counter = 0;
                        t_last = now;
                }

        } /* end for(;;) */
}
