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
 *        • ZC detector on CH1 — Schmitt-trigger on rolling 1000-sample mean.
 *          Frequency from DWT timestamps at last 20 rising crossings:
 *            freq = (N-1) * CPU_HZ / (ts[N-1] - ts[0])
 *          No sample-rate estimation needed — time measured directly in cycles.
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

/* ── CPU clock ───────────────────────────────────────────────────────────── *
 * DWT->CYCCNT increments every CPU clock cycle.                               *
 * Must match sysclk_XTAL32M configured in main.c.                            *
 * ─────────────────────────────────────────────────────────────────────────  */
#define CPU_CLOCK_HZ    32000000UL

/* ── Zero-crossing frequency detection ──────────────────────────────────── *
 *                                                                             *
 *  STATS_WIN   : rolling mean window for DC threshold (samples)              *
 *  CROSS_MAXLEN: DWT timestamp history depth                                 *
 *  HYST_MV     : hysteresis dead-band — must be < AC amplitude               *
 *  FREQ_NOM    : nominal signal freq, used only for the crossing guard       *
 *                                                                             *
 *  Algorithm:                                                                 *
 *   1. threshold  = rolling mean of last STATS_WIN CH1 samples (≈ DC level) *
 *   2. Schmitt-trigger: arm on fall below (mean − HYST), fire on rise above  *
 *   3. At each rising crossing: record DWT->CYCCNT in circular buffer        *
 *   4. freq = (N−1) × CPU_HZ / (ts[N−1] − ts[0])                           *
 *      No sample-rate estimate — time measured directly in cycles.           *
 * ─────────────────────────────────────────────────────────────────────────  */
#define ZC_STATS_WIN     1000
#define ZC_CROSS_MAXLEN    20
#define ZC_HYST_MV         10
#define SIGNAL_FREQ_NOM  50.0f

/* Rolling mean buffer */
static int32_t  zc_stats[ZC_STATS_WIN];
static uint32_t zc_stats_idx   = 0;
static int32_t  zc_stats_sum   = 0;
static uint32_t zc_stats_count = 0;   /* saturates at ZC_STATS_WIN */

/* Crossing timestamp history — DWT cycles at each rising crossing */
static uint32_t zc_cross_cy[ZC_CROSS_MAXLEN];
static int      zc_cross_cnt = 0;

/* Schmitt-trigger state */
static bool     zc_armed        = true;   /* TRUE = below threshold, ready to detect */
static uint32_t zc_last_cross   = 0;      /* zc_sample_count value at last crossing  */
static uint32_t zc_sample_count = 0;      /* total CH1 samples processed             */

/* Published result */
static float    freq_hz = 0.0f;

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_init(void) {}

static float correct_mv(uint32_t mv_raw, float offset, float gain)
{
        if ((float)mv_raw < offset) return 0.0f;
        return ((float)mv_raw - offset) / gain;
}

/*
 * zc_update — call once per CH1 sample.
 *
 * Directly mirrors the per-sample block in Python's update() function:
 *
 *   ch1_stats.append(mv1)
 *   ch1_mean = sum(ch1_stats) / len(ch1_stats)
 *   if cross_armed:
 *       if mv1 >= ch1_mean and sample_count - last_cross > min_cross_gap:
 *           cross_timestamps.append(sample_count)
 *           cross_armed = False
 *           freq = measured_rate_hz / avg_gap
 *   else:
 *       if mv1 < ch1_mean - HYST_MV:
 *           cross_armed = True
 */
static void zc_update(int mv1)
{
        /* ── 1. Rolling mean — mirrors ch1_stats deque(maxlen=1000) ── */
        zc_stats_sum -= zc_stats[zc_stats_idx];
        zc_stats[zc_stats_idx] = (int32_t)mv1;
        zc_stats_sum += (int32_t)mv1;
        zc_stats_idx = (zc_stats_idx + 1) % ZC_STATS_WIN;
        if (zc_stats_count < ZC_STATS_WIN) zc_stats_count++;

        /* Wait until the buffer is full — mirrors Python's implicit warm-up
         * (deque starts empty; len(ch1_stats) < STATS_WIN returns a biased mean) */
        if (zc_stats_count < ZC_STATS_WIN) {
                zc_sample_count++;
                return;
        }

        int ch1_mean = (int)(zc_stats_sum / (int32_t)ZC_STATS_WIN);

        /* ── 2. Guard: ignore crossings closer than half a nominal period ── *
         * CPU_CLOCK_HZ / (SIGNAL_FREQ_NOM * 2) = cycles in one half-period.  *
         * Compared against DWT cycles so no sample-rate estimate is needed.   */
        static uint32_t zc_last_cross_cy = 0;
        uint32_t now_cy        = DWT->CYCCNT;
        uint32_t min_gap_cy    = (uint32_t)((float)CPU_CLOCK_HZ / (SIGNAL_FREQ_NOM * 2.0f));

        /* ── 3. Schmitt-trigger state machine ── */
        if (zc_armed) {
                if ((mv1 >= ch1_mean) &&
                    (now_cy - zc_last_cross_cy > min_gap_cy)) {

                        /* Store DWT timestamp of this crossing */
                        if (zc_cross_cnt < ZC_CROSS_MAXLEN) {
                                zc_cross_cy[zc_cross_cnt++] = now_cy;
                        } else {
                                for (int k = 0; k < ZC_CROSS_MAXLEN - 1; k++)
                                        zc_cross_cy[k] = zc_cross_cy[k + 1];
                                zc_cross_cy[ZC_CROSS_MAXLEN - 1] = now_cy;
                        }

                        zc_last_cross    = zc_sample_count;
                        zc_last_cross_cy = now_cy;
                        zc_armed         = false;

                        /* freq = (N-1 half-periods) * CPU_HZ / total_cycles
                         * Each rising crossing is one full period apart, so N-1
                         * gaps span N-1 full periods.                           */
                        if (zc_cross_cnt >= 2) {
                                uint32_t total_cy = zc_cross_cy[zc_cross_cnt - 1] - zc_cross_cy[0];
                                if (total_cy > 0)
                                        freq_hz = (float)(zc_cross_cnt - 1)
                                                  * (float)CPU_CLOCK_HZ
                                                  / (float)total_cy;
                        }
                }
        } else {
                if (mv1 < ch1_mean - ZC_HYST_MV)
                        zc_armed = true;
        }

        zc_sample_count++;
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
                        t_ch0_done = DWT->CYCCNT;
                        ad_gpadc_close(h, false);

                        h = ad_gpadc_open(CHAN1_DEVICE);
                        if (h) {
                                ad_gpadc_read_nof_conv(h, 1, &dummy);
                                t_ch1_done = DWT->CYCCNT;
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

                /* ── 2. Per-sample: convert, ZC detect, print ────────────── */
                for (int i = 0; i < BATCH_SIZE; i++) {
                        int mv0_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN0_DEVICE->drv, raw0[i]),
                                OFFSET_MV_CH0, GAIN_CH0);
                        int mv1_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(
                                        CHAN1_DEVICE->drv, raw1[i]),
                                OFFSET_MV_CH1, GAIN_CH1);

                        zc_update(mv1_val);   /* single argument — all state is internal */

                        if (i % PRINT_EVERY == 0) {
                                printf("%d,%d\n", mv0_val, mv1_val);
                        }
                }

                /* ── 3. Diagnostics (once per second) ────────────────────── */
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