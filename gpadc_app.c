/**
 ****************************************************************************************
 *
 * @file gpadc_app.c
 *
 * @brief GPADC application logic — DA14706
 *        Reads two ADC channels (CH0: P0_5, CH1: P0_6) via DMA in batches,
 *        converts raw results to millivolts, and streams them over UART as
 *        comma-separated pairs for the uart_analyzer.py tool.
 *
 *        Architecture
 *        ─────────────
 *        The adapter (ad_gpadc) uses a single shared hardware mutex for the
 *        one GPADC block. Opening two handles simultaneously would deadlock,
 *        so the task follows the open → read → close pattern per channel,
 *        per batch. With BATCH_SIZE = 64 the open/close overhead is paid
 *        once per 64 DMA-driven samples — negligible.
 *
 *        Sample rate diagnostic
 *        ──────────────────────
 *        Every second the task emits a line starting with '*' which
 *        uart_analyzer.py ignores (per its parse_line() filter). This lets
 *        you verify the firmware-side sample rate independently of the
 *        Python-side estimate, which is subject to UART buffering jitter.
 *        The reported count is the total number of individual ADC samples
 *        delivered (both channels combined) in the last second.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "gpadc_app.h"

/* ── Aliases ─────────────────────────────────────────────────────────────── */
#define CHAN0_DEVICE    ADC_CH0_DEVICE
#define CHAN1_DEVICE    ADC_CH1_DEVICE

/* ── Batch size ──────────────────────────────────────────────────────────── *
 * Number of DMA-transferred samples collected per channel per loop cycle.   *
 * At ~5 kHz per channel, 64 samples = ~12.8 ms of acquisition per channel, *
 * so one full loop (ch0 + ch1) completes in ~25.6 ms.                       *
 *                                                                            *
 * This value is independent of platform_devices.c — the DMA transfer length *
 * is set at runtime by nof_conv passed to ad_gpadc_read_nof_conv().         *
 * ─────────────────────────────────────────────────────────────────────────  */
#define BATCH_SIZE      64

/* ── Calibration coefficients ────────────────────────────────────────────── *
 * Offset (mV) and gain are applied after the adapter's mV conversion.       *
 * Set both to identity (0.0 / 1.0) until empirical calibration is done.     *
 * ─────────────────────────────────────────────────────────────────────────  */
#define OFFSET_MV_CH0   0.0f
#define GAIN_CH0        1.0f
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f

/* ── Hardware init ───────────────────────────────────────────────────────── *
 * Centralised in main.c → prvSetupHardware(). Nothing to do here.           *
 * ─────────────────────────────────────────────────────────────────────────  */
void gpadc_app_init(void) {}

/* ── correct_mv ──────────────────────────────────────────────────────────── *
 * Applies a per-channel offset and gain correction to a raw mV reading.     *
 *   offset — dead-band: values below this threshold are clamped to 0 mV.    *
 *   gain   — multiplicative correction for full-scale error.                 *
 *                                                                            *
 * Returns a corrected float mV value.                                        *
 * ─────────────────────────────────────────────────────────────────────────  */
static float correct_mv(uint32_t mv_raw, float offset, float gain)
{
        if ((float)mv_raw < offset) {
                return 0.0f;
        }
        return ((float)mv_raw - offset) / gain;
}

/* ── gpadc_app_task ──────────────────────────────────────────────────────── *
 * Main FreeRTOS task. Never returns.                                         *
 *                                                                            *
 * Loop structure                                                             *
 *   1. Open ch0, collect BATCH_SIZE samples via DMA, close ch0.             *
 *   2. Open ch1, collect BATCH_SIZE samples via DMA, close ch1.             *
 *   3. Convert and print all pairs in one tight printf loop.                 *
 *   4. Update the sample-rate diagnostic counter.                            *
 *                                                                            *
 * Static buffers are placed in retained RAM rather than the task stack to   *
 * avoid stack overflow (BATCH_SIZE × 2 × sizeof(uint16_t) = 256 bytes).    *
 * ─────────────────────────────────────────────────────────────────────────  */
void gpadc_app_task(void *pvParameters)
{
        /* DMA destination buffers — one per channel */
        static uint16_t raw0[BATCH_SIZE];
        static uint16_t raw1[BATCH_SIZE];

        /* Sample-rate diagnostic — counts total individual samples (both channels) */
        static uint32_t  sample_counter = 0;
        static TickType_t t_last        = 0;

        /* ── Warmup reads ────────────────────────────────────────────────── *
         * Allow the sample-and-hold capacitor on each input to settle after  *
         * the ADC mux switches for the first time. One dummy conversion per  *
         * channel is sufficient. The warmup uses nof_conv = 1 with a scalar  *
         * buffer, which is valid because irq_nr_of_trans = 0 in dma_cfg_adc  *
         * (the DMA IRQ fires at the end of the transfer regardless of        *
         * length, so no constraint violation occurs).                         *
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
                        /* Should not happen; log and retry next cycle */
                        printf("[GPADC] open ch0 failed\n");
                        continue;
                }
                int ret0 = ad_gpadc_read_nof_conv(h0, BATCH_SIZE, raw0);
                /* Task is blocked here until the DMA IRQ fires (BATCH_SIZE
                 * transfers complete). The OS scheduler runs other tasks. */
                ad_gpadc_close(h0, false);

                /* ── CH1: open → DMA read → close ───────────────────────── */
                ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                if (!h1) {
                        printf("[GPADC] open ch1 failed\n");
                        continue;
                }
                int ret1 = ad_gpadc_read_nof_conv(h1, BATCH_SIZE, raw1);
                ad_gpadc_close(h1, false);

                /* ── Convert and stream ──────────────────────────────────── *
                 * Only print if both reads succeeded. raw0[i] and raw1[i]    *
                 * are time-aligned only approximately (ch0 batch was          *
                 * collected before ch1 batch), which is acceptable for        *
                 * independent channel monitoring. If strict time-alignment    *
                 * is ever needed, interleaving at the hardware level would    *
                 * require a different acquisition strategy.                   *
                 * ──────────────────────────────────────────────────────────  */
                if (ret0 == AD_GPADC_ERROR_NONE && ret1 == AD_GPADC_ERROR_NONE) {
                        for (int i = 0; i < BATCH_SIZE; i++) {
                                int mv0 = (int)correct_mv(
                                        (uint32_t)ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw0[i]),
                                        OFFSET_MV_CH0, GAIN_CH0);
                                int mv1 = (int)correct_mv(
                                        (uint32_t)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw1[i]),
                                        OFFSET_MV_CH1, GAIN_CH1);
                                // Print only every 4th sample pair — reduces UART load by 4×
                                if (i % 10 == 0) {
                                    printf("%d,%d\n", mv0, mv1);
                                }
                        }
                }

                /* ── Sample-rate diagnostic (once per second) ────────────── *
                 * Count both channels: BATCH_SIZE samples × 2 channels.       *
                 * Output starts with '*' so uart_analyzer.py ignores it.      *
                 * Read the printed value from your terminal to get the         *
                 * firmware-side ground-truth sample rate.                      *
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
