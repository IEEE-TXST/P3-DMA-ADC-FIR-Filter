/*
 * P3 Session 2 reference: the full pipeline. Same DMA/ADC/TPM ping-pong
 * acquisition as 01_dma_adc_ping_pong/, with an 8-tap Q15 fixed-point FIR
 * low-pass filter applied to each completed buffer, then a decimated
 * raw,filtered CSV stream over UART for python/plot_fir_demo.py to plot
 * live. See the manual, Sections 8 to 11, for how the filter coefficients
 * were designed and why the output is decimated before it's printed.
 *
 * Reference only. Get Session 1's ping-pong acquisition working and
 * understood first; open this only if stuck on the filtering or streaming
 * parts specifically.
 */

/*
 * WHAT: Builds on Session 1's DMA ping-pong acquisition by running each
 * completed buffer through an 8-tap low-pass filter, then streaming both
 * the raw and filtered values over UART as CSV for a Python script to plot
 * live.
 *
 * HOW: Acquisition (TPM0 -> ADC -> DMA ping-pong) is identical to Session
 * 1; see 01_dma_adc_ping_pong/main.c for that half. What's new here is
 * ApplyFirFilter, run on the CPU once per completed buffer, and a decimated
 * CSV print loop that sends only every 16th raw/filtered pair instead of
 * every sample.
 *
 * WHY: This is the point of the whole project: DMA handles acquisition
 * with zero CPU involvement, freeing the CPU to do something useful with
 * the data, here, digital filtering, instead of spending its time on the
 * mechanics of sampling. The FIR filter itself uses fixed-point (Q15) math
 * rather than floating point, both because this build's Redlib config
 * can't print floats (PRINTF_FLOAT_ENABLE=0) and because fixed-point
 * integer math is dramatically faster on a Cortex-M0+, which has no
 * hardware floating-point unit.
 */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"
#include "fsl_tpm.h"

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_CHANNEL 23U /* PTE30, same analog header pin P1 used */
#define ADC16_RESULT_REG_ADDR ((uint32_t)(&ADC0->R[0]))

#define DEMO_DMA_BASEADDR DMA0
#define DEMO_DMA_CHANNEL 0U
#define DEMO_DMAMUX_BASEADDR DMAMUX0

#define BUFFER_SAMPLES 64U
#define SAMPLE_RATE_HZ 10000U

/*
 * 8-tap Hamming-windowed-sinc low-pass, cutoff 500 Hz at a 10 kHz sample
 * rate, coefficients scaled to Q15 (32768 = 1.0) and rounded. Computed,
 * not guessed; the manual, Section 9, shows the exact design script and
 * this filter's real measured frequency response (roughly -3.6 dB at
 * 1 kHz, -15 dB at 2 kHz, -35 dB by 3 kHz). Symmetric coefficients mean
 * this is a linear-phase filter: every frequency is delayed by the same
 * amount, so the waveform's shape isn't smeared, only smoothed.
 */
/*
 * WHAT: The 8 fixed filter coefficients (the "taps") that define this
 * filter's exact frequency response.
 * HOW: Each coefficient is a real number between 0 and 1 (the filter's
 * impulse response), scaled by 32768 (2^15) and rounded to the nearest
 * integer, that's what "Q15" means: a fixed-point format where the value 1.0
 * is represented as the integer 32768.
 * WHY: Q15 fixed-point lets this filter run entirely in integer arithmetic
 * (ApplyFirFilter below), which is both faster than floating point on this
 * CPU and avoids the float-printing limitation mentioned above. The
 * coefficients being symmetric (570, 2006, 5445, 8363, 8363, 5445, 2006,
 * 570, a mirror image around the center) is what makes this a linear-phase
 * filter: it delays every frequency component of the signal by the same
 * amount, so the filtered waveform is a smoothed version of the original
 * shape, not a phase-distorted one.
 */
#define FIR_TAPS 8U
static const int32_t g_firCoeffsQ15[FIR_TAPS] = {570, 2006, 5445, 8363, 8363, 5445, 2006, 570};

/* Only print every DECIMATION-th sample. At 10 kHz raw, printing every
   sample would need about 120 KB/sec of UART bandwidth; 115200 baud only
   gives about 11.5 KB/sec. Decimating by 16 brings the reported rate down
   to 625 Hz, well within budget with headroom to spare. See the manual,
   Section 11, for the full bandwidth arithmetic. */
/*
 * WHAT: How many samples to skip between each one actually printed over
 * UART.
 * WHY: The ADC/DMA pipeline captures all 10,000 samples/second regardless;
 * DECIMATION only controls how much of that gets sent over the (much
 * slower) UART link for plotting. This is purely an output bandwidth
 * decision, not a change to the filter or the acquisition rate itself.
 */
#define DECIMATION 16U

static uint32_t g_bufferA[BUFFER_SAMPLES];
static uint32_t g_bufferB[BUFFER_SAMPLES];
static uint32_t *const g_pingPongBuffers[2] = {g_bufferA, g_bufferB};

static dma_handle_t g_dmaHandle;
static dma_transfer_config_t g_transferConfig;

static volatile uint8_t g_fillIndex = 0U;
static volatile int32_t g_readyIndex = -1;

/* Last FIR_TAPS-1 raw samples from the previous buffer, carried forward so
   the filter has real history at every buffer's first few samples instead
   of a glitch at every ping-pong boundary. Zero at startup, which causes
   one brief, expected startup transient on the very first buffer, exactly
   like any real filter the instant after power-on. */
/*
 * WHAT: Holds the last 7 raw samples from whichever buffer was processed
 * most recently, and the buffer that receives this buffer's filtered
 * output.
 * WHY: An 8-tap filter needs the current sample plus its 7 predecessors to
 * compute each output; at the very start of a new buffer, those
 * predecessors live in the PREVIOUS buffer, which may already be getting
 * overwritten by the next DMA transfer. g_firHistory solves this by saving
 * just those 7 needed values before that happens, so filtering is
 * continuous across the ping-pong boundary instead of having a small glitch
 * at the start of every buffer.
 */
static uint32_t g_firHistory[FIR_TAPS - 1U];
static uint32_t g_filteredBuffer[BUFFER_SAMPLES];

static void ConfigureAdcHardwareTrigger(void)
{
    SIM->SOPT7 = (SIM->SOPT7 & ~SIM_SOPT7_ADC0TRGSEL_MASK) | SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN(1);
}

static void ConfigureAdc(void)
{
    adc16_config_t adcConfig;
    adc16_channel_config_t channelConfig;

    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_Resolution16Bit;
    adcConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
    adcConfig.enableContinuousConversion = false;
    ADC16_Init(DEMO_ADC16_BASEADDR, &adcConfig);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR);
#endif

    channelConfig.channelNumber = DEMO_ADC16_CHANNEL;
    channelConfig.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &channelConfig);

    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, true);
    ADC16_EnableDMA(DEMO_ADC16_BASEADDR, true);
}

static void ConfigureTpmTrigger(void)
{
    tpm_config_t tpmConfig;
    uint32_t tpmClockHz = CLOCK_GetFreq(kCLOCK_PllFllSelClk);

    CLOCK_SetTpmClock(1U);
    TPM_GetDefaultConfig(&tpmConfig);
    TPM_Init(TPM0, &tpmConfig);
    TPM_SetTimerPeriod(TPM0, (tpmClockHz / SAMPLE_RATE_HZ) - 1U);
    TPM_StartTimer(TPM0, kTPM_SystemClock);
}

static void StartDmaIntoBuffer(uint8_t bufferIndex)
{
    DMA_PrepareTransfer(&g_transferConfig, (void *)ADC16_RESULT_REG_ADDR, sizeof(uint32_t),
                        (void *)g_pingPongBuffers[bufferIndex], sizeof(uint32_t), BUFFER_SAMPLES * sizeof(uint32_t),
                        kDMA_PeripheralToMemory);
    DMA_SetTransferConfig(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, &g_transferConfig);
    DMA_EnableInterrupts(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL);
    DMA_EnableAsyncRequest(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true);
    DMA_EnableCycleSteal(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true);
    DMA_StartTransfer(&g_dmaHandle);
}

static void ConfigureDma(void)
{
    DMAMUX_Init(DEMO_DMAMUX_BASEADDR);
    DMAMUX_SetSource(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL, kDmaRequestMux0ADC0);
    DMAMUX_EnableChannel(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL);

    DMA_Init(DEMO_DMA_BASEADDR);
    DMA_CreateHandle(&g_dmaHandle, DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL);

    g_fillIndex = 0U;
    StartDmaIntoBuffer(g_fillIndex);
    NVIC_EnableIRQ(DMA0_IRQn);
}

/*
 * WHAT: Identical acquisition handoff logic to Session 1's DMA0_IRQHandler.
 * WHY: Filtering deliberately does NOT happen inside this interrupt
 * handler: it happens in main()'s loop instead (below), after the handler
 * has already re-armed DMA into the other buffer. Keeping this handler
 * fast and simple (re-arm, flag, done) means the more time-consuming FIR
 * computation never delays the next buffer's acquisition or risks missing
 * a DMA completion.
 */
void DMA0_IRQHandler(void)
{
    uint8_t justFilled = g_fillIndex;

    DMA_ClearChannelStatusFlags(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, kDMA_TransactionsDoneFlag);
    g_fillIndex = 1U - g_fillIndex;
    StartDmaIntoBuffer(g_fillIndex);
    g_readyIndex = (int32_t)justFilled;
}

/*
 * y[n] = sum(k = 0..7) h[k] * x[n-k], in Q15: each product is accumulated
 * in a 32-bit accumulator BEFORE the single final right-shift by 15, not
 * shifted after every multiply, which is what keeps rounding error low.
 * x[n-1] .. x[n-7] for the first few samples of a new buffer come from
 * g_firHistory (the tail of the previous buffer), not zeros, so the
 * filter has real signal history at every buffer boundary.
 */
/*
 * WHAT: Computes the filtered value of every sample in rawBuffer, writing
 * results into filteredBuffer, then saves this buffer's tail as history for
 * next time.
 * HOW: For each output sample n, sums 8 products (each tap's coefficient
 * times the corresponding raw sample, counting backward from n) into a
 * 32-bit accumulator, then shifts right by 15 once at the very end to
 * undo the Q15 scaling. When n-k goes negative (meaning "before this
 * buffer started"), the needed sample is pulled from g_firHistory instead
 * of rawBuffer. After processing the whole buffer, the last 7 raw samples
 * are copied into g_firHistory so the NEXT call to this function has
 * correct history too.
 * WHY: Accumulating all 8 products before the single right-shift, rather
 * than shifting after each multiply, is a real fixed-point-math technique:
 * shifting early would throw away precision on every single tap instead of
 * just once at the end, compounding rounding error 8 times over instead of
 * losing it only once. This is exactly the kind of fixed-point detail that
 * makes integer DSP code look deceptively simple but actually embed real
 * numerical-accuracy decisions.
 */
static void ApplyFirFilter(const uint32_t *rawBuffer, uint32_t *filteredBuffer, uint32_t count)
{
    uint32_t n;
    uint32_t k;

    for (n = 0U; n < count; n++)
    {
        int32_t acc = 0;
        for (k = 0U; k < FIR_TAPS; k++)
        {
            int32_t idx = (int32_t)n - (int32_t)k;
            uint32_t sample = (idx >= 0) ? rawBuffer[idx] : g_firHistory[(FIR_TAPS - 1U) + idx];
            acc += g_firCoeffsQ15[k] * (int32_t)sample;
        }
        filteredBuffer[n] = (uint32_t)(acc >> 15);
    }

    for (k = 0U; k < (FIR_TAPS - 1U); k++)
    {
        g_firHistory[k] = rawBuffer[count - (FIR_TAPS - 1U) + k];
    }
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n=== P3 full pipeline reference (Session 2: FIR + Python plot) ===\r\n");
    PRINTF("Sampling ADC0_SE23 (PTE30) at %u Hz, printing every %uth sample as raw,filtered.\r\n",
           SAMPLE_RATE_HZ, DECIMATION);
    PRINTF("Point python/plot_fir_demo.py at this board's serial port to see it live.\r\n\r\n");

    ConfigureAdc();
    ConfigureAdcHardwareTrigger();
    ConfigureDma();
    ConfigureTpmTrigger();

    /*
     * WHAT: Waits for a completed buffer, filters it, then prints a
     * decimated raw,filtered CSV stream.
     * HOW: Same g_readyIndex flag-check pattern as Session 1, but each
     * ready buffer now goes through ApplyFirFilter before printing, and the
     * print loop steps by DECIMATION instead of printing every sample.
     * WHY: Printing "raw,filtered" pairs (not just filtered values alone)
     * is deliberate: it lets python/plot_fir_demo.py draw both signals on
     * the same plot, making the filter's smoothing effect visually obvious
     * by direct comparison, rather than just trusting the numbers.
     */
    for (;;)
    {
        if (g_readyIndex >= 0)
        {
            int32_t index = g_readyIndex;
            uint32_t i;
            uint32_t *raw;

            g_readyIndex = -1;
            raw = g_pingPongBuffers[index];

            ApplyFirFilter(raw, g_filteredBuffer, BUFFER_SAMPLES);

            for (i = 0U; i < BUFFER_SAMPLES; i += DECIMATION)
            {
                PRINTF("%u,%u\r\n", raw[i], g_filteredBuffer[i]);
            }
        }
    }
}
