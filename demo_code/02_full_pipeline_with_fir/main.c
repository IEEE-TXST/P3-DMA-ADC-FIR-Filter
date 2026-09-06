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
#define FIR_TAPS 8U
static const int32_t g_firCoeffsQ15[FIR_TAPS] = {570, 2006, 5445, 8363, 8363, 5445, 2006, 570};

/* Only print every DECIMATION-th sample. At 10 kHz raw, printing every
   sample would need about 120 KB/sec of UART bandwidth; 115200 baud only
   gives about 11.5 KB/sec. Decimating by 16 brings the reported rate down
   to 625 Hz, well within budget with headroom to spare. See the manual,
   Section 11, for the full bandwidth arithmetic. */
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
                PRINTF("%lu,%lu\r\n", raw[i], g_filteredBuffer[i]);
            }
        }
    }
}
