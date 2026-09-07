/*
 * P3 Session 1 reference: DMA-driven ADC ping-pong, no filtering yet.
 *
 * TPM0 overflows at 10 kHz and triggers the ADC directly in hardware
 * (Section 7 of the manual). Each conversion result is moved from the ADC
 * result register into RAM by DMA, with no CPU involvement at all. Every
 * time one buffer fills, DMA0_IRQHandler immediately repoints DMA at the
 * other buffer (so acquisition never stops) and hands the full buffer to
 * main() to look at. main() just dumps the first few raw samples of each
 * completed buffer over UART, to prove the pipeline is really running.
 *
 * Reference only. Build the DMA/ADC/TPM configuration yourself first;
 * open this only if stuck.
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

static uint32_t g_bufferA[BUFFER_SAMPLES];
static uint32_t g_bufferB[BUFFER_SAMPLES];
static uint32_t *const g_pingPongBuffers[2] = {g_bufferA, g_bufferB};

static dma_handle_t g_dmaHandle;
static dma_transfer_config_t g_transferConfig;

static volatile uint8_t g_fillIndex = 0U;  /* which buffer DMA is currently writing into */
static volatile int32_t g_readyIndex = -1; /* which buffer main() should look at; -1 = none waiting */
static volatile uint32_t g_buffersCaptured = 0U;

/*
 * ADC0TRGSEL = 0b1000 selects TPM0 overflow as the ADC's hardware trigger
 * source; ADC0ALTTRGEN routes SOPT7's selection into the ADC instead of
 * the chip's default PDB-based trigger. Verified against the chip's own
 * register description (MKL26Z4.xml, SOPT7.ADC0TRGSEL enumeration), not
 * guessed. See the manual, Section 7.
 */
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

    /* Both of these are what let DMA, not software, drive every conversion:
       hardware trigger means the ADC starts on its own on each TPM
       overflow, and EnableDMA means each finished conversion raises a DMA
       request instead of (or in addition to) a CPU interrupt. */
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
    /* Overflow fires once every (MOD + 1) clock ticks; no channel or pin
       is configured here, this timer never drives an output. Only its
       internal overflow event is used, routed to the ADC through
       ConfigureAdcHardwareTrigger() above. */
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
    DMA_EnableCycleSteal(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true); /* one sample per ADC request, not the whole buffer at once */
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

    /* Ping-pong, the actual point of this project: hand the buffer that
       just finished to main(), and immediately repoint DMA at the OTHER
       buffer so sampling never stops while main() is still looking at this
       one. This chip's DMA has no built-in scatter-gather (unlike the
       eDMA on larger Kinetis parts), so the alternation is done here in
       software, not by a hardware descriptor chain. See the manual,
       Section 6. */
    g_fillIndex = 1U - g_fillIndex;
    StartDmaIntoBuffer(g_fillIndex);

    g_readyIndex = (int32_t)justFilled;
    g_buffersCaptured++;
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n=== P3 DMA ping-pong reference (Session 1: raw dump only) ===\r\n");
    PRINTF("Sampling ADC0_SE23 (PTE30) at %u Hz, %u samples per buffer.\r\n\r\n", SAMPLE_RATE_HZ, BUFFER_SAMPLES);

    ConfigureAdc();
    ConfigureAdcHardwareTrigger();
    ConfigureDma();
    ConfigureTpmTrigger(); /* start sampling last, once every downstream piece is ready */

    for (;;)
    {
        if (g_readyIndex >= 0)
        {
            int32_t index = g_readyIndex;
            uint32_t i;
            uint32_t *buf;

            g_readyIndex = -1;
            buf = g_pingPongBuffers[index];

            PRINTF("buffer %d ready (capture #%u), first 8 raw samples: ", index, g_buffersCaptured);
            for (i = 0U; i < 8U; i++)
            {
                PRINTF("%u ", buf[i]);
            }
            PRINTF("\r\n");
        }
    }
}
