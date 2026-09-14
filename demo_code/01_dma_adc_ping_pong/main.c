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

/*
 * WHAT: Continuously samples one analog pin at 10,000 samples/second, using
 * hardware alone (a timer, the ADC, and DMA) to acquire each sample, with
 * the CPU only touching the data after a whole 64-sample buffer is full.
 *
 * HOW: TPM0 is configured to overflow (reset back to 0) 10,000 times a
 * second, and that overflow event is wired, purely in hardware, to trigger
 * an ADC conversion. Each ADC result then triggers a DMA transfer that
 * copies the result out of the ADC's result register into one of two
 * buffers. When a buffer fills, an interrupt fires: it immediately points
 * DMA at the OTHER buffer (so sampling never has a gap) and flags the
 * just-filled buffer for main() to read.
 *
 * WHY: P1's ADC demo triggered one conversion per keypress, with the CPU
 * blocking until it finished, that doesn't scale to thousands of samples
 * per second. Here, none of the three peripherals involved (TPM, ADC, DMA)
 * ever interrupts the CPU to do the sampling itself; the CPU only gets
 * involved once every 64 samples, to consume a finished buffer. This is
 * the same "configure hardware once, let it run itself" idea as TPM PWM in
 * P1, taken further: an entire acquisition pipeline running with zero CPU
 * polling, freeing the CPU for other work (this project's eventual FIR
 * filtering, in Session 2) while acquisition continues underneath.
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
/*
 * WHAT: The raw memory address of the ADC's conversion-result register.
 * WHY: DMA needs a plain memory address to read from, not an SDK function
 * call; &ADC0->R[0] is that register's location, and DMA will read a fresh
 * value from this exact address once per trigger.
 */
#define ADC16_RESULT_REG_ADDR ((uint32_t)(&ADC0->R[0]))

#define DEMO_DMA_BASEADDR DMA0
#define DEMO_DMA_CHANNEL 0U
#define DEMO_DMAMUX_BASEADDR DMAMUX0

#define BUFFER_SAMPLES 64U
#define SAMPLE_RATE_HZ 10000U

/*
 * WHAT: Two fixed buffers DMA alternates between ("ping-pong"), and a
 * lookup array so code can refer to "the buffer at index 0 or 1" instead of
 * naming A/B directly.
 * WHY: Two separate buffers, not one, is what lets acquisition and
 * consumption happen at the same time: while DMA fills buffer B, main() (or
 * the FIR filter in Session 2) can safely read the already-complete buffer
 * A, with no risk of reading data that's still being written.
 */
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
/*
 * WHAT: Wires TPM0's overflow event directly to the ADC's trigger input, in
 * hardware.
 * HOW: Writes two bitfields in the SIM (System Integration Module)'s SOPT7
 * register: ADC0TRGSEL picks which hardware signal triggers the ADC (8 =
 * TPM0 overflow), and ADC0ALTTRGEN switches the ADC from its default
 * trigger source (a different peripheral called the PDB) to this
 * alternate, SOPT7-selected one.
 * WHY: This is a chip-level routing decision, not something the ADC or TPM
 * driver alone can express, which is why it's a direct register write
 * instead of an SDK function call. Without ADC0ALTTRGEN set, TPM0's
 * overflow would have no effect on the ADC at all, since the ADC would
 * still be listening to the PDB by default.
 */
static void ConfigureAdcHardwareTrigger(void)
{
    SIM->SOPT7 = (SIM->SOPT7 & ~SIM_SOPT7_ADC0TRGSEL_MASK) | SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN(1);
}

/*
 * WHAT: Configures ADC0 to convert on channel 23 (PTE30) at 16-bit
 * resolution, driven by hardware triggers instead of software, with DMA
 * requests enabled.
 * HOW: Same GetDefaultConfig-then-Init pattern as every other peripheral in
 * this series, but with two settings deliberately overridden from default:
 * 16-bit resolution (finer-grained than P1's 12-bit reads) and
 * enableContinuousConversion left false, since each conversion here is
 * meant to be triggered individually by TPM0, not run back-to-back on its
 * own.
 * WHY: ADC16_EnableHardwareTrigger(true) and ADC16_EnableDMA(true) together
 * are what actually make this pipeline hardware-driven: hardware trigger
 * means the ADC starts a new conversion by itself whenever TPM0 overflows,
 * without any CPU or software call; EnableDMA means a finished conversion
 * raises a DMA request instead of needing a CPU interrupt to notice and
 * copy the result out.
 */
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

/*
 * WHAT: Configures TPM0 to overflow (and thus trigger the ADC) exactly
 * 10,000 times per second, driving nothing else.
 * HOW: TPM_SetTimerPeriod sets the counter's reload value (MOD) so it
 * overflows once every tpmClockHz/SAMPLE_RATE_HZ ticks; no PWM channel or
 * output pin is configured, unlike P1's PWM demo, since this timer's only
 * job is to generate a periodic internal event, never to drive a physical
 * pin.
 * WHY: This is the "metronome" for the whole acquisition pipeline: every
 * other piece (the ADC trigger routing in ConfigureAdcHardwareTrigger, and
 * ultimately the DMA transfers) only happens because this timer ticks at a
 * steady, hardware-precise 10kHz, far steadier than any software delay
 * loop could achieve.
 */
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

/*
 * WHAT: Arms one DMA transfer that will copy BUFFER_SAMPLES conversion
 * results, one at a time, from the ADC's result register into the chosen
 * buffer.
 * HOW: DMA_PrepareTransfer describes the transfer (source address, dest
 * address, how many bytes per request, total bytes) as
 * kDMA_PeripheralToMemory; DMA_EnableCycleSteal is the setting that makes
 * DMA move exactly one sample per ADC trigger instead of racing through the
 * whole buffer the instant the first trigger arrives.
 * WHY: Cycle-steal mode is what keeps this transfer synchronized with the
 * real 10kHz sample rate: without it, DMA would copy all 64 samples back to
 * back as fast as the bus allows the moment it saw one DMA request,
 * defeating the entire point of pacing acquisition with TPM0.
 */
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

/*
 * WHAT: One-time DMA/DMAMUX setup, then kicks off the very first transfer
 * into buffer 0.
 * HOW: DMAMUX (DMA Multiplexer) is configured first, since it's the piece
 * that connects a specific peripheral's DMA request (here, ADC0's) to a
 * specific DMA channel; only after that routing exists does starting an
 * actual transfer make sense.
 * WHY: This function's last two lines (StartDmaIntoBuffer + NVIC_EnableIRQ)
 * put the very first buffer into motion; from this point on,
 * DMA0_IRQHandler below is what keeps re-arming subsequent transfers, this
 * function is never called again.
 */
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
 * WHAT: Fires once a DMA transfer finishes filling a whole buffer (64
 * samples).
 * HOW: Clears the DMA channel's completion flag, flips g_fillIndex to the
 * OTHER buffer and immediately starts a new transfer into it, then marks
 * the just-completed buffer as g_readyIndex for main() to consume.
 * WHY: The order here matters: re-arming DMA into the other buffer happens
 * BEFORE handing the finished buffer to main(), so there is no gap in
 * acquisition, the next sample after this interrupt fires still lands
 * somewhere valid. This chip's DMA has no built-in scatter-gather (unlike
 * larger Kinetis parts' eDMA), so this software-driven alternation between
 * two fixed buffers is what stands in for that hardware feature.
 */
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

    /*
     * WHAT: Brings up the whole pipeline in a specific order.
     * WHY: ConfigureTpmTrigger() runs LAST, deliberately, same reasoning as
     * P1 dashboard's InitPitTick(): TPM0 is the piece that actually starts
     * triggering conversions, so the ADC, its hardware-trigger routing, and
     * DMA must all already be fully configured and armed before that first
     * trigger can possibly arrive.
     */
    ConfigureAdc();
    ConfigureAdcHardwareTrigger();
    ConfigureDma();
    ConfigureTpmTrigger(); /* start sampling last, once every downstream piece is ready */

    /*
     * WHAT: Waits for a full buffer to be ready, then prints its first 8
     * raw samples.
     * HOW: Checks g_readyIndex (set by DMA0_IRQHandler) every pass; when a
     * buffer is ready, immediately clears the flag back to -1 (so the same
     * buffer isn't printed twice) before touching the buffer's contents.
     * WHY: main() here does almost nothing, on purpose: it's proof that the
     * acquisition pipeline runs entirely on its own. Printing only 8 of the
     * 64 samples (not all of them) keeps this demo's UART output readable;
     * Session 2's full pipeline instead processes and streams every sample
     * through a FIR filter.
     */
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
