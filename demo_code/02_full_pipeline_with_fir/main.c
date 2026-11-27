/* ==========================================================================
 *  P3 DEMO 2: CLEANING THE SIGNAL WITH A FIR FILTER, A STEP-BY-STEP LADDER
 * ==========================================================================
 *
 *  Goal: take the stream of samples from Demo 1, run it through a digital
 *  LOW-PASS FILTER that smooths out fast jittery noise, and plot the raw
 *  and the filtered signal together on your computer, live.
 *
 *  THIS FILE IS DIFFERENT FROM THE OTHERS: the sampling pipeline you built
 *  in Demo 1 (timer, ADC, DMA, ping-pong buffers) is ALREADY TURNED ON in
 *  this file. It is exactly the same code. Do not change it. What is new
 *  here, and switched off, is the FILTER, in three steps.
 *
 *  With every step still off, this program already runs. It prints lines
 *  like  34012,0  where the first number is the raw sample and the second
 *  (the filtered one) is just 0, because no filter is running yet. Each
 *  step you turn on makes the second number come alive.
 *
 *  BEFORE YOU START: SET UP YOUR PROJECT (3 quick steps)
 *
 *    1. Create a new project with the SDK wizard. The P0 manual, Section 9,
 *       Step 1, shows exactly how.
 *    2. Paste this whole file over the contents of your project's main
 *       source file.
 *    3. Replace your project's pin_mux.c with the pin_mux.c that sits next
 *       to this file. It tells the chip which pins are the LED, the button
 *       and so on. Forget it and the program still builds and runs, but
 *       the LED or button does nothing, with no error message at all.
 *
 *  HOW TO USE THIS FILE
 *
 *    1. Read the STEP box. It explains the idea in plain words.
 *    2. Select ALL the grey "//" code lines under it (the whole block).
 *    3. Press  Cmd + /  (Mac)  or  Ctrl + /  (Windows)  to turn them ON.
 *    4. Build, flash, and watch the terminal.
 *    5. Do the "TRY IT" exercise, then move on to the next STEP.
 *
 *  Do the steps IN ORDER. Each step is one self-contained block, so after
 *  you turn on a step the program always builds and runs with no errors.
 *  Each box has a "NEEDS" line telling you which earlier steps must be on.
 *
 *  THE ONE RULE
 *
 *    Text inside  / * ... * /  boxes is explanation. Leave it alone.
 *    Lines starting with  //  are code that is switched off. Those are the
 *    only lines you ever turn on. Code that does NOT start with  //  is
 *    already on. Leave it alone too.
 *
 *  WHAT IS A FIR FILTER?
 *    A FIR (Finite Impulse Response) filter computes each output sample as
 *    a WEIGHTED AVERAGE of the latest few input samples. Here we use 8
 *    samples, each multiplied by its own weight (called a coefficient, or
 *    a "tap"), then added up. Fast wiggles cancel out in the average. Slow
 *    changes survive. So fast jitter is removed and the overall shape of
 *    the signal remains. That is what "low-pass" means: low frequencies
 *    pass, high ones get blocked.
 * ========================================================================== */


/* --------------------------------------------------------------------------
 *  INCLUDES (already on, nothing to do)
 * -------------------------------------------------------------------------- */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"
#include "fsl_tpm.h"





/* --------------------------------------------------------------------------
 *  ALREADY ON: THE NUMBERS, BUFFERS AND FLAGS (same as Demo 1, STEP 1)
 *
 *  Two additions that are new in this demo:
 *
 *    DECIMATION 16     we only PRINT every 16th sample. Printing all 10,000
 *                      samples a second would need about 120 KB per second
 *                      of serial bandwidth, but the serial line only
 *                      carries about 11.5 KB per second. Printing every 16th
 *                      cuts the rate to 625 per second, which fits. The
 *                      sampling and filtering still run at the full
 *                      10,000 per second. Only the PRINTING is thinned out.
 *
 *    g_filteredBuffer  where the filter will put its output, one filtered
 *                      sample for each raw sample. It starts as all zeros,
 *                      which is why you see 0 until the filter is on.
 * -------------------------------------------------------------------------- */

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_CHANNEL 23U
#define ADC16_RESULT_REG_ADDR ((uint32_t)(&ADC0->R[0]))

#define DEMO_DMA_BASEADDR DMA0
#define DEMO_DMA_CHANNEL 0U
#define DEMO_DMAMUX_BASEADDR DMAMUX0

#define BUFFER_SAMPLES 64U
#define SAMPLE_RATE_HZ 10000U

#define DECIMATION 16U

static uint32_t g_bufferA[BUFFER_SAMPLES];
static uint32_t g_bufferB[BUFFER_SAMPLES];
static uint32_t *const g_pingPongBuffers[2] = {g_bufferA, g_bufferB};

static dma_handle_t g_dmaHandle;
static dma_transfer_config_t g_transferConfig;

static volatile uint8_t g_fillIndex = 0U;
static volatile int32_t g_readyIndex = -1;

static uint32_t g_filteredBuffer[BUFFER_SAMPLES];





/* --------------------------------------------------------------------------
 *  ALREADY ON: THE SAMPLING PIPELINE (same as Demo 1, STEPS 2 to 4)
 *
 *  Do not change this part. It is the same code you built in Demo 1: the
 *  ADC trigger wiring, the ADC setup, the timer that sets 10,000 samples a
 *  second, the DMA, and the interrupt that swaps the two buffers.
 *
 *  One thing to notice in the interrupt handler at the bottom: the FILTER
 *  is deliberately NOT run in there. Handlers must be fast. The filter is
 *  slower, so it runs later, in main(), after the handler has already
 *  pointed DMA at the other buffer. That way the filter can never make
 *  the pipeline miss a sample.
 * -------------------------------------------------------------------------- */

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





/* ==========================================================================
 *  STEP 1: THE FILTER'S WEIGHTS
 *
 *  CONCEPT
 *    FIR_TAPS 8 means the filter looks at 8 samples at a time.
 *
 *    The 8 coefficients are the weights, one per sample. Notice they are
 *    symmetric, a mirror image:
 *
 *        570  2006  5445  8363  |  8363  5445  2006  570
 *
 *    Big weights in the middle, small at the ends. A smooth bump. The
 *    symmetry gives the filter "linear phase": every frequency is delayed
 *    by the same amount, so the shape of the waveform is smoothed, not
 *    smeared out of shape.
 *
 *    What is "Q15"? The real weights are decimals between 0 and 1 (for
 *    example 0.255). But this chip has no hardware for decimals, and it
 *    cannot print them either. So each weight is multiplied by 32768 (that
 *    is 2 to the power 15) and rounded to a whole number. 1.0 is stored as
 *    32768. Later we divide by 32768 once to undo it. Whole-number math is
 *    much faster on this chip than decimal math.
 *
 *    These particular weights make a low-pass filter with a cutoff of
 *    about 500 Hz when sampling at 10,000 per second. Fast wiggles above
 *    about 1 kHz are weakened a lot. The weights were computed, not
 *    guessed. The manual has the design details.
 *
 *  NEEDS: nothing from the filter steps. Safe to flash right away.
 *
 *  TRY IT
 *    Turn this on and build. Add up the 8 numbers. Is the total close to
 *    32768? (It should be: 570+2006+5445+8363+8363+5445+2006+570 = 32768.)
 *    That total being exactly 1.0 means a steady (unchanging) signal comes
 *    out at exactly the same level it went in. The filter does not make
 *    things louder or quieter.
 *
 *  NEXT: go to STEP 2.
 * ========================================================================== */

//#define FIR_TAPS 8U
//
//const int32_t g_firCoeffsQ15[FIR_TAPS] = {570, 2006, 5445, 8363, 8363, 5445, 2006, 570};





/* ==========================================================================
 *  STEP 2: THE FILTER FUNCTION AND ITS MEMORY
 *
 *  CONCEPT
 *    Two parts.
 *
 *    g_firHistory
 *        To compute the output for sample number n, the filter needs
 *        sample n AND the 7 samples before it. For the first few samples of
 *        a new buffer, those 7 earlier samples are in the PREVIOUS buffer,
 *        which DMA may already be overwriting. So we save the last 7 raw
 *        samples of each buffer here, and use them at the start of the
 *        next one. This keeps the filtering continuous across the
 *        ping-pong boundary, with no glitch at every swap. At power on it
 *        is all zeros, which makes one brief startup glitch. Any real
 *        filter does that.
 *
 *    ApplyFirFilter(rawBuffer, filteredBuffer, count)
 *        For every sample n in the buffer:
 *          1. start an accumulator at zero
 *          2. for each of the 8 taps k, multiply weight k by the sample
 *             that is k places back, and add to the accumulator
 *             (if that sample is before the start of this buffer, take it
 *             from g_firHistory instead)
 *          3. shift the total right by 15, which divides by 32768 and
 *             undoes the Q15 scaling
 *        Then, at the end, save the last 7 raw samples into g_firHistory
 *        for next time.
 *
 *    Why shift only ONCE at the end, and not after every multiply? Every
 *    shift throws away some precision. Shifting once instead of 8 times
 *    loses far less. This is a real technique used in real fixed-point
 *    code.
 *
 *  NEEDS: STEP 1 already on. Safe to flash right away. Defined, not yet
 *  called, so the "filtered" number on screen is still 0.
 *
 *  TRY IT
 *    Turn the whole block on and build. Find the line with  acc >> 15.
 *    Find the line with  (idx >= 0)  and work out in your own words what
 *    it chooses between.
 *
 *  NEXT: go to STEP 3.
 * ========================================================================== */

//uint32_t g_firHistory[FIR_TAPS - 1U];
//
//void ApplyFirFilter(const uint32_t *rawBuffer, uint32_t *filteredBuffer, uint32_t count)
//{
//    uint32_t n;
//    uint32_t k;
//
//    for (n = 0U; n < count; n++)
//    {
//        int32_t acc = 0;
//
//        for (k = 0U; k < FIR_TAPS; k++)
//        {
//            int32_t idx = (int32_t)n - (int32_t)k;
//            uint32_t sample = (idx >= 0) ? rawBuffer[idx] : g_firHistory[(FIR_TAPS - 1U) + idx];
//
//            acc += g_firCoeffsQ15[k] * (int32_t)sample;
//        }
//
//        filteredBuffer[n] = (uint32_t)(acc >> 15);
//    }
//
//    for (k = 0U; k < (FIR_TAPS - 1U); k++)
//    {
//        g_firHistory[k] = rawBuffer[count - (FIR_TAPS - 1U) + k];
//    }
//}





int main(void)
{

    /* ----------------------------------------------------------------------
     *  ALREADY ON: START EVERYTHING (same as Demo 1, STEPS 5 to 8)
     *
     *  Wake the board, print a banner, then start the pipeline in the same
     *  order as before. The timer is last, because it is the call that
     *  actually starts sampling.
     * -------------------------------------------------------------------- */

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n=== P3 full pipeline (FIR + Python plot) ===\r\n");
    PRINTF("Sampling ADC0_SE23 (PTE30) at %u Hz, printing every %uth sample as raw,filtered.\r\n",
           SAMPLE_RATE_HZ, DECIMATION);
    PRINTF("Point python/plot_fir_demo.py at this board's serial port to see it live.\r\n\r\n");

    ConfigureAdc();
    ConfigureAdcHardwareTrigger();
    ConfigureDma();
    ConfigureTpmTrigger();

    /* ----------------------------------------------------------------------
     *  THE MAIN LOOP (already on)
     *
     *  Waits for a finished buffer, then prints every 16th sample as a pair
     *  "raw,filtered". Printing both together is what lets the plot draw the
     *  two signals on top of each other, so the effect of the filter is
     *  easy to see.
     *
     *  Notice the gap in the middle, marked STEP 3. That is where you will
     *  call the filter. Until then g_filteredBuffer stays all zeros.
     * -------------------------------------------------------------------- */

    for (;;)
    {
        if (g_readyIndex >= 0)
        {
            int32_t index = g_readyIndex;
            uint32_t i;
            uint32_t *raw;

            g_readyIndex = -1;
            raw = g_pingPongBuffers[index];


            /* ==============================================================
             *  STEP 3: TURN THE FILTER ON
             *
             *  CONCEPT
             *    One line. Filter this finished buffer: read the raw
             *    samples, write the smoothed result into
             *    g_filteredBuffer. The print loop just below then shows
             *    real filtered numbers instead of zeros.
             *
             *  NEEDS: STEPS 1 and 2 already on. Safe to flash right away.
             *
             *  TRY IT
             *    Turn this on and flash. The terminal lines change from
             *    "34012,0" to something like "34012,33890". Now the second
             *    number is the filtered version of the first. Touch PTE30
             *    with your finger. The raw number jumps around a lot. The
             *    filtered one jumps around less.
             *
             *  NEXT: see the plotting section below, then the exercises.
             * ============================================================ */

            //ApplyFirFilter(raw, g_filteredBuffer, BUFFER_SAMPLES);


            for (i = 0U; i < BUFFER_SAMPLES; i += DECIMATION)
            {
                PRINTF("%u,%u\r\n", raw[i], g_filteredBuffer[i]);
            }
        }
    }
}





/* ==========================================================================
 *  SEE IT ON A GRAPH
 *
 *  Numbers scrolling by are hard to judge. Plot them instead.
 *
 *  1. Close the terminal window (only one program can use the serial port
 *     at a time).
 *  2. In the python/ folder next to this one, run plot_fir_demo.py and
 *     point it at your board's serial port. Your project leader can tell
 *     you the port name.
 *  3. You get two lines on a live graph, raw and filtered. Touch the PTE30
 *     pin, wave a wire near it, or connect a potentiometer or a signal
 *     source. Watch the filtered line stay smoother than the raw line.
 *
 *
 *  EXERCISES
 *
 *  1. Watch the delay. A filter averages the recent past, so its output
 *     lags behind its input a little. On the plot, make a sudden change
 *     (touch the pin). Does the filtered line follow slightly later than
 *     the raw one? That lag is the price of smoothing.
 *
 *  2. Make a stronger filter. In STEP 1, try these 8 equal weights, which
 *     add to 32768 and just take a plain average:
 *
 *         {4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096}
 *
 *     Compare the plot with the original weights. Which one smooths more?
 *     Which one keeps sharper corners?
 *
 *  3. Break the weights on purpose. Change one coefficient so that the 8
 *     no longer add up to 32768, say make the first one 5000. What
 *     happens to the level of the filtered signal compared with the raw
 *     one? Why? (Put it back afterward.)
 *
 *  4. Look at the startup glitch. Press the reset button and look at the
 *     very first few lines. The first filtered value is probably too low.
 *     Why? (Hint: g_firHistory starts as all zeros.)
 *
 *  5. Change the printing rate. In the "ALREADY ON" section change
 *     DECIMATION from 16U to 4U. Flash. Does the plot get smoother?
 *     Does the terminal ever look like it falls behind? Why might it? (See
 *     the explanation of DECIMATION above.)
 *
 *  6. Challenge: remove history. In ApplyFirFilter, replace
 *     g_firHistory[(FIR_TAPS - 1U) + idx] with 0 and look at the plot at
 *     every buffer boundary. You should see a small glitch every 64
 *     samples. That is the problem the history array solves. Put it back.
 *
 *  WHERE TO GO NEXT
 *    P4A: a structured packet protocol, so the board can send and receive
 *    proper commands instead of just printing text.
 * ========================================================================== */
