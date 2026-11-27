/* ==========================================================================
 *  P3 DEMO 1: SAMPLING WITH DMA (PING-PONG BUFFERS), A STEP-BY-STEP LADDER
 * ==========================================================================
 *
 *  Goal: measure an analog voltage 10,000 times every second, store every
 *  measurement in memory, and print a few of them. And do ALL the
 *  measuring without the CPU lifting a finger.
 *
 *  Three pieces of hardware work together as a pipeline:
 *
 *    TPM0 timer  : ticks exactly 10,000 times a second, like a metronome
 *    ADC         : on each tick, measures the voltage on pin PTE30
 *    DMA         : on each finished measurement, copies the result into
 *                  memory, automatically
 *
 *  DMA stands for Direct Memory Access. Think of it as a courier that moves
 *  data between a peripheral and memory by itself, so the CPU is free for
 *  other work. In P1 Demo 3 the CPU waited for each ADC reading. That can
 *  not keep up with thousands of samples a second. DMA can.
 *
 *  This file starts almost empty on purpose. The code for each step is
 *  already here, but turned OFF (commented out). You switch it on one step
 *  at a time, run it on the board, and see what changed.
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
 *    only lines you ever turn on.
 *
 *  HOW THIS FILE IS LAID OUT
 *
 *    PART A (STEPS 1 to 4, above main): writes the pieces of the pipeline
 *    as functions. Nothing runs yet. Each step just needs to build with no
 *    errors.
 *
 *    PART B (STEPS 5 to 9, inside main): starts the pieces in the right
 *    order, and finally prints what was captured.
 *
 *  WHAT IS A PING-PONG BUFFER?
 *    DMA fills a buffer (a block of memory) with 64 samples. While we read
 *    the finished buffer, DMA must keep sampling without a gap, so it needs
 *    somewhere else to put new samples. So we use TWO buffers, A and B.
 *    DMA fills A while we read B. Then they swap. Like a ping-pong ball
 *    going back and forth, hence the name.
 * ========================================================================== */


/* --------------------------------------------------------------------------
 *  INCLUDES (already on, nothing to do)
 *
 *  fsl_adc16.h : the ADC driver
 *  fsl_dma.h   : the DMA driver
 *  fsl_dmamux.h: the DMA multiplexer, which decides which peripheral's
 *                requests are connected to which DMA channel
 *  fsl_tpm.h   : the timer driver
 * -------------------------------------------------------------------------- */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "fsl_dmamux.h"
#include "fsl_tpm.h"





/* ==========================================================================
 *  PART A, STEP 1: THE NUMBERS, THE TWO BUFFERS AND THE FLAGS
 *
 *  CONCEPT
 *    Everything else depends on these, so they come first.
 *
 *    Which hardware:
 *      DEMO_ADC16_CHANNEL 23    the ADC input wired to pin PTE30, the same
 *                               pin P1 Demo 3 used
 *      ADC16_RESULT_REG_ADDR    the raw memory ADDRESS of the ADC's result
 *                               register. DMA works with plain addresses,
 *                               not driver functions. It reads a fresh
 *                               measurement from this address each time.
 *      DMA0, channel 0, DMAMUX0 which DMA engine and channel we use
 *
 *    How much:
 *      BUFFER_SAMPLES 64        each buffer holds 64 samples
 *      SAMPLE_RATE_HZ 10000     10,000 samples every second
 *
 *    The two buffers, A and B, and a small list g_pingPongBuffers so that
 *    code can say "buffer 0 or buffer 1" instead of naming A or B.
 *
 *    The flags, shared between the DMA interrupt and main():
 *      g_fillIndex       which buffer DMA is writing into right now
 *      g_readyIndex      which finished buffer main() should look at.
 *                        -1 means "none waiting".
 *      g_buffersCaptured how many buffers have been filled so far
 *    "volatile" is required because they change behind main()'s back.
 *
 *  NEEDS: nothing. Safe to flash right away.
 *
 *  TRY IT
 *    Turn the whole block on and build. Work out for yourself: how long, in
 *    milliseconds, does it take to fill one 64-sample buffer at 10,000
 *    samples a second? (Answer in the exercises.)
 *
 *  NEXT: go to STEP 2.
 * ========================================================================== */

//#define DEMO_ADC16_BASEADDR ADC0
//#define DEMO_ADC16_CHANNEL_GROUP 0U
//#define DEMO_ADC16_CHANNEL 23U
//#define ADC16_RESULT_REG_ADDR ((uint32_t)(&ADC0->R[0]))
//
//#define DEMO_DMA_BASEADDR DMA0
//#define DEMO_DMA_CHANNEL 0U
//#define DEMO_DMAMUX_BASEADDR DMAMUX0
//
//#define BUFFER_SAMPLES 64U
//#define SAMPLE_RATE_HZ 10000U
//
//uint32_t g_bufferA[BUFFER_SAMPLES];
//uint32_t g_bufferB[BUFFER_SAMPLES];
//uint32_t *const g_pingPongBuffers[2] = {g_bufferA, g_bufferB};
//
//dma_handle_t g_dmaHandle;
//dma_transfer_config_t g_transferConfig;
//
//volatile uint8_t g_fillIndex = 0U;
//volatile int32_t g_readyIndex = -1;
//volatile uint32_t g_buffersCaptured = 0U;





/* ==========================================================================
 *  PART A, STEP 2: THE ADC AND ITS HARDWARE TRIGGER
 *
 *  CONCEPT
 *    Two functions.
 *
 *    ConfigureAdcHardwareTrigger
 *        Wires the timer straight to the ADC, inside the chip. The ADC
 *        normally listens to a different trigger source. This one line
 *        writes to a chip-level routing register (SIM->SOPT7) to say:
 *          ADC0TRGSEL = 8      "start a conversion when TPM0 overflows"
 *          ADC0ALTTRGEN = 1    "listen to this choice instead of the
 *                               default"
 *        Forget ADC0ALTTRGEN and the timer ticks with no effect, because
 *        the ADC is still listening to something else.
 *
 *    ConfigureAdc
 *        Same "fill defaults, then apply" pattern as P1. Two changes:
 *          16-bit resolution     results from 0 to 65535, finer than the
 *                                12-bit of P1
 *          continuous conversion OFF, because the timer will start each
 *                                conversion individually
 *        It also calibrates the ADC, then picks channel 23. The last two
 *        lines are the heart of the pipeline:
 *          EnableHardwareTrigger  the ADC starts itself on each timer tick
 *          EnableDMA              each finished conversion raises a DMA
 *                                 request, so DMA collects the result
 *
 *  NEEDS: STEP 1 already on. Safe to flash right away. Defined, not run.
 *
 *  TRY IT
 *    Turn the whole block on and build. Find the line that selects 16-bit
 *    resolution. Next to it, what would 12-bit be called?
 *
 *  NEXT: go to STEP 3.
 * ========================================================================== */

//void ConfigureAdcHardwareTrigger(void)
//{
//    SIM->SOPT7 = (SIM->SOPT7 & ~SIM_SOPT7_ADC0TRGSEL_MASK) | SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN(1);
//}
//
//void ConfigureAdc(void)
//{
//    adc16_config_t adcConfig;
//    adc16_channel_config_t channelConfig;
//
//    ADC16_GetDefaultConfig(&adcConfig);
//    adcConfig.resolution = kADC16_Resolution16Bit;
//    adcConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
//    adcConfig.enableContinuousConversion = false;
//    ADC16_Init(DEMO_ADC16_BASEADDR, &adcConfig);
//
//#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
//    ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR);
//#endif
//
//    channelConfig.channelNumber = DEMO_ADC16_CHANNEL;
//    channelConfig.enableInterruptOnConversionCompleted = false;
//    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &channelConfig);
//
//    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, true);
//    ADC16_EnableDMA(DEMO_ADC16_BASEADDR, true);
//}





/* ==========================================================================
 *  PART A, STEP 3: THE TIMER THAT SETS THE SAMPLE RATE
 *
 *  CONCEPT
 *    The timer TPM0 is the metronome of the whole pipeline. We set it to
 *    overflow, meaning count up to a limit and wrap to zero, exactly 10,000
 *    times every second.
 *
 *    How: the timer counts clock ticks. If the clock runs at tpmClockHz
 *    ticks per second, then counting to  tpmClockHz / 10,000  takes
 *    1/10,000 of a second. The "- 1" is because counting starts at zero.
 *
 *    This timer does NOT drive any pin or LED. Its only job is the
 *    internal overflow event, which STEP 2 wired to the ADC. That makes the
 *    sample rate hardware-exact, far steadier than any delay loop in
 *    software.
 *
 *  NEEDS: STEPS 1 and 2 already on. Safe to flash right away. Defined, not
 *  run.
 *
 *  TRY IT
 *    Turn the whole block on and build. Which single number would you
 *    change to sample 2,000 times a second instead?
 *
 *  NEXT: go to STEP 4.
 * ========================================================================== */

//void ConfigureTpmTrigger(void)
//{
//    tpm_config_t tpmConfig;
//    uint32_t tpmClockHz = CLOCK_GetFreq(kCLOCK_PllFllSelClk);
//
//    CLOCK_SetTpmClock(1U);
//    TPM_GetDefaultConfig(&tpmConfig);
//    TPM_Init(TPM0, &tpmConfig);
//    TPM_SetTimerPeriod(TPM0, (tpmClockHz / SAMPLE_RATE_HZ) - 1U);
//    TPM_StartTimer(TPM0, kTPM_SystemClock);
//}





/* ==========================================================================
 *  PART A, STEP 4: THE DMA AND THE PING-PONG SWAP
 *
 *  CONCEPT
 *    Three pieces.
 *
 *    StartDmaIntoBuffer(bufferIndex)
 *        Tells DMA: "copy 64 results, one at a time, from the ADC result
 *        register into this buffer."
 *          DMA_PrepareTransfer      describes the job: source address, the
 *                                   destination buffer, and the size.
 *                                   kDMA_PeripheralToMemory means the data
 *                                   flows from a peripheral into memory.
 *          DMA_EnableCycleSteal     very important. It makes DMA move ONE
 *                                   sample per ADC request. Without it,
 *                                   DMA would race through the whole buffer
 *                                   the instant the first request arrives,
 *                                   copying the same single reading 64
 *                                   times, defeating the point of the
 *                                   timer.
 *          DMA_EnableInterrupts     ring the CPU's doorbell when the whole
 *                                   buffer is full
 *
 *    ConfigureDma
 *        One-time setup. DMAMUX comes first because it connects "ADC0
 *        finished a conversion" to DMA channel 0. Then start the first
 *        transfer, into buffer 0, and switch on the DMA interrupt.
 *
 *    DMA0_IRQHandler
 *        Runs each time a buffer is full. Order matters:
 *          1. clear the "done" marker
 *          2. flip g_fillIndex to the OTHER buffer
 *          3. immediately start DMA into that other buffer
 *          4. only then tell main() "the first buffer is ready"
 *        Restarting DMA BEFORE handing off the finished buffer means there
 *        is no gap in sampling. This chip's DMA has no automatic "chain to
 *        the next buffer" feature (bigger chips do), so the swap is done
 *        in software right here.
 *
 *  NEEDS: STEPS 1 to 3 already on. Safe to flash right away. Defined, not
 *  run.
 *
 *  TRY IT
 *    Turn the whole block on and build. That finishes the toolbox. In the
 *    handler, find the one line that does the ping-pong flip:
 *    g_fillIndex = 1U - g_fillIndex;  If it is 0, what does it become? If
 *    it is 1?
 *
 *  NEXT: go to STEP 5 inside main().
 * ========================================================================== */

//void StartDmaIntoBuffer(uint8_t bufferIndex)
//{
//    DMA_PrepareTransfer(&g_transferConfig, (void *)ADC16_RESULT_REG_ADDR, sizeof(uint32_t),
//                        (void *)g_pingPongBuffers[bufferIndex], sizeof(uint32_t), BUFFER_SAMPLES * sizeof(uint32_t),
//                        kDMA_PeripheralToMemory);
//
//    DMA_SetTransferConfig(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, &g_transferConfig);
//    DMA_EnableInterrupts(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL);
//    DMA_EnableAsyncRequest(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true);
//    DMA_EnableCycleSteal(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, true);
//    DMA_StartTransfer(&g_dmaHandle);
//}
//
//void ConfigureDma(void)
//{
//    DMAMUX_Init(DEMO_DMAMUX_BASEADDR);
//    DMAMUX_SetSource(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL, kDmaRequestMux0ADC0);
//    DMAMUX_EnableChannel(DEMO_DMAMUX_BASEADDR, DEMO_DMA_CHANNEL);
//
//    DMA_Init(DEMO_DMA_BASEADDR);
//    DMA_CreateHandle(&g_dmaHandle, DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL);
//
//    g_fillIndex = 0U;
//    StartDmaIntoBuffer(g_fillIndex);
//
//    NVIC_EnableIRQ(DMA0_IRQn);
//}
//
//void DMA0_IRQHandler(void)
//{
//    uint8_t justFilled = g_fillIndex;
//
//    DMA_ClearChannelStatusFlags(DEMO_DMA_BASEADDR, DEMO_DMA_CHANNEL, kDMA_TransactionsDoneFlag);
//
//    g_fillIndex = 1U - g_fillIndex;
//    StartDmaIntoBuffer(g_fillIndex);
//
//    g_readyIndex = (int32_t)justFilled;
//    g_buffersCaptured++;
//}





int main(void)
{

    /* ======================================================================
     *  PART B, STEP 5: WAKE UP THE BOARD
     *
     *  CONCEPT
     *    Three setup calls run once, in this order, before anything else:
     *
     *      BOARD_InitPins()          decide what each pin does
     *      BOARD_BootClockRUN()      start the clock (the chip's heartbeat)
     *      BOARD_InitDebugConsole()  open the serial line to your computer
     *
     *    Then two PRINTF lines say what we are about to do.
     *
     *  NEEDS: STEP 1 already on (for the numbers printed). Safe to flash
     *  right away.
     *
     *  TRY IT
     *    Turn the whole block on, flash, and open the terminal. You should
     *    see the title and "10000 Hz, 64 samples per buffer".
     *
     *  NEXT: go to STEP 6.
     * ==================================================================== */

    //BOARD_InitPins();
    //BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();
    //
    //PRINTF("\r\n=== P3 DMA ping-pong (raw dump only) ===\r\n");
    //PRINTF("Sampling ADC0_SE23 (PTE30) at %u Hz, %u samples per buffer.\r\n\r\n", SAMPLE_RATE_HZ, BUFFER_SAMPLES);





    /* ======================================================================
     *  PART B, STEP 6: SET UP THE ADC
     *
     *  CONCEPT
     *    Calls the two ADC functions from STEP 2. The ADC is now ready and
     *    listening for the timer. The timer has not started yet, so no
     *    measurements are taken.
     *
     *  NEEDS: STEPS 1, 2 and 5 already on. Safe to flash right away.
     *
     *  TRY IT
     *    Turn this on and flash. Nothing new appears. The ADC is armed and
     *    waiting.
     *
     *  NEXT: go to STEP 7.
     * ==================================================================== */

    //ConfigureAdc();
    //ConfigureAdcHardwareTrigger();





    /* ======================================================================
     *  PART B, STEP 7: SET UP THE DMA
     *
     *  CONCEPT
     *    Calls ConfigureDma() from STEP 4. DMA is now armed and waiting for
     *    ADC requests. It will copy each result into buffer A as soon as
     *    they arrive.
     *
     *  NEEDS: STEPS 1 to 6 already on. Safe to flash right away.
     *
     *  TRY IT
     *    Turn this on and flash. Still nothing new. DMA is waiting for the
     *    ADC, and the ADC is waiting for the timer.
     *
     *  NEXT: go to STEP 8.
     * ==================================================================== */

    //ConfigureDma();





    /* ======================================================================
     *  PART B, STEP 8: START THE TIMER, LAST
     *
     *  CONCEPT
     *    ConfigureTpmTrigger() is deliberately the LAST setup call. It is
     *    the one that actually starts the pipeline. Once the timer ticks,
     *    the ADC measures and DMA copies, immediately. So the ADC, the
     *    trigger wiring, and DMA must ALL be ready first. This is the same
     *    "start the heartbeat last" rule you saw in P1 Demo 7.
     *
     *    From this line on, the pipeline runs entirely by itself. The CPU
     *    is not involved in taking a single sample.
     *
     *  NEEDS: STEPS 1 to 7 already on. Safe to flash right away.
     *
     *  TRY IT
     *    Turn this on and flash. The pipeline is running, and buffers are
     *    filling about 156 times a second. But nothing reads them yet, so
     *    you still see nothing new. STEP 9 fixes that.
     *
     *  NEXT: go to STEP 9.
     * ==================================================================== */

    //ConfigureTpmTrigger();





    /* ======================================================================
     *  PART B, STEP 9: LOOK AT WHAT WAS CAPTURED
     *
     *  CONCEPT
     *    This loop checks g_readyIndex. When a buffer is ready, it:
     *      1. copies the index and sets g_readyIndex back to -1, so the
     *         same buffer is not printed twice
     *      2. looks up which buffer that is
     *      3. prints the buffer number, how many buffers have been
     *         captured, and the first 8 raw samples
     *
     *    Only 8 of 64 are printed to keep the output readable. 156 buffers
     *    a second is far too fast to read, but you will see them scroll.
     *
     *    main() does almost nothing here, on purpose. That is the point:
     *    the pipeline runs on its own.
     *
     *    About the numbers: the ADC gives a 16-bit value, 0 to 65535, where
     *    0 is 0 volts and 65535 is 3.3 volts. So 32768 is about 1.65 volts.
     *
     *  NEEDS: STEPS 1 to 8 already on. Safe to flash right away.
     *
     *  TRY IT
     *    Turn this on and flash. Lines of numbers scroll quickly. Leave
     *    PTE30 unconnected and the numbers wander. Touch PTE30 with a
     *    finger and watch them move. Connect a potentiometer and turn it
     *    slowly. The numbers should rise and fall smoothly.
     *
     *  NEXT: go to the exercises below.
     * ==================================================================== */

    //for (;;)
    //{
    //    if (g_readyIndex >= 0)
    //    {
    //        int32_t index = g_readyIndex;
    //        uint32_t i;
    //        uint32_t *buf;
    //
    //        g_readyIndex = -1;
    //        buf = g_pingPongBuffers[index];
    //
    //        PRINTF("buffer %d ready (capture #%u), first 8 raw samples: ", index, g_buffersCaptured);
    //
    //        for (i = 0U; i < 8U; i++)
    //        {
    //            PRINTF("%u ", buf[i]);
    //        }
    //
    //        PRINTF("\r\n");
    //    }
    //}

}





/* ==========================================================================
 *  EXERCISES
 *
 *  1. Check the timing (STEP 1 question). One buffer holds 64 samples at
 *     10,000 per second: 64 / 10000 = 0.0064 seconds, or 6.4 ms. So about
 *     156 buffers a second. Now look at the "capture #" number on screen.
 *     Does it climb by roughly that much per second?
 *
 *  2. Change the sample rate. In STEP 1 change SAMPLE_RATE_HZ to 2000.
 *     Flash. The capture number should climb about five times slower. Try
 *     20000. Does the pipeline keep up? (If the printing cannot keep up,
 *     what will happen to the data you see?)
 *
 *  3. Change the buffer size. In STEP 1 change BUFFER_SAMPLES from 64U to
 *     128U. Does everything still work? What happens to the time one
 *     buffer takes to fill?
 *
 *  4. Show more samples. In STEP 9 change the 8U in the "for" loop to 16U.
 *
 *  5. Watch a real signal. Connect a potentiometer's middle leg to PTE30
 *     (outer legs to 3.3V and GND). Turn it slowly from one end to the
 *     other. Do the numbers span about 0 to 65535?
 *
 *  6. Break it on purpose (predict first!). In STEP 4, comment out the
 *     DMA_EnableCycleSteal line. Before you flash, predict what the 8
 *     printed samples will look like, and why. Then flash and check your
 *     prediction. Put the line back afterward.
 *
 *  WHERE TO GO NEXT
 *    Demo 2: apply a digital filter (FIR) to clean up the captured
 *    signal, and plot it live on your computer.
 * ========================================================================== */
