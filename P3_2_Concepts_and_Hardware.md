# P3: Concepts and Hardware

*Part of the P3 manual split. See `P3_1_Start_Here.md` for the full file list and how to use this manual.*

---

## 1. New Concepts You Need Before Starting

Builds on P0 through P2 (microcontroller basics, interrupts and the NVIC, the superloop-plus-flag pattern, and P1's ADC configuration specifically). Everything below is new to P3.

**DMA (Direct Memory Access).** A DMA controller is a small, separate piece of hardware whose only job is copying data from one address to another, addresses in peripheral registers, RAM, or both, without the CPU executing a single instruction for each byte moved. You configure it once (source address, destination address, how many bytes, what triggers each transfer) and then it runs autonomously; the CPU finds out only when it's done, via an interrupt, if it asks to be told at all.

**DMAMUX.** On this chip, the DMA controller itself doesn't know which peripheral's "I have new data" signal should trigger which DMA channel; a separate small peripheral, the DMA Multiplexer (DMAMUX), does that routing. You tell the DMAMUX "channel 0 listens to the ADC," and only after that is set up does an ADC conversion completing actually cause channel 0 to fire.

**Hardware trigger vs. software trigger.** Every ADC read you've done through P1 was software-triggered: your code called a function, and that function's own instructions caused the conversion to start. A hardware trigger removes your code from that decision entirely; some other piece of hardware (here, a timer) directly signals the ADC to start converting, on a schedule your code set up once and then has no further say over. This is the only way to guarantee a sample rate isn't jittered by whatever the CPU happened to be doing (an interrupt, a cache effect, anything) at the moment a software-triggered read would have been issued.

**Ping-pong buffering.** Two buffers, used alternately: while one is being filled (here, by DMA), the other, already full from the previous round, is available for something else (here, your code) to read, with no risk of reading data that's still being written. The instant the active buffer fills, the roles swap. This is how a system samples continuously with a bounded amount of memory, rather than needing one buffer that grows forever.

**Fixed-point arithmetic and Q15.** The Cortex-M0+ has no FPU (floating-point unit); every `float` or `double` operation is emulated in software, one instruction at a time, which is far too slow for filtering thousands of samples per second. The production answer on hardware like this is fixed-point: represent a fractional number as a plain integer that's implicitly been multiplied by some fixed scale factor. Q15 is one specific, extremely common convention: a 16-bit signed integer represents a real value from -1.0 to just under 1.0, with the integer equal to the real value times 32768 (2^15). `0.5` in Q15 is `16384`; `1.0` itself doesn't quite fit (the closest representable value is `32767`, or `0.99997`). This project uses Q15 specifically for filter coefficients, which are naturally small fractions.

**FIR filter and convolution.** An FIR (Finite Impulse Response) filter produces each output sample as a weighted sum of the current input sample and a fixed number of recent past samples, `y[n] = sum(k) h[k] * x[n-k]`, where the `h[k]` values are the filter's coefficients. That weighted-sum operation, applied at every sample, is called convolution. Which coefficients you choose determines what the filter does; Section 9 covers designing an actual low-pass set.

**Cutoff frequency.** The rough frequency above which a low-pass filter starts attenuating (reducing the amplitude of) a signal. Below the cutoff, the signal passes through close to unchanged; well above it, the signal is strongly reduced. "Rough" because a real filter's transition isn't a sharp wall, Section 9 shows the actual, computed, gradual roll-off this project's filter has.

## 4. Hardware and Register Reference

| Fact | Value | Verified against |
|---|---|---|
| Analog input pin | **PTE30** = ADC0_SE23, same channel P1 used | `driver_examples/adc16/polling`, reused directly from P1's manual |
| ADC hardware trigger source select | `SIM->SOPT7`, field `ADC0TRGSEL` (bits 3:0), value `0b1000` = TPM0 overflow | `MKL26Z4.xml`, the chip's own register description (see the full enumeration in Section 7) |
| ADC alternate trigger enable | `SIM->SOPT7`, bit `ADC0ALTTRGEN` = 1, required for `ADC0TRGSEL` to take effect at all | `MKL26Z4.xml`, confirmed working in `demo_apps/adc16_low_power_async_dma` |
| DMA peripheral on this chip | Simple 4-channel DMA (register names `SAR`/`DAR`/`DCR`/`DSR_BCR`), **not** the eDMA found on larger Kinetis parts | `fsl_dma.h`, `fsl_dma.c` |
| DMAMUX source for ADC0 | `kDmaRequestMux0ADC0` | `demo_apps/adc16_low_power_async_dma`, `fsl_dmamux.h` |
| DMA transfer function signature | `DMA_PrepareTransfer(config, srcAddr, srcWidth, destAddr, destWidth, transferBytes, type)` | `fsl_dma.h`, confirmed by reading the implementation |
| ADC result register address (DMA source) | `(uint32_t)(&ADC0->R[0])` | `demo_apps/adc16_low_power_async_dma` |
| Does `DMA_SetTransferConfig()` disturb the enable/async-request bits? | No; it only touches the size/increment fields in `DCR`, confirmed by reading `fsl_dma.c` directly | `fsl_dma.c` (`DMA_SetTransferConfig` implementation) |

> **This project resolves P0's open question.** P0 flagged that the FRDM-KL26Z's on-board "ambient light sensor," referenced in the original project spec, doesn't actually exist as a dedicated component anywhere in the SDK, and suggested the ADC0_SE23 header pin (PTE30) with an external photoresistor or potentiometer as the practical substitute. That's exactly what this project uses. If a leader has since confirmed there's no dedicated ALS on the board, this is the moment that substitution becomes load-bearing, not optional; wire an actual potentiometer to PTE30 before WS6, or the "signal" being filtered is just electrical noise on a floating pin, which still demonstrates the pipeline but makes for a much less convincing plot.

---

**Next:** `P3_3_Setup_and_Walkthrough.md` for the hands-on session steps.

---
*IEEE Texas State University Student Branch. Connect. Build. Inspire.*
