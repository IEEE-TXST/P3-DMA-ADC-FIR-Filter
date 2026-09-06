# TXST IEEE Student Branch: FRDM-KL26Z Project Series
## Project Manual, P3: DMA-Driven ADC Pipeline with Fixed-Point FIR Filter

**Document status:** DRAFT v0.1, for project-leader review and bench testing before member use
**Track:** Embedded Systems | **Difficulty:** Advanced | **Sessions:** 2 (WS6, Oct 22 and WS7, Oct 29, 2026), with pre-reading assigned Oct 15
**Applies to:** All 8 groups (24 members)
**Companion documents:** P0's manual (`P0_Board_Orientation_and_Toolchain_Setup/`, start at `P0_1_Start_Here.md`), `P1_Project_Manual.md`, `P2_Project_Manual.md` (read all three first), *TXST IEEE FRDM-KL26Z Project Specification*
**Hardware:** FRDM-KL26Z plus a potentiometer (roughly $1) wired to PTE30, or the on-board ADC header pin left floating for a noisier demo
**Demo code:** see `demo_code/` in this project's folder

---

## How to Use This Manual

Same rule as every prior manual: a reference, not required reading. Build each stage hands-on and open this only when stuck.

Project leaders: bench-test both `demo_code/01_dma_adc_ping_pong/` and `demo_code/02_full_pipeline_with_fir/` on a real board before WS6 and WS7, with an actual potentiometer wired to PTE30 (Section 4). This project's failure modes are quieter than P2's, a misconfigured trigger source doesn't hard-fault, it just silently produces flat or garbage data, so seeing it work correctly once, on real hardware, before the session matters more here than the raw difficulty might suggest.

**A note on accuracy:** every DMA, DMAMUX, and ADC-trigger register value in this manual was verified against either a working NXP SDK example (`demo_apps/adc16_low_power_async_dma`, which combines exactly these three peripherals) or the chip's own register-description file (`MKL26Z4.xml`), which lists the ADC's hardware trigger source options directly. The FIR filter's coefficients were computed with a real filter-design script (Section 9 includes it), not chosen by eye, and its frequency response was verified by computing it directly, not assumed. Section 20 has the full verification trail.

---

## 0. Why This Session Exists

P1 read an ADC by asking for a value and waiting. P2 built a scheduler that lets several tasks share a CPU. P3 removes the CPU from the acquisition loop entirely: a timer and a DMA controller, working together in hardware, sample a signal at a precise, guaranteed rate while the processor does something else, or nothing at all, until there's real work (filtering) to do. This is not a niche technique. It's the actual architecture behind every audio codec, motor controller, and data-acquisition system built on a microcontroller, because it's the only way to guarantee sample timing isn't at the mercy of whatever else the CPU happens to be doing. P2's scheduler taught you that CPU time is a resource tasks compete for; P3 teaches you how to take an entire job (sampling) off that competition altogether.

## 1. New Concepts You Need Before Starting

Builds on P0 through P2 (microcontroller basics, interrupts and the NVIC, the superloop-plus-flag pattern, and P1's ADC configuration specifically). Everything below is new to P3.

**DMA (Direct Memory Access).** A DMA controller is a small, separate piece of hardware whose only job is copying data from one address to another, addresses in peripheral registers, RAM, or both, without the CPU executing a single instruction for each byte moved. You configure it once (source address, destination address, how many bytes, what triggers each transfer) and then it runs autonomously; the CPU finds out only when it's done, via an interrupt, if it asks to be told at all.

**DMAMUX.** On this chip, the DMA controller itself doesn't know which peripheral's "I have new data" signal should trigger which DMA channel; a separate small peripheral, the DMA Multiplexer (DMAMUX), does that routing. You tell the DMAMUX "channel 0 listens to the ADC," and only after that is set up does an ADC conversion completing actually cause channel 0 to fire.

**Hardware trigger vs. software trigger.** Every ADC read you've done through P1 was software-triggered: your code called a function, and that function's own instructions caused the conversion to start. A hardware trigger removes your code from that decision entirely; some other piece of hardware (here, a timer) directly signals the ADC to start converting, on a schedule your code set up once and then has no further say over. This is the only way to guarantee a sample rate isn't jittered by whatever the CPU happened to be doing (an interrupt, a cache effect, anything) at the moment a software-triggered read would have been issued.

**Ping-pong buffering.** Two buffers, used alternately: while one is being filled (here, by DMA), the other, already full from the previous round, is available for something else (here, your code) to read, with no risk of reading data that's still being written. The instant the active buffer fills, the roles swap. This is how a system samples continuously with a bounded amount of memory, rather than needing one buffer that grows forever.

**Fixed-point arithmetic and Q15.** The Cortex-M0+ has no FPU (floating-point unit); every `float` or `double` operation is emulated in software, one instruction at a time, which is far too slow for filtering thousands of samples per second. The production answer on hardware like this is fixed-point: represent a fractional number as a plain integer that's implicitly been multiplied by some fixed scale factor. Q15 is one specific, extremely common convention: a 16-bit signed integer represents a real value from -1.0 to just under 1.0, with the integer equal to the real value times 32768 (2^15). `0.5` in Q15 is `16384`; `1.0` itself doesn't quite fit (the closest representable value is `32767`, or `0.99997`). This project uses Q15 specifically for filter coefficients, which are naturally small fractions.

**FIR filter and convolution.** An FIR (Finite Impulse Response) filter produces each output sample as a weighted sum of the current input sample and a fixed number of recent past samples, `y[n] = sum(k) h[k] * x[n-k]`, where the `h[k]` values are the filter's coefficients. That weighted-sum operation, applied at every sample, is called convolution. Which coefficients you choose determines what the filter does; Section 9 covers designing an actual low-pass set.

**Cutoff frequency.** The rough frequency above which a low-pass filter starts attenuating (reducing the amplitude of) a signal. Below the cutoff, the signal passes through close to unchanged; well above it, the signal is strongly reduced. "Rough" because a real filter's transition isn't a sharp wall, Section 9 shows the actual, computed, gradual roll-off this project's filter has.

## 2. Purpose

By the end of these two sessions, every member has: a DMA channel and DMAMUX configured to move ADC results into RAM with zero CPU polling; an ADC triggered by a TPM timer, not by software; a working ping-pong buffer swap proven by a UART dump of real, changing samples; an 8-tap Q15 FIR low-pass filter applied to every completed buffer; and a live, side-by-side plot of raw vs. filtered data running on a laptop, driven by data streamed from the board. This is the closest thing in the series to a real, shippable signal-acquisition pipeline.

## 3. Prerequisites

P1 and P2 complete. This manual assumes you're comfortable with ADC configuration and UART output (P1) and won't re-explain them; it also assumes the general habit, from every prior project, of reading interrupt-driven code as "hardware event sets a flag, main() reacts to the flag," which P3's ISR follows exactly.

## 4. Hardware and Register Reference

| Fact | Value | Verified against |
|---|---|---|
| Analog input pin | **PTE30** = ADC0_SE23, same channel P1 used | `driver_examples/adc16/polling`, reused directly from `P1_Project_Manual.md` |
| ADC hardware trigger source select | `SIM->SOPT7`, field `ADC0TRGSEL` (bits 3:0), value `0b1000` = TPM0 overflow | `MKL26Z4.xml`, the chip's own register description (see the full enumeration in Section 7) |
| ADC alternate trigger enable | `SIM->SOPT7`, bit `ADC0ALTTRGEN` = 1, required for `ADC0TRGSEL` to take effect at all | `MKL26Z4.xml`, confirmed working in `demo_apps/adc16_low_power_async_dma` |
| DMA peripheral on this chip | Simple 4-channel DMA (register names `SAR`/`DAR`/`DCR`/`DSR_BCR`), **not** the eDMA found on larger Kinetis parts | `fsl_dma.h`, `fsl_dma.c` |
| DMAMUX source for ADC0 | `kDmaRequestMux0ADC0` | `demo_apps/adc16_low_power_async_dma`, `fsl_dmamux.h` |
| DMA transfer function signature | `DMA_PrepareTransfer(config, srcAddr, srcWidth, destAddr, destWidth, transferBytes, type)` | `fsl_dma.h`, confirmed by reading the implementation |
| ADC result register address (DMA source) | `(uint32_t)(&ADC0->R[0])` | `demo_apps/adc16_low_power_async_dma` |
| Does `DMA_SetTransferConfig()` disturb the enable/async-request bits? | No; it only touches the size/increment fields in `DCR`, confirmed by reading `fsl_dma.c` directly | `fsl_dma.c` (`DMA_SetTransferConfig` implementation) |

> **This project resolves P0's open question.** P0 flagged that the FRDM-KL26Z's on-board "ambient light sensor," referenced in the original project spec, doesn't actually exist as a dedicated component anywhere in the SDK, and suggested the ADC0_SE23 header pin (PTE30) with an external photoresistor or potentiometer as the practical substitute. That's exactly what this project uses. If a leader has since confirmed there's no dedicated ALS on the board, this is the moment that substitution becomes load-bearing, not optional; wire an actual potentiometer to PTE30 before WS6, or the "signal" being filtered is just electrical noise on a floating pin, which still demonstrates the pipeline but makes for a much less convincing plot.

## 5. Additional Toolchain for This Project

Everything from prior projects still applies. New for P3: **Python 3** with **pyserial** and **matplotlib** (`pip3 install pyserial matplotlib`), needed for `demo_code/02_full_pipeline_with_fir/python/plot_fir_demo.py`. A potentiometer (or any variable analog voltage source, a photoresistor with a fixed resistor as a voltage divider works too) wired to PTE30 and ground/3.3V is strongly recommended; without one, the filter has nothing but noise to work on.

## Session 1 (WS6, Oct 22): DMA Configuration and Ping-Pong Setup

## 6. This Chip's DMA, and Why Ping-Pong Has to Be Done in Software

Larger Kinetis parts (the K-series) have an "eDMA" controller with hardware scatter-gather: you can chain multiple transfer descriptors together so the hardware itself automatically moves to the next buffer when one fills, with no CPU intervention at any point, including the handoff. The FRDM-KL26Z's MKL26Z128 has a simpler DMA controller, four channels, no descriptor chaining. Each channel has a source address register (SAR), destination address register (DAR), and a control register (DCR) that holds the transfer size and increment settings, plus a byte-count register (DSR_BCR) that counts down as the transfer proceeds and sets a "done" flag at zero.

This means true ping-pong, "the moment one buffer fills, start filling the other, automatically," isn't a single hardware mode you enable. It's a technique: when the DMA-complete interrupt fires, your ISR immediately reconfigures the channel (new destination address, reset byte count) to point at the *other* buffer, and restarts it, fast enough that no samples are lost between one buffer finishing and the next one starting. `demo_code/01_dma_adc_ping_pong/main.c`'s `DMA0_IRQHandler` does exactly this: it hands the buffer that just finished off to `main()` (Section 11's superloop-plus-flag pattern again), and in the same breath, points the DMA channel at the other buffer and calls `DMA_StartTransfer()` again.

**One subtlety worth knowing, confirmed by reading the driver source directly, not assumed:** `DMA_SetTransferConfig()` only touches the size and increment bits in DCR; it does not disturb the enable-request or async-request bits set once at initial configuration. That's why the ISR doesn't need to re-enable those every time, only reconfigure the addresses, clear the "done" flag, and restart.

**For the curious:** this DMA controller does have one genuine hardware circular-addressing feature (a "modulo" setting that makes an address wrap automatically within a power-of-two-sized region), which this project doesn't use. It solves a different problem (one continuously-wrapping buffer with a moving read/write pointer) than the two-fixed-buffers ping-pong pattern this project builds, and combining it correctly with this pipeline is a real, unexplored extension, not something this manual verifies.

## 7. Triggering the ADC from a Timer, Not from Code

Configuring the ADC itself (resolution, channel, reference voltage) is exactly what P1 covered; nothing new there. What's new is telling it to convert automatically:

```c
ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, true);
ADC16_EnableDMA(DEMO_ADC16_BASEADDR, true);
```

`EnableHardwareTrigger` means "don't wait for software to ask for a conversion, start one whenever the selected hardware trigger fires." `EnableDMA` means "when a conversion finishes, raise a DMA request instead of requiring the CPU to read the result." Together, these two lines are what let a conversion happen, and its result reach RAM, without a single line of your `main()` code being involved.

Which hardware signal actually triggers it is chosen in `SIM->SOPT7`, a register outside any peripheral driver, set directly:

```c
SIM->SOPT7 = (SIM->SOPT7 & ~SIM_SOPT7_ADC0TRGSEL_MASK) | SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN(1);
```

`ADC0TRGSEL` is a 4-bit field; the chip's register description lists several options, including `0b0100` (PIT trigger 0), `0b1000` (TPM0 overflow, what this project uses), `0b1001` (TPM1 overflow), and `0b1010` (TPM2 overflow), among others. `ADC0ALTTRGEN` has to be set to 1 for `ADC0TRGSEL`'s choice to actually take effect; without it, the ADC falls back to its default trigger source regardless of what `ADC0TRGSEL` says.

**Configuring TPM0 to produce that trigger** doesn't use PWM mode at all (contrast with P1, Section 8, which did):

```c
TPM_SetTimerPeriod(TPM0, (tpmClockHz / SAMPLE_RATE_HZ) - 1U);
TPM_StartTimer(TPM0, kTPM_SystemClock);
```

No channel, no output pin, nothing routed to a GPIO. TPM0 just free-runs and overflows (wraps back to zero) once every `MOD + 1` clock ticks; that overflow event is itself the signal `ADC0TRGSEL(8)` is listening for. At a 10 kHz target and whatever the TPM's actual source clock frequency is (queried at runtime with `CLOCK_GetFreq(kCLOCK_PllFllSelClk)`, not hardcoded, since that frequency depends on which clock mode is active), `MOD` works out to a few thousand ticks, comfortably within the 16-bit MOD register's range.

**Session 1 deliverable check:** ADC samples filling RAM continuously at 10 kHz, confirmed by the UART dump in `demo_code/01_dma_adc_ping_pong/main.c` showing real, changing values (not a frozen or all-zero buffer) each time a ping-pong swap happens.

## Session 2 (WS7, Oct 29): FIR Filter and Python Plot

## 8. Designing an 8-Tap Low-Pass Filter, With Real Numbers

Rather than picking coefficients by eye, `demo_code/02_full_pipeline_with_fir/main.c`'s filter was computed with a real windowed-sinc low-pass design, the standard, textbook technique for building a short FIR filter with a specific cutoff:

```python
import math

Fs = 10000.0   # sample rate, Hz
Fc = 500.0     # desired cutoff, Hz
N = 8          # taps
fc_norm = Fc / Fs

M = N - 1
h = []
for n in range(N):
    x = n - M / 2.0
    sinc = 2 * fc_norm if abs(x) < 1e-9 else math.sin(2 * math.pi * fc_norm * x) / (math.pi * x)
    w = 0.54 - 0.46 * math.cos(2 * math.pi * n / M)  # Hamming window
    h.append(sinc * w)

h_norm = [c / sum(h) for c in h]  # rescale so DC (0 Hz) passes with unity gain
q15 = [max(-32768, min(32767, round(c * 32768))) for c in h_norm]
print(q15)
```

Running this produces exactly the coefficients in the demo code:

```
Q15 coefficients: [570, 2006, 5445, 8363, 8363, 5445, 2006, 570]
Sum: 32768  (= 1.0 in Q15, i.e. unity gain at DC)
```

**Notice the coefficients are symmetric**, `h[0] == h[7]`, `h[1] == h[6]`, and so on. That's not a coincidence of this particular design; it's what makes this a *linear-phase* filter, meaning every frequency component gets delayed by the same amount of time passing through it, so the filtered waveform's shape is preserved (just smoothed and slightly delayed), rather than distorted the way an asymmetric filter's phase response could distort it.

**The filter's real, computed frequency response** (also worth verifying rather than assuming):

| Frequency | Attenuation |
|---|---|
| 0 Hz (DC) | 0.00 dB (unchanged) |
| 300 Hz | -0.32 dB (essentially unchanged) |
| 500 Hz (the design cutoff) | -0.89 dB |
| 1000 Hz | -3.60 dB |
| 1500 Hz | -8.27 dB |
| 2000 Hz | -15.04 dB |
| 3000 Hz | -35.37 dB |
| 5000 Hz (Nyquist, half the sample rate) | effectively zero |

This is a gentle, gradual roll-off, not a sharp wall, which is exactly what an 8-tap filter can and can't do: enough taps to meaningfully smooth a noisy signal, not enough to carve a razor-sharp cutoff (that would need many more taps, and correspondingly more multiply-accumulate work per sample). Tuning the cutoff (Session 2's "tune cutoff frequency and show the difference visually" deliverable) means changing `Fc` in the script above, regenerating coefficients, and rebuilding; try 1000 Hz or 200 Hz and compare the plot.

## 9. Applying the Filter in Fixed-Point, Across Buffer Boundaries

`ApplyFirFilter()` in `demo_code/02_full_pipeline_with_fir/main.c` implements `y[n] = sum(k=0..7) h[k] * x[n-k]` directly:

```c
int32_t acc = 0;
for (k = 0U; k < FIR_TAPS; k++)
{
    int32_t idx = (int32_t)n - (int32_t)k;
    uint32_t sample = (idx >= 0) ? rawBuffer[idx] : g_firHistory[(FIR_TAPS - 1U) + idx];
    acc += g_firCoeffsQ15[k] * (int32_t)sample;
}
filteredBuffer[n] = (uint32_t)(acc >> 15);
```

Two details worth understanding, not just copying:

**The accumulator is 32-bit, and the `>> 15` (equivalent to dividing by 32768) happens exactly once, after all 8 taps are summed, not after each individual multiply.** Each `coefficient * sample` product is a full-precision 32-bit value; shifting after every tap would throw away low-order bits 8 separate times, accumulating rounding error. Shifting once, at the end, keeps the accumulated sum at full precision until the very last step, giving a materially more accurate result for the same amount of code.

**The first 7 samples of every new buffer need history from the *previous* buffer, or every buffer boundary produces an audible or visible glitch.** Convolution needs each sample's 7 predecessors; for sample 0 of a new buffer, those predecessors are the last 7 samples of whichever buffer was filled before it, not zeros and not garbage. `g_firHistory[]` is a small, 7-element array that persists across calls specifically to hold those samples, updated at the end of every `ApplyFirFilter()` call:

```c
for (k = 0U; k < (FIR_TAPS - 1U); k++)
{
    g_firHistory[k] = rawBuffer[count - (FIR_TAPS - 1U) + k];
}
```

The very first buffer the board ever processes has no real prior history (`g_firHistory` starts at all zeros), which causes one brief, expected startup transient on the first handful of samples, exactly like any real filter the instant after power-on. This is normal, not a bug to chase.

## 10. The UART Bandwidth Budget, and Why the Output Is Decimated

This is a real constraint, not a simplification for the sake of the demo. At 115200 baud, 8 data bits, 1 start bit, 1 stop bit (10 bits per byte), the UART can move at most `115200 / 10 = 11520` bytes per second. Printing every single sample at the full 10 kHz acquisition rate, even at a lean 12 to 13 bytes per line (`"1234,1180\r\n"`), would need roughly `10000 * 13 = 130000` bytes per second, more than ten times what the link can carry. Acquisition and filtering both genuinely run at the full 10 kHz internally, with zero CPU polling; what's actually limited is how much of that stream can be reported back to a laptop over a slow serial link, which is a completely different, and very real, bottleneck.

`demo_code/02_full_pipeline_with_fir/main.c` handles this by only printing every 16th sample (`DECIMATION`):

```
reported rate = 10000 Hz / 16 = 625 Hz
bytes/sec ≈ 625 * 13 ≈ 8125 bytes/sec, comfortably under the 11520 byte/sec budget
```

If you change `DECIMATION`, redo this arithmetic; a value too small will overrun the UART and the Python script will see truncated, unparseable lines (Section 17 covers what that looks like and how to recognize it).

## 11. Running the Python Real-Time Plot

```bash
pip3 install pyserial matplotlib   # once
python3 demo_code/02_full_pipeline_with_fir/python/plot_fir_demo.py /dev/tty.usbmodemXXXX
```

(Use the board's actual serial device name; on Windows this looks like `COM5` instead.) The script opens the port at 115200 baud, reads whatever complete lines are available on each animation frame, splits each on the comma, and silently skips anything that isn't exactly two valid integers (the startup banner text, for instance), so it doesn't need to know or care what non-data text the board might print. It keeps a rolling window of the most recent 625 points (roughly one second at this project's decimated reporting rate) and plots raw and filtered side by side, live.

**The CSV-parsing and rolling-window logic in this script was unit-tested in isolation** (feeding it a mix of banner text, malformed lines, and valid CSV pairs, and confirming the rolling window ends up holding exactly the right values); the plotting itself, which depends on `matplotlib`, could not be end-to-end tested in the environment this was written in. Bench-test the full script against a real board before WS7.

## 12. Code Structure Explanation

| File | What It Does |
|---|---|
| `01_dma_adc_ping_pong/main.c` | `ConfigureAdc()`, `ConfigureAdcHardwareTrigger()`, `ConfigureTpmTrigger()`, `ConfigureDma()` (Sections 6 to 7), and `DMA0_IRQHandler()`, the actual ping-pong swap. `main()` just dumps a few raw samples per completed buffer. |
| `02_full_pipeline_with_fir/main.c` | The same acquisition setup, plus `g_firCoeffsQ15[]` (Section 8), `ApplyFirFilter()` (Section 9), and a `main()` loop that filters each completed buffer and prints a decimated `raw,filtered` CSV stream (Section 10). |
| `python/plot_fir_demo.py` | Reads that CSV stream over serial and plots both signals live, side by side, in a rolling window (Section 11). |

## 13. Sample Output

Session 1, raw dump:

```
=== P3 DMA ping-pong reference (Session 1: raw dump only) ===
Sampling ADC0_SE23 (PTE30) at 10000 Hz, 64 samples per buffer.

buffer 0 ready (capture #1), first 8 raw samples: 31204 31198 31301 31150 31266 31189 31240 31177
buffer 1 ready (capture #2), first 8 raw samples: 31212 31220 31195 31264 31183 31207 31251 31166
```

(Exact values depend on whatever's actually wired to PTE30; a potentiometer sitting still should show similar values buffer to buffer, with small variation from real electrical noise. A floating pin will look far noisier and less consistent.)

Session 2, decimated CSV stream (what the Python script consumes):

```
=== P3 full pipeline reference (Session 2: FIR + Python plot) ===
Sampling ADC0_SE23 (PTE30) at 10000 Hz, printing every 16th sample as raw,filtered.
Point python/plot_fir_demo.py at this board's serial port to see it live.

31204,29850
31266,30012
31183,30105
31240,30188
```

The plot itself should show the left (raw) panel visibly noisier, jumping sample to sample, and the right (filtered) panel tracking the same overall trend but noticeably smoother, with high-frequency jitter visibly reduced. Turning the potentiometer slowly should move both panels together; turning it quickly (introducing higher-frequency content) should show the filtered panel lagging and smoothing that motion more than the raw panel does, direct, visible evidence of Section 8's attenuation numbers.

## 14. Session Plan (maps to Guideline Section 4.6)

| Meeting | Phase | What Members Do | Deliverable | Slide Focus |
|---|---|---|---|---|
| 1 of 2 | DMA config and ping-pong setup | Configure DMAMUX, the DMA transfer, and the ADC's hardware trigger (Sections 6 to 7). Implement the ping-pong buffer swap in the DMA-complete ISR. Verify by UART-dumping raw buffer contents. | ADC samples filling RAM continuously at 10 kHz. CPU load near zero during acquisition. UART dump of raw buffer values confirmed correct. | Terminal screenshot of the raw dump; what broke and how it was fixed; a one-line explanation of why the trigger is TPM-driven, not software. |
| 2 of 2 | FIR filter and Python plot | Apply the Q15 FIR convolution to each ready buffer (Sections 8 to 9). Stream raw and filtered output over UART at 115200 baud. Run `plot_fir_demo.py`. Tune the cutoff frequency and show the difference visually. | Real-time plot showing raw noisy signal and clean filtered output side by side. Filter cutoff effect visible. | Live demo is the whole slide; be ready to explain the decimation math (Section 10) if asked, since it's a real design decision, not an implementation detail. |

## 15. Milestones and Success Criteria

| Milestone | Success Criteria | Evidence |
|---|---|---|
| ADC hardware-triggered, not software-triggered | Samples continue arriving with no code in `main()` calling an ADC "start conversion" function | Code review by leader |
| DMA ping-pong (Session 1 exit criteria) | Buffers alternate correctly; UART dump shows real, changing data at both indices | Terminal screenshot |
| Zero CPU polling during acquisition | `main()`'s loop only checks a flag, never spins on an ADC or DMA status register | Code review by leader |
| FIR filter applied correctly | Filtered output visibly smoother than raw in the terminal dump, before even opening Python | Terminal screenshot |
| Full pipeline with live plot (Session 2 exit criteria) | Python plot shows raw vs. filtered side by side, live, with visible attenuation of fast changes in the filtered panel | Live demo plus screenshot of the plot |

## 16. Project-Specific Debugging Reference

| # | Common Problem | Suggested Debugging Steps | Difficulty |
|---|---|---|---|
| 1 | Buffers fill once, then stop; no further UART output | The DMA-complete ISR isn't re-arming correctly. Confirm `DMA0_IRQHandler` is actually being entered (add a temporary `PRINTF` at its very start) and that `DMA_ClearChannelStatusFlags()` is called before reconfiguring; a still-set DONE flag can block the next transfer. | Medium |
| 2 | Both buffers show identical or frozen values | `ADC0TRGSEL`/`ADC0ALTTRGEN` aren't set correctly, so the ADC is still on its default (likely non-firing) trigger source, and DMA is copying whatever stale value sits in the result register repeatedly. Re-check the exact `SIM->SOPT7` write against Section 7. | Medium |
| 3 | Samples look like pure noise with no relation to the potentiometer position | Nothing is actually wired to PTE30, or the wiring is loose; this is expected behavior on a floating analog pin, not a software bug. Confirm the physical connection before debugging code. | Easy |
| 4 | Filtered output looks identical to raw, no smoothing visible | The FIR coefficients sum to something other than 32768 (broken unity gain), or `acc >> 15` is being applied per-tap instead of once at the end (Section 9), effectively discarding most of the filter's effect. Recompute the coefficients with the Section 8 script and compare against the known-correct values. | Medium |
| 5 | A visible glitch or spike appears at regular intervals in the filtered signal | `g_firHistory` isn't being updated, or is being read/written in the wrong order, so every buffer boundary filters against stale or zeroed history instead of the true preceding samples. Confirm the history-update loop runs after, not before, the main filtering loop in `ApplyFirFilter()`. | Hard |
| 6 | Python script shows a blank or frozen plot | Confirm the serial port name is correct and nothing else (like the IDE's own terminal) has it open at the same time, only one program can hold a serial port open at once. Check `ser.in_waiting` is actually growing by adding a temporary `print()` in `read_available_lines()`. | Easy |
| 7 | Python script's plot updates but looks garbled or the values jump wildly | The UART is likely overrunning its bandwidth budget (Section 10); confirm `DECIMATION` wasn't lowered without redoing the arithmetic, and check for dropped/split lines by temporarily printing raw `readline()` output before parsing. | Medium |
| 8 | Build fails referencing `SIM_SOPT7_ADC0TRGSEL` or `SIM_SOPT7_ADC0ALTTRGEN` as undefined | Confirm `fsl_device_registers.h` (pulled in transitively through `board.h`) is actually included; these are chip-specific register macros, not part of a peripheral driver header, so a project missing the base device header won't see them. | Easy |

## 17. Glossary

- **DMA (Direct Memory Access):** hardware that copies data between memory and peripheral registers without CPU instructions executing per byte.
- **DMAMUX:** the peripheral that routes a specific hardware event (here, an ADC conversion completing) to a specific DMA channel.
- **SAR / DAR / DCR / DSR_BCR:** this chip's DMA channel registers: source address, destination address, control (size, increment, enable bits), and byte-count/status, respectively.
- **Cycle-steal mode:** a DMA mode where exactly one transfer happens per trigger request, rather than the whole transfer completing on a single request; required here so each ADC conversion moves exactly one sample.
- **Hardware trigger:** a conversion (or other action) initiated directly by another piece of hardware, with no CPU instruction causing it at the moment it happens.
- **Ping-pong buffering:** two buffers used alternately, so one can be read while the other is being written, with no collision.
- **Fixed-point arithmetic:** representing fractional values as integers implicitly scaled by a fixed factor, used to avoid floating-point on hardware without an FPU.
- **Q15:** a specific fixed-point format, 16-bit signed integers representing real values from -1.0 to just under 1.0, scaled by 32768.
- **FIR filter (Finite Impulse Response):** a filter whose output is a weighted sum of a fixed number of current and past input samples.
- **Convolution:** the weighted-sum operation an FIR filter performs at every sample.
- **Cutoff frequency:** the rough frequency above which a low-pass filter begins meaningfully attenuating a signal.
- **Linear phase:** a filter property, guaranteed here by symmetric coefficients, where every frequency component is delayed by the same amount, preserving the filtered waveform's shape.
- **Decimation (in this context):** printing only every Nth sample to fit a UART's real bandwidth limit, distinct from decimation as a signal-processing downsampling technique, though the two are related.

## 18. References

- `demo_apps/adc16_low_power_async_dma` (in `SDK_2_2_0_FRDM-KL26Z`): the verified, working reference combining ADC hardware triggering, DMA, and DMAMUX on this exact chip.
- `MKL26Z4.xml` (in `SDK_2_2_0_FRDM-KL26Z/devices/MKL26Z4`): the chip's own register description, source of the `ADC0TRGSEL` enumeration in Section 7.
- `fsl_dma.h`, `fsl_dma.c`, `fsl_dmamux.h` (in the SDK): the DMA and DMAMUX driver source, ground truth for every function signature in this manual.
- `P1_Project_Manual.md` (this repository): the ADC configuration and PTE30/ADC0_SE23 pin this project reuses without re-deriving.
- Any standard digital signal processing reference covering windowed-sinc FIR filter design, for going deeper than Section 8's script.

## 19. Developer Notes

- **Both `demo_code/01_dma_adc_ping_pong/` and `02_full_pipeline_with_fir/` were compiled successfully against the real toolchain** (17 KB and 17.5 KB of 128 KB flash respectively), with zero warnings from this project's own code. Neither has been flashed to physical hardware. A project leader must bench-test both, with a real potentiometer wired to PTE30, before WS6/WS7; this project's failure modes tend to be silent (flat or noisy-looking data) rather than crashes, which makes a known-good reference run more valuable here than for most prior projects.
- The Python script's data-handling logic (CSV parsing, malformed-line skipping, rolling-window behavior) was unit-tested standalone with synthetic input; the live-plotting path itself, which depends on `matplotlib`, was not runnable in the environment this was authored in and needs a leader's real-hardware check before WS7.
- This project is the concrete resolution to the ambient-light-sensor question P0 first flagged and P1 deferred; see the callout in Section 4. If chapter leadership confirms there genuinely is no dedicated ALS on the board, it may be worth updating P0's and P1's manuals to point here directly instead of leaving the question open in three separate documents.
- If a future capstone wants a sharper filter (steeper roll-off, more taps), the design script in Section 8 generalizes directly, just increase `N`; the `ApplyFirFilter()` C code would need `FIR_TAPS` and `g_firHistory[]`'s size updated to match, but the algorithm itself doesn't change.

---
*IEEE Texas State University Student Branch. Connect. Build. Inspire.*
