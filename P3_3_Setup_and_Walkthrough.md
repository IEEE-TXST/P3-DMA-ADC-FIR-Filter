# P3: Setup and Walkthrough

*Part of the P3 manual split. See `P3_1_Start_Here.md` for the full file list and how to use this manual.*

**Before you start writing code:** create one new project for this entire project, via MCUXpresso's SDK wizard (P0 manual, Section 9): device `MKL26Z128VLH4`, board files = **Default board files**, project type = **C Project**, SDK Debug Console = **UART**. Every `demo_code/` folder mentioned below is something to read and copy logic from, never something to build on its own; a wizard-created project gives you MCUXpresso's normal managed build (no CMake, no `armgcc`, no relative-path setup), the same smooth build experience as any other wizard project in this series.

---

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

---

**Next:** `P3_4_Reference.md` for code structure, sample output, debugging, and the glossary.

---
*IEEE Texas State University Student Branch. Connect. Build. Inspire.*
