# P3: Reference

*Part of the P3 manual split. See `P3_1_Start_Here.md` for the full file list and how to use this manual.*

---

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
- P1's manual (`P1_Sensor_Dashboard/`, this repository): the ADC configuration and PTE30/ADC0_SE23 pin this project reuses without re-deriving.
- Any standard digital signal processing reference covering windowed-sinc FIR filter design, for going deeper than Section 8's script.

## 19. Developer Notes

- **Both `demo_code/01_dma_adc_ping_pong/` and `02_full_pipeline_with_fir/` were compiled successfully against the real toolchain** (17 KB and 17.5 KB of 128 KB flash respectively), with zero warnings from this project's own code. Neither has been flashed to physical hardware. A project leader must bench-test both, with a real potentiometer wired to PTE30, before WS6/WS7; this project's failure modes tend to be silent (flat or noisy-looking data) rather than crashes, which makes a known-good reference run more valuable here than for most prior projects.
- The Python script's data-handling logic (CSV parsing, malformed-line skipping, rolling-window behavior) was unit-tested standalone with synthetic input; the live-plotting path itself, which depends on `matplotlib`, was not runnable in the environment this was authored in and needs a leader's real-hardware check before WS7.
- This project is the concrete resolution to the ambient-light-sensor question P0 first flagged and P1 deferred; see the callout in Section 4. If chapter leadership confirms there genuinely is no dedicated ALS on the board, it may be worth updating P0's and P1's manuals to point here directly instead of leaving the question open in three separate documents.
- If a future capstone wants a sharper filter (steeper roll-off, more taps), the design script in Section 8 generalizes directly, just increase `N`; the `ApplyFirFilter()` C code would need `FIR_TAPS` and `g_firHistory[]`'s size updated to match, but the algorithm itself doesn't change.

---
*IEEE Texas State University Student Branch. Connect. Build. Inspire.*
