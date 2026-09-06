# P3 Demo Code

Reference material only. Try building each stage yourself from the Project Manual first;
open these only if you get stuck and can't figure out why.

- `01_dma_adc_ping_pong/`: Session 1. TPM0 triggers the ADC in hardware at 10 kHz; DMA moves
  every result straight into RAM with zero CPU involvement; `DMA0_IRQHandler` alternates
  between two buffers (software-managed ping-pong, since this chip's DMA has no built-in
  scatter-gather). `main()` just dumps the first few raw samples of each completed buffer over
  UART, to prove the pipeline is really running continuously. No filtering yet.

- `02_full_pipeline_with_fir/`: Session 2. Same acquisition pipeline, plus an 8-tap Q15
  fixed-point FIR low-pass filter applied to each completed buffer, and a decimated
  `raw,filtered` CSV stream over UART. `python/plot_fir_demo.py` reads that stream and plots
  both signals live, side by side.

**Both firmware projects were compiled and checked, not just written.** They build cleanly
against `SDK_2_2_0_FRDM-KL26Z` (17 KB and 17.5 KB of the 128 KB flash budget respectively,
comfortably under), and every DMA/ADC/TPM register value was verified against either a working
NXP SDK example (`demo_apps/adc16_low_power_async_dma`) or the chip's own register-description
file (`MKL26Z4.xml`), not guessed. Neither has been flashed to physical hardware; a project
leader must bench-test both on a real board (with an actual potentiometer or other analog
source wired to PTE30, see the manual, Section 4) before WS6/WS7.

The Python script's CSV-parsing and rolling-window logic was unit-tested standalone (its
`matplotlib` dependency isn't installed in the environment this was written in, so the plotting
itself couldn't be smoke-tested end to end); install `pyserial` and `matplotlib` before running
it (`pip3 install pyserial matplotlib`).

To build the firmware, see the P0 manual, Section 8, for the general `cmake` / `make` /
`objcopy` flow; each `armgcc/` folder here already has its build scripts patched for this
repository's folder layout, the same fixes every prior project's combined reference has needed.
