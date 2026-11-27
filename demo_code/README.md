# P3 Demo Code

These are **step-by-step code files**, not answer keys. The code is already in each file, but
switched off (commented out). You switch it on one step at a time, run it on the board, and do
a small exercise. All the explanation is written inside the file itself.

## How to use a demo

1. Create a new project with the SDK wizard. The P0 manual, Section 9, Step 1, shows exactly how.
2. Open the demo's `main.c`, copy everything, and paste it over your project's main source file.
3. Replace your project's `pin_mux.c` with the `pin_mux.c` in the same demo folder. If you forget
   this step the program may still build and run, but the analog input may not be set up.
4. Read the first step in the file, select its grey `//` lines, and press **Cmd + /** (Mac) or
   **Ctrl + /** (Windows) to turn them on. Build, flash, and watch the terminal. Do the
   exercise and move to the next step.

Lines inside `/* ... */` boxes are explanation. Leave them alone.

## What is here

- `01_dma_adc_ping_pong/main.c`: Session 1. 9 steps. A timer triggers the ADC in hardware at
  10 kHz, DMA moves every result into RAM with no CPU involvement, and two buffers alternate
  (software ping-pong, because this chip's DMA has no built-in scatter-gather). The first 4
  steps build the pieces, the last 5 start them and print what was captured.
- `02_full_pipeline_with_fir/main.c`: Session 2. **The Demo 1 pipeline is already switched on
  in this file**, because you just built it. Only the filter is switched off, in 3 steps: the
  8-tap Q15 fixed-point FIR coefficients, the filter function, and the call that turns it on.
  - `python/plot_fir_demo.py` reads the `raw,filtered` stream and plots both signals live. Install
    `pyserial` and `matplotlib` first (`pip3 install pyserial matplotlib`).

You need an analog signal on PTE30: a potentiometer (about $1) or the on-board ambient light
sensor. See the manual, Section 4.

## For project leaders

- The steps have not yet been built and flashed one by one on real hardware. The DMA, ADC and
  timer register values were checked against a working NXP example
  (`demo_apps/adc16_low_power_async_dma`) and the chip's register description file in the
  earlier version. Please bench-test both demos with a real analog source before the session.
- The Python script's CSV parsing was unit-tested standalone, but its live plot has not been
  smoke-tested end to end.
