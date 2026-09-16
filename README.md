# P3: DMA-Driven ADC Pipeline with Fixed-Point FIR Filter

*CPU-free data acquisition, ping-pong buffering, and real-time signal processing.*

IEEE TXST Student Branch, Fall 2026 FRDM-KL26Z project series.

**Platform:** NXP FRDM-KL26Z (ARM Cortex-M0+, 48 MHz, 128 KB Flash) with a potentiometer (roughly $1) or the on-board ambient light sensor. Prerequisite: P1 and P2 complete.

## What this project is

Members configure the KL26Z's DMA controller to fill two alternating RAM buffers ("ping-pong") with ADC samples triggered by a hardware timer, with zero CPU involvement in acquisition. While DMA fills one buffer, the CPU processes the other: applying an 8-tap fixed-point (Q15) FIR low-pass filter. Filtered and raw samples stream over UART and get plotted live in Python. This is the architecture behind every audio codec, motor controller, and data acquisition system in industry.

## Exit criteria

DMA filling ping-pong buffers at 10 kHz with zero CPU polling. FIR filter applied to each completed buffer. Python plot showing raw vs. filtered signal live, with visible attenuation above the cutoff frequency.

## Repo layout

- `P3_1_Start_Here.md` through `P3_4_Reference.md`: the Project Manual, split into four files.
- `demo_code/01_dma_adc_ping_pong/`: TPM-triggered ADC conversions moved into RAM by DMA, ping-ponging between two buffers, no filtering yet.
- `demo_code/02_full_pipeline_with_fir/`: the same acquisition pipeline plus the 8-tap Q15 FIR low-pass filter and a decimated raw/filtered UART stream.
  - `python/plot_fir_demo.py`: live matplotlib plot of raw vs. filtered signal.
- `demo_code/_reference_drivers/`: vendored NXP SDK driver source, reference only.

## Getting started

Read `P3_1_Start_Here.md` first, then `P3_3_Setup_and_Walkthrough.md`. Get Session 1's ping-pong acquisition working and understood before moving to the FIR filter.

## Resume line

> Designed DMA ping-pong ADC pipeline on ARM Cortex-M0+ with real-time Q15 fixed-point FIR filtering at 10 kHz sample rate; zero CPU involvement in data acquisition.
