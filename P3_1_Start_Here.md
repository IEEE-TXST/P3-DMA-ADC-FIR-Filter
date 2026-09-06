# TXST IEEE Student Branch: FRDM-KL26Z Project Series
## Project Manual, P3: DMA-Driven ADC Pipeline with Fixed-Point FIR Filter

**Document status:** DRAFT v0.1, for project-leader review and bench testing before member use
**Track:** Embedded Systems | **Difficulty:** Advanced | **Sessions:** 2 (WS6, Oct 22 and WS7, Oct 29, 2026), with pre-reading assigned Oct 15
**Applies to:** All 8 groups (24 members)
**Companion documents:** P0's manual (`P0_Board_Orientation_and_Toolchain_Setup/`, start at `P0_1_Start_Here.md`), P1's manual (`P1_Sensor_Dashboard/`), P2's manual (`P2_Cooperative_Task_Scheduler/`) (read all three first), *TXST IEEE FRDM-KL26Z Project Specification*
**Hardware:** FRDM-KL26Z plus a potentiometer (roughly $1) wired to PTE30, or the on-board ADC header pin left floating for a noisier demo
**Demo code:** see `demo_code/` in this project's folder

---

## This Manual Is Split Into 4 Files

Long single files invite procrastination. Read only what you need, when you need it:

1. **`P3_1_Start_Here.md`** (this file) — how to use this manual, why P3 exists, purpose, prerequisites.
2. **`P3_2_Concepts_and_Hardware.md`** — background theory (DMA, DMAMUX, hardware triggers, ping-pong buffering, fixed-point/Q15, FIR filters) and the hardware/register reference. Read once if any term below is new to you.
3. **`P3_3_Setup_and_Walkthrough.md`** — the actual hands-on steps for both sessions. **This is the file you follow during the sessions.**
4. **`P3_4_Reference.md`** — code structure explanation, sample output, session plan, milestones, debugging table, glossary, references, developer notes. Look things up here when stuck.

Section numbers (0-19) are kept consistent across all 4 files, so "see Section 9" always means the same section no matter which file you're in.

---

## How to Use This Manual

Same rule as every prior manual: a reference, not required reading. Build each stage hands-on and open this only when stuck.

Project leaders: bench-test both `demo_code/01_dma_adc_ping_pong/` and `demo_code/02_full_pipeline_with_fir/` on a real board before WS6 and WS7, with an actual potentiometer wired to PTE30 (Section 4). This project's failure modes are quieter than P2's, a misconfigured trigger source doesn't hard-fault, it just silently produces flat or garbage data, so seeing it work correctly once, on real hardware, before the session matters more here than the raw difficulty might suggest.

**A note on accuracy:** every DMA, DMAMUX, and ADC-trigger register value in this manual was verified against either a working NXP SDK example (`demo_apps/adc16_low_power_async_dma`, which combines exactly these three peripherals) or the chip's own register-description file (`MKL26Z4.xml`), which lists the ADC's hardware trigger source options directly. The FIR filter's coefficients were computed with a real filter-design script (Section 9 includes it), not chosen by eye, and its frequency response was verified by computing it directly, not assumed. Section 20 has the full verification trail.

---

## 0. Why This Session Exists

P1 read an ADC by asking for a value and waiting. P2 built a scheduler that lets several tasks share a CPU. P3 removes the CPU from the acquisition loop entirely: a timer and a DMA controller, working together in hardware, sample a signal at a precise, guaranteed rate while the processor does something else, or nothing at all, until there's real work (filtering) to do. This is not a niche technique. It's the actual architecture behind every audio codec, motor controller, and data-acquisition system built on a microcontroller, because it's the only way to guarantee sample timing isn't at the mercy of whatever else the CPU happens to be doing. P2's scheduler taught you that CPU time is a resource tasks compete for; P3 teaches you how to take an entire job (sampling) off that competition altogether.

## 2. Purpose

By the end of these two sessions, every member has: a DMA channel and DMAMUX configured to move ADC results into RAM with zero CPU polling; an ADC triggered by a TPM timer, not by software; a working ping-pong buffer swap proven by a UART dump of real, changing samples; an 8-tap Q15 FIR low-pass filter applied to every completed buffer; and a live, side-by-side plot of raw vs. filtered data running on a laptop, driven by data streamed from the board. This is the closest thing in the series to a real, shippable signal-acquisition pipeline.

## 3. Prerequisites

P1 and P2 complete. This manual assumes you're comfortable with ADC configuration and UART output (P1) and won't re-explain them; it also assumes the general habit, from every prior project, of reading interrupt-driven code as "hardware event sets a flag, main() reacts to the flag," which P3's ISR follows exactly.

---

**Next:** `P3_2_Concepts_and_Hardware.md` for the concepts and register reference, or skip straight to `P3_3_Setup_and_Walkthrough.md` if you're already comfortable with DMA and fixed-point arithmetic.

---
*IEEE Texas State University Student Branch. Connect. Build. Inspire.*
