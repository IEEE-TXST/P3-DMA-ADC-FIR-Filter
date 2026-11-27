#!/usr/bin/env python3
"""
P3 real-time plot: reads "raw,filtered\r\n" CSV lines from the board's
UART (see 02_full_pipeline_with_fir/main.c) and plots both side by side,
live, using matplotlib.

Any line that isn't exactly two comma-separated integers (the startup
banner text, for instance) is silently skipped, not treated as an error.

Usage:
    python3 plot_fir_demo.py <serial-port> [baud]

Examples:
    python3 plot_fir_demo.py /dev/tty.usbmodem14203
    python3 plot_fir_demo.py COM5 115200

Install dependencies first if needed:
    pip3 install pyserial matplotlib
"""
# A desktop companion to 02_full_pipeline_with_fir/main.c: reads the
# board's decimated "raw,filtered" CSV stream over serial and draws both
# signals live, side by side, so the FIR filter's smoothing effect is
# visible directly instead of just inferred from printed numbers. A
# pyserial connection reads whatever full lines have arrived each
# animation frame, without blocking; each line's two numbers go into two
# fixed-size rolling buffers that matplotlib redraws roughly 20x/second.
# The firmware can only print text, it has no way to draw a graph itself,
# so offloading visualization to a desktop script is the standard split
# for embedded work: the constrained device streams numbers, a real
# computer turns them into a picture.
import sys
import collections

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# How many points to keep on screen. At the firmware's default decimation
# (every 16th sample of a 10 kHz stream), the board reports about 625
# points/second, so 625 points here is roughly a 1-second rolling window.
WINDOW_POINTS = 625


def parse_args():
    """Reads the serial port and optional baud rate from the command line;
    baud defaults to 115200 to match the firmware's console. Printing the
    module's own docstring as the usage message means it only needs to be
    written once, at the top of the file."""
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    return port, baud


def main():
    port, baud = parse_args()
    # timeout=0.1 keeps any single read from blocking for long, since
    # read_available_lines() below runs inside the animation loop and must
    # return promptly even if the board briefly stops sending.
    ser = serial.Serial(port, baud, timeout=0.1)

    # Two fixed-capacity rolling buffers, one per signal. deque(maxlen=...)
    # discards the oldest point automatically once full, exactly what a
    # live scrolling plot needs, with no manual trimming logic.
    raw_data = collections.deque(maxlen=WINDOW_POINTS)
    filtered_data = collections.deque(maxlen=WINDOW_POINTS)

    fig, (ax_raw, ax_filtered) = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.suptitle("P3: raw ADC samples vs. 8-tap FIR filtered output")

    (raw_line,) = ax_raw.plot([], [], color="tab:red", linewidth=0.8)
    ax_raw.set_title("Raw (noisy)")
    ax_raw.set_xlabel("sample (decimated)")
    ax_raw.set_ylabel("ADC counts (16-bit)")

    (filtered_line,) = ax_filtered.plot([], [], color="tab:blue", linewidth=1.2)
    ax_filtered.set_title("Filtered (8-tap low-pass, 500 Hz cutoff)")
    ax_filtered.set_xlabel("sample (decimated)")

    for ax in (ax_raw, ax_filtered):
        ax.set_ylim(0, 65535)  # full 16-bit ADC range; narrow this if your signal sits in a smaller band

    def read_available_lines():
        """Drain whatever full lines are currently waiting, without blocking
        the animation loop waiting for more."""
        # ser.in_waiting reports bytes already arrived; looping on it drains
        # everything available right now and stops as soon as the buffer is
        # empty, instead of waiting for a byte that hasn't arrived yet.
        lines = []
        while ser.in_waiting:
            raw_line_bytes = ser.readline()
            try:
                text = raw_line_bytes.decode("ascii", errors="ignore").strip()
            except UnicodeDecodeError:
                continue
            if text:
                lines.append(text)
        return lines

    def update(_frame):
        # Called by matplotlib roughly every 50ms; ingests new lines and
        # redraws. Skips any line that isn't exactly two integers (startup
        # banner text, a corrupted line) rather than raising, since serial
        # data is inherently unreliable, dropping one bad point beats
        # crashing the whole live plot.
        for line in read_available_lines():
            parts = line.split(",")
            if len(parts) != 2:
                continue  # not a data line (startup banner, etc.); skip it
            try:
                raw_value = int(parts[0])
                filtered_value = int(parts[1])
            except ValueError:
                continue  # malformed line; skip it, don't crash the plot
            raw_data.append(raw_value)
            filtered_data.append(filtered_value)

        # set_data feeds new x/y arrays into the existing Line2D objects
        # (created once above) instead of creating new plot objects every
        # frame; set_xlim grows the visible range as points accumulate.
        raw_line.set_data(range(len(raw_data)), raw_data)
        ax_raw.set_xlim(0, max(len(raw_data), 1))

        filtered_line.set_data(range(len(filtered_data)), filtered_data)
        ax_filtered.set_xlim(0, max(len(filtered_data), 1))

        return raw_line, filtered_line

    # blit=False because both axes' x-limits change every frame as the
    # window fills; blitting only redraws the changed artists, which looks
    # wrong when the axes themselves are also moving. cache_frame_data=False
    # avoids matplotlib caching every past frame, which would grow memory
    # without bound for a live plot meant to run indefinitely.
    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    ser.close()


if __name__ == "__main__":
    main()
