#!/usr/bin/env python3
"""
Coast-down test logger for the DC motor rig (pairs with coastdown_test.cpp).

For each requested PWM level and repeat, the script tells the Arduino to spin the
motor up, cut the drive (coast), and log encoder speed until the motor stops.
All runs are written to a single CSV file.

Usage:
    pip install pyserial          (and matplotlib if you want --plot)
    python coastdown_logger.py --port COM3 --pwm 120 180 255 --runs 3 --spinup 5000 --plot
    python coastdown_logger.py --port /dev/ttyACM0 --pwm 200

CSV columns:
    run         run index (1, 2, ...)
    pwm         PWM used during spin-up
    phase       0 = spin-up (driven), 1 = coasting
    t_s         sample time, seconds, relative to the drive cut (negative = before cut)
    edge_t_s    time of the most recent encoder edge, seconds, relative to the cut
    count       cumulative encoder edge count (raw)
    new_edges   edges since the previous sample; 0 means the rpm on that row is
                only an upper bound, not a measurement (filter these out for fitting)
    rpm         speed estimate from the Arduino
    omega_rad_s same speed in rad/s
"""

import argparse
import csv
import math
import sys
import time
from datetime import datetime

import serial

CSV_HEADER = ["run", "pwm", "phase", "t_s", "edge_t_s", "count",
              "new_edges", "rpm", "omega_rad_s"]


def read_line(ser):
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("ascii", errors="replace").strip()


def wait_for_ready(ser, timeout=6.0):
    """Opening the port resets most Arduinos; wait for the boot message."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser)
        if line == "#READY":
            return True
    return False


def run_one(ser, pwm, spinup_ms, max_coast_s=65.0):
    """Run a single coast-down test and return (rows, end_reason)."""
    ser.reset_input_buffer()
    ser.write(f"RUN {pwm} {spinup_ms}\n".encode("ascii"))

    samples = []
    cut_us = None
    reason = None
    deadline = time.time() + spinup_ms / 1000.0 + max_coast_s + 5.0

    while time.time() < deadline:
        line = read_line(ser)
        if not line:
            continue
        if line.startswith("D,"):
            parts = line.split(",")
            if len(parts) != 6:
                continue  # corrupted line
            try:
                t_us, phase, count, edge_us = (int(p) for p in parts[1:5])
                rpm = float(parts[5])
            except ValueError:
                continue
            samples.append((t_us, phase, count, edge_us, rpm))
        elif line.startswith("#CUT,"):
            cut_us = int(line.split(",")[1])
        elif line.startswith("#END,"):
            reason = line.split(",")[1]
            break
        elif line.startswith("#ERR,"):
            raise RuntimeError(f"Arduino error: {line[5:]}")
        # "#START" and anything else is ignored

    if reason is None:
        raise TimeoutError("No #END received from the Arduino.")
    if cut_us is None:
        raise RuntimeError(f"Test ended ({reason}) before the drive was cut.")

    rows = []
    prev_count = None
    for t_us, phase, count, edge_us, rpm in samples:
        new_edges = 0 if prev_count is None else count - prev_count
        prev_count = count
        rows.append({
            "phase": phase,
            "t_s": (t_us - cut_us) / 1e6,
            "edge_t_s": (edge_us - cut_us) / 1e6,
            "count": count,
            "new_edges": new_edges,
            "rpm": rpm,
            "omega_rad_s": rpm * 2.0 * math.pi / 60.0,
        })
    return rows, reason


def summarize(rows):
    """Speed at cut, steadiness before the cut, and coast duration."""
    pre = [r["rpm"] for r in rows if r["phase"] == 0 and r["t_s"] > -1.0]
    rpm_cut = sum(pre) / len(pre) if pre else float("nan")
    spread = (max(pre) - min(pre)) / rpm_cut * 100 if pre and rpm_cut > 0 else float("nan")
    moving = [r["t_s"] for r in rows if r["phase"] == 1 and r["new_edges"] > 0]
    coast_s = max(moving) if moving else 0.0
    return rpm_cut, spread, coast_s


def plot(all_rows):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plot.")
        return
    runs = sorted({(r["run"], r["pwm"]) for r in all_rows})
    for run, pwm in runs:
        pts = [r for r in all_rows
               if r["run"] == run and r["phase"] == 1 and r["new_edges"] > 0]
        plt.plot([r["t_s"] for r in pts], [r["rpm"] for r in pts],
                 ".-", ms=3, label=f"run {run}, pwm {pwm}")
    plt.xlabel("time since drive cut [s]")
    plt.ylabel("speed [RPM]")
    plt.title("Coast-down")
    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Run coast-down tests and log to CSV.")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pwm", type=int, nargs="+", default=[200],
                    help="One or more PWM levels (1-255) to spin up to")
    ap.add_argument("--runs", type=int, default=3, help="Repeats per PWM level")
    ap.add_argument("--spinup", type=int, default=5000,
                    help="Spin-up duration in ms (long enough to reach steady speed)")
    ap.add_argument("--rest", type=float, default=2.0,
                    help="Pause between runs, seconds")
    ap.add_argument("--out", default=None, help="Output CSV path")
    ap.add_argument("--plot", action="store_true", help="Plot the coast-down curves at the end")
    args = ap.parse_args()

    for p in args.pwm:
        if not 1 <= p <= 255:
            sys.exit(f"PWM {p} is out of range (1-255).")

    out_path = args.out or f"coastdown_{datetime.now():%Y%m%d_%H%M%S}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    all_rows = []
    try:
        if not wait_for_ready(ser):
            print("Warning: no #READY from the Arduino (board may not reset on connect). "
                  "Continuing anyway.")

        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
            writer.writeheader()

            run_idx = 0
            for pwm in args.pwm:
                for rep in range(args.runs):
                    run_idx += 1
                    print(f"Run {run_idx}: PWM {pwm}, spin-up {args.spinup} ms ...", flush=True)
                    rows, reason = run_one(ser, pwm, args.spinup)

                    for r in rows:
                        r["run"] = run_idx
                        r["pwm"] = pwm
                        writer.writerow(r)
                    f.flush()  # keep data even if a later run fails
                    all_rows.extend(rows)

                    rpm_cut, spread, coast_s = summarize(rows)
                    print(f"  end: {reason}, {len(rows)} samples, "
                          f"speed at cut {rpm_cut:.0f} RPM, coast time {coast_s:.2f} s")
                    if spread > 2.0:
                        print(f"  Warning: speed varied {spread:.1f}% in the last second of "
                              f"spin-up; consider a longer --spinup.")
                    time.sleep(args.rest)

        print(f"Saved {len(all_rows)} rows to {out_path}")

    except KeyboardInterrupt:
        print("\nInterrupted, turning motor off.")
        ser.write(b"X\n")
    finally:
        ser.close()

    if args.plot and all_rows:
        plot(all_rows)


if __name__ == "__main__":
    main()
