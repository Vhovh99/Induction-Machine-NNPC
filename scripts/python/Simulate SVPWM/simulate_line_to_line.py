#!/usr/bin/env python3
"""Simulate line-to-line voltages from gate signals CSV."""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def phase_voltage_from_gates(high, low, vdc):
    """Map high/low gate signals to half-bridge phase voltage."""
    # High=1, Low=0 -> +Vdc/2; High=0, Low=1 -> -Vdc/2
    # Both 0 (tri-state) or both 1 (shoot-through) -> 0
    v = np.zeros_like(high, dtype=float)
    v[(high == 1) & (low == 0)] = 0.5 * vdc
    v[(high == 0) & (low == 1)] = -0.5 * vdc
    return v


def low_pass_1st_order(x, dt, cutoff_hz):
    """Simple 1st-order low-pass filter."""
    if cutoff_hz <= 0:
        return x
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt / (rc + dt)
    y = np.empty_like(x, dtype=float)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
    return y


def pick_csv_file(script_dir, provided_path):
    if provided_path is not None:
        return provided_path

    csv_files = sorted(script_dir.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {script_dir}")

    if len(csv_files) == 1 or not sys.stdin.isatty():
        return csv_files[0]

    print("Select input CSV file:")
    for idx, path in enumerate(csv_files, start=1):
        print(f"  {idx}. {path.name}")

    while True:
        choice = input("Enter number: ").strip()
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(csv_files):
                return csv_files[index - 1]
        print("Invalid selection. Try again.")


def gate_stats(high, low):
    tri_state = int(np.sum((high == 0) & (low == 0)))
    shoot_through = int(np.sum((high == 1) & (low == 1)))
    high_only = int(np.sum((high == 1) & (low == 0)))
    low_only = int(np.sum((high == 0) & (low == 1)))
    invalid = int(np.sum(~np.isin(high, [0, 1])) + np.sum(~np.isin(low, [0, 1])))
    return {
        "tri_state": tri_state,
        "shoot_through": shoot_through,
        "high_only": high_only,
        "low_only": low_only,
        "invalid_bits": invalid,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Simulate line-to-line voltages from gate signals CSV."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Input CSV file with gate signals (leave empty to choose).",
    )
    parser.add_argument(
        "--vdc",
        type=float,
        default=1.0,
        help="DC bus voltage used for scaling.",
    )
    parser.add_argument(
        "--lpf-hz",
        type=float,
        default=0.0,
        help="Optional low-pass cutoff in Hz (0 to disable).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV path (default: <input>_line_to_line.csv).",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable plotting of output waveforms.",
    )
    parser.add_argument(
        "--plot-skip",
        type=int,
        default=1,
        help="Plot every Nth sample to reduce rendering load.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print debug stats about sampling and gate states.",
    )

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    csv_path = pick_csv_file(script_dir, args.csv)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {args.csv}")

    df = pd.read_csv(csv_path)
    df.columns = [c.strip() for c in df.columns]

    time_s = df["Time[s]"]
    dt = float(np.median(np.diff(time_s.values))) if len(time_s) > 1 else 0.0
    fs = 1.0 / dt if dt > 0.0 else 0.0

    ha = df["Phase A/H"].astype(int).values
    la = df["Phase A_L"].astype(int).values
    hb = df["Phase B/H"].astype(int).values
    lb = df["Phase B_L"].astype(int).values
    hc = df["Phase C/H"].astype(int).values
    lc = df["Phase C_L"].astype(int).values

    va = phase_voltage_from_gates(ha, la, args.vdc)
    vb = phase_voltage_from_gates(hb, lb, args.vdc)
    vc = phase_voltage_from_gates(hc, lc, args.vdc)

    vab = va - vb
    vbc = vb - vc
    vca = vc - va

    if args.lpf_hz > 0.0 and dt > 0.0:
        va = low_pass_1st_order(va, dt, args.lpf_hz)
        vb = low_pass_1st_order(vb, dt, args.lpf_hz)
        vc = low_pass_1st_order(vc, dt, args.lpf_hz)
        vab = low_pass_1st_order(vab, dt, args.lpf_hz)
        vbc = low_pass_1st_order(vbc, dt, args.lpf_hz)
        vca = low_pass_1st_order(vca, dt, args.lpf_hz)

    out_df = pd.DataFrame(
        {
            "Time[s]": time_s,
            "Va": va,
            "Vb": vb,
            "Vc": vc,
            "Vab": vab,
            "Vbc": vbc,
            "Vca": vca,
        }
    )

    output = args.output
    if output is None:
        output = csv_path.with_name(csv_path.stem + "_line_to_line.csv")
    out_df.to_csv(output, index=False)

    if args.debug:
        stats_a = gate_stats(ha, la)
        stats_b = gate_stats(hb, lb)
        stats_c = gate_stats(hc, lc)
        print(f"Input file: {csv_path}")
        print(f"Samples: {len(time_s)}")
        print(f"dt: {dt:.9f} s | fs: {fs:.2f} Hz")
        print("Phase A stats:", stats_a)
        print("Phase B stats:", stats_b)
        print("Phase C stats:", stats_c)

    if not args.no_plot:
        import matplotlib.pyplot as plt

        skip = max(1, int(args.plot_skip))
        t_plot = time_s.values[::skip]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        ax1.plot(t_plot, va[::skip], label="Va")
        ax1.plot(t_plot, vb[::skip], label="Vb")
        ax1.plot(t_plot, vc[::skip], label="Vc")
        ax1.set_ylabel("Phase Voltage")
        ax1.set_title("Phase Voltages")
        ax1.legend()

        ax2.plot(t_plot, vab[::skip], label="Vab")
        ax2.plot(t_plot, vbc[::skip], label="Vbc")
        ax2.plot(t_plot, vca[::skip], label="Vca")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Line-to-Line Voltage")
        ax2.set_title("Line-to-Line Voltages")
        ax2.legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
