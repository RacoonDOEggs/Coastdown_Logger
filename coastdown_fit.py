#!/usr/bin/env python3
"""
Fit coast-down curves recorded by coastdown_logger.py.

With the motor open circuit (no current), the rotor obeys

    J dω/dt = -b ω - τc            (ω > 0)

so the speed decays as

    ω(t) = (ω0 + a/β) e^(-β t) - a/β,    β = b/J,  a = τc/J

The coast-down alone gives β = b/J [1/s] and a = τc/J [rad/s²]. To get J, supply
b or τc from a separate test (for example the steady-state sweep):

    J = b / β     or     J = τc / a

Two fits are done:
  * one per run (ω0, β, a free), to check repeatability;
  * one joint fit over all runs (β and a shared, one ω0 per run), which gives
    the final estimate.

Usage:
    pip install numpy scipy matplotlib
    python coastdown_fit.py coastdown_20261117_220222.csv --plot
    python coastdown_fit.py data.csv --b 1.2e-4 --tau-c 3.5e-3 --save-plot fit.png
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


# --------------------------------------------------------------------------- #
# Data loading
# --------------------------------------------------------------------------- #
def load_runs(path, min_rpm):
    """Return {run: dict(pwm, t, w)} with the coast-down measurements of each run.

    Only rows that are real measurements are kept: phase 1 (coasting) with at
    least one new encoder edge. Each speed is an average over the interval
    between the previous and the current encoder edge, so it is timestamped at
    the middle of that interval. Intervals that start before the cut (t < 0)
    are dropped because they mix driven and coasting motion.
    """
    by_run = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            by_run.setdefault(int(r["run"]), []).append(r)

    runs = {}
    for run, rows in sorted(by_run.items()):
        rows.sort(key=lambda r: float(r["t_s"]))
        t, w = [], []
        prev_edge = None
        for r in rows:
            edge = float(r["edge_t_s"])
            new_edges = int(r["new_edges"])
            if new_edges > 0:
                if (r["phase"] == "1" and prev_edge is not None and prev_edge >= 0.0
                        and float(r["rpm"]) >= min_rpm):
                    t.append(0.5 * (prev_edge + edge))
                    w.append(float(r["omega_rad_s"]))
                prev_edge = edge
        if len(t) >= 5:
            runs[run] = {"pwm": int(rows[0]["pwm"]),
                         "t": np.array(t), "w": np.array(w)}
        else:
            print(f"Run {run}: only {len(t)} usable coast-down points, skipped.")
    return runs


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #
def model(t, w0, beta, a):
    """ω(t) for the viscous + Coulomb coast-down, clipped at 0 once stopped."""
    if beta < 1e-9:  # pure Coulomb limit, avoids a/β blowing up
        w = w0 - a * t
    else:
        c = a / beta
        w = (w0 + c) * np.exp(-beta * t) - c
    return np.maximum(w, 0.0)


def initial_guess(t, w):
    """Linear regression of deceleration against speed: -dω/dt = β ω + a."""
    dwdt = np.gradient(w, t)
    A = np.column_stack([w, np.ones_like(w)])
    (beta, a), *_ = np.linalg.lstsq(A, -dwdt, rcond=None)
    beta = max(beta, 1e-3)
    a = max(a, 1e-3)
    return w[0] + a * t[0], beta, a  # back-extrapolate ω to t = 0


def param_errors(res):
    """1-sigma parameter uncertainties from the Jacobian of a least_squares fit."""
    J = res.jac
    dof = max(len(res.fun) - len(res.x), 1)
    s2 = 2.0 * res.cost / dof
    try:
        cov = np.linalg.inv(J.T @ J) * s2
        return np.sqrt(np.diag(cov)), math.sqrt(s2)
    except np.linalg.LinAlgError:
        return np.full(len(res.x), np.nan), math.sqrt(s2)


def fit_single(t, w):
    p0 = initial_guess(t, w)
    res = least_squares(lambda p: model(t, *p) - w, p0,
                        bounds=([0.0, 0.0, 0.0], [np.inf, np.inf, np.inf]),
                        x_scale="jac")
    err, rmse = param_errors(res)
    return res.x, err, rmse


def fit_joint(runs):
    keys = list(runs)
    guesses = [initial_guess(runs[k]["t"], runs[k]["w"]) for k in keys]
    beta0 = np.median([g[1] for g in guesses])
    a0 = np.median([g[2] for g in guesses])
    p0 = np.array([beta0, a0] + [g[0] for g in guesses])

    def residuals(p):
        beta, a = p[0], p[1]
        return np.concatenate([model(runs[k]["t"], p[2 + i], beta, a) - runs[k]["w"]
                               for i, k in enumerate(keys)])

    res = least_squares(residuals, p0, bounds=(0.0, np.inf), x_scale="jac")
    err, rmse = param_errors(res)
    w0s = {k: res.x[2 + i] for i, k in enumerate(keys)}
    return res.x[0], res.x[1], err[0], err[1], w0s, rmse


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
def plot(runs, single, joint, save_path=None, show=True):
    import matplotlib.pyplot as plt

    beta, a, _, _, w0s, _ = joint
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, (k, d) in enumerate(runs.items()):
        c = colors[i % len(colors)]
        tt = np.linspace(0, d["t"].max(), 400)
        ax1.plot(d["t"], d["w"] * RPM_PER_RAD_S, ".", ms=2, color=c, alpha=0.5)
        ax1.plot(tt, model(tt, w0s[k], beta, a) * RPM_PER_RAD_S, "-", lw=1,
                 color=c, label=f"run {k} (PWM {d['pwm']})")
        resid = (d["w"] - model(d["t"], w0s[k], beta, a)) * RPM_PER_RAD_S
        ax2.plot(d["t"], resid, ".", ms=2, color=c, alpha=0.6)

    ax1.set_ylabel("speed [RPM]")
    ax1.set_title(f"Coast-down, joint fit: b/J = {beta:.4g} 1/s, "
                  f"τc/J = {a:.4g} rad/s²  (dots: data, lines: fit)")
    ax1.grid(True)
    ax1.legend(fontsize=8, ncol=2)
    ax2.axhline(0, color="k", lw=0.8)
    ax2.set_xlabel("time since drive cut [s]")
    ax2.set_ylabel("residual [RPM]")
    ax2.grid(True)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Plot saved to {save_path}")
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Fit coast-down curves to "
                                 "ω(t) = (ω0 + a/β)e^(-βt) - a/β.")
    ap.add_argument("csv", help="CSV written by coastdown_logger.py")
    ap.add_argument("--runs", type=int, nargs="+",
                    help="Only fit these run numbers (default: all)")
    ap.add_argument("--min-rpm", type=float, default=0.0,
                    help="Ignore points below this speed (to exclude stiction effects "
                         "near standstill)")
    ap.add_argument("--b", type=float,
                    help="Viscous friction b [N·m·s/rad], if known, to compute J = b/β")
    ap.add_argument("--tau-c", type=float,
                    help="Coulomb friction τc [N·m], if known, to compute J = τc/a")
    ap.add_argument("--out", help="Results CSV (default: <input>_fit.csv)")
    ap.add_argument("--plot", action="store_true", help="Show the fit plot")
    ap.add_argument("--save-plot", help="Save the fit plot to this image file")
    args = ap.parse_args()

    runs = load_runs(args.csv, args.min_rpm)
    if args.runs:
        runs = {k: v for k, v in runs.items() if k in args.runs}
    if not runs:
        sys.exit("No usable coast-down data in the file.")

    # ---- Per-run fits ----
    print("\nPer-run fits (±1σ)")
    print(f"{'run':>4} {'pwm':>4} {'pts':>5} {'ω0 [RPM]':>10} "
          f"{'b/J [1/s]':>18} {'τc/J [rad/s²]':>20} {'RMSE [RPM]':>11}")
    single = {}
    for k, d in runs.items():
        p, e, rmse = fit_single(d["t"], d["w"])
        single[k] = (p, e, rmse)
        print(f"{k:>4} {d['pwm']:>4} {len(d['t']):>5} {p[0] * RPM_PER_RAD_S:>10.0f} "
              f"{p[1]:>9.4f} ± {e[1]:<6.4f} {p[2]:>10.3f} ± {e[2]:<7.3f} "
              f"{rmse * RPM_PER_RAD_S:>11.1f}")

    betas = np.array([single[k][0][1] for k in runs])
    accs = np.array([single[k][0][2] for k in runs])
    if len(runs) > 1:
        print(f"\nSpread between runs: b/J = {betas.mean():.4f} ± {betas.std(ddof=1):.4f} 1/s, "
              f"τc/J = {accs.mean():.3f} ± {accs.std(ddof=1):.3f} rad/s²")

    # ---- Joint fit ----
    joint = fit_joint(runs)
    beta, a, ebeta, ea, w0s, rmse = joint
    print("\nJoint fit over all runs (±1σ)")
    print(f"  b/J  = {beta:.5g} ± {ebeta:.2g} 1/s   "
          f"(mechanical coast time constant J/b = {1 / beta if beta > 0 else math.inf:.3g} s)")
    print(f"  τc/J = {a:.5g} ± {ea:.2g} rad/s²")
    print(f"  RMSE = {rmse * RPM_PER_RAD_S:.1f} RPM")
    w_eq = a / beta if beta > 0 else math.inf
    print(f"  Viscous and Coulomb friction are equal at {w_eq * RPM_PER_RAD_S:.0f} RPM "
          f"(viscous dominates above, Coulomb below).")

    J_from_b = J_from_tc = None
    if args.b is not None and beta > 0:
        J_from_b = args.b / beta
        print(f"\n  J = b / (b/J)     = {J_from_b:.4g} kg·m²   (from --b {args.b:g})")
    if args.tau_c is not None and a > 0:
        J_from_tc = args.tau_c / a
        print(f"  J = τc / (τc/J)   = {J_from_tc:.4g} kg·m²   (from --tau-c {args.tau_c:g})")
    if J_from_b and J_from_tc:
        diff = abs(J_from_b - J_from_tc) / ((J_from_b + J_from_tc) / 2) * 100
        print(f"  The two estimates of J differ by {diff:.1f} %.")
    if args.b is None and args.tau_c is None:
        print("\n  Pass --b and/or --tau-c (from your steady-state test) to compute J.")

    # ---- Save results ----
    out = args.out or os.path.splitext(args.csv)[0] + "_fit.csv"
    with open(out, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["fit", "run", "pwm", "points", "omega0_rad_s", "b_over_J",
                     "b_over_J_err", "tauc_over_J", "tauc_over_J_err", "rmse_rad_s",
                     "J_from_b", "J_from_tauc"])
        for k, d in runs.items():
            p, e, r = single[k]
            wr.writerow(["run", k, d["pwm"], len(d["t"]), p[0], p[1], e[1], p[2], e[2],
                         r, "", ""])
        wr.writerow(["joint", "all", "", sum(len(d["t"]) for d in runs.values()), "",
                     beta, ebeta, a, ea, rmse,
                     J_from_b if J_from_b else "", J_from_tc if J_from_tc else ""])
    print(f"\nResults saved to {out}")

    if args.plot or args.save_plot:
        plot(runs, single, joint, save_path=args.save_plot, show=args.plot)


if __name__ == "__main__":
    main()
