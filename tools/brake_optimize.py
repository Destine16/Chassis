#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from brake_analyze import TrialMetrics, analyze_one_source, load_csv


TRIAL_COLUMNS = {
    "source",
    "trial_index",
    "k_brake",
    "direction",
    "valid",
    "stop_time_s",
    "stop_distance_m",
    "max_reverse_speed_mps",
    "max_current_est",
    "oscillation_count",
    "peak_speed_mps",
    "score",
}


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compute a continuous K_brake recommendation from active-brake capture data. "
            "Input can be raw brake_capture.py CSV files or brake_analyze.py trials.csv files."
        )
    )
    parser.add_argument("csv", nargs="+", type=Path, help="Raw capture CSV or trials.csv files")
    parser.add_argument("-o", "--output-dir", type=Path, help="Output directory")
    parser.add_argument("--method", choices=("auto", "quadratic", "pchip", "measured"), default="auto")
    parser.add_argument("--cost-mode", choices=("score", "peak-normalized", "direction-normalized"),
                        default="peak-normalized")
    parser.add_argument("--k-min", type=float, help="Lower bound for continuous optimization")
    parser.add_argument("--k-max", type=float, help="Upper bound for continuous optimization")
    parser.add_argument("--bootstrap", type=int, default=500, help="Bootstrap resamples for confidence interval")
    parser.add_argument("--seed", type=int, default=7, help="Random seed for bootstrap")
    parser.add_argument("--grid-count", type=int, default=2001, help="Dense grid count for curve search")
    parser.add_argument("--min-fit-r2", type=float, default=0.20, help="Minimum quadratic R^2 used by auto mode")

    parser.add_argument("--min-start-speed", type=float, default=0.25, help="Minimum planar speed to detect a trial")
    parser.add_argument("--stop-speed", type=float, default=0.05, help="Speed regarded as stopped")
    parser.add_argument("--stop-hold-s", type=float, default=0.25, help="Stopped speed must hold this long")
    parser.add_argument("--min-moving-s", type=float, default=0.35, help="Minimum moving segment duration")
    parser.add_argument("--max-stop-time-s", type=float, default=3.0, help="Reject if braking takes longer than this")
    parser.add_argument("--max-bad-flag-frac", type=float, default=0.05, help="Reject if BAD frame ratio exceeds this")
    parser.add_argument("--max-yaw-equiv-ratio", type=float, default=0.65, help="Reject if rotation dominates translation")
    parser.add_argument("--current-limit", type=float, default=4000.0, help="Firmware brake current clamp")
    return parser


def parse_bool_series(series: pd.Series) -> pd.Series:
    if series.dtype == bool:
        return series
    return series.astype(str).str.lower().isin(("1", "true", "yes", "y"))


def load_trials_path(path: Path, args: argparse.Namespace) -> pd.DataFrame:
    head = pd.read_csv(path, nrows=5)
    if TRIAL_COLUMNS.issubset(set(head.columns)):
        df = pd.read_csv(path)
        df["source"] = df["source"].astype(str)
        df["valid"] = parse_bool_series(df["valid"])
        return df

    raw_df = load_csv(path)
    trials = analyze_one_source(raw_df, args)
    return pd.DataFrame([asdict(trial) for trial in trials])


def load_trials(paths: list[Path], args: argparse.Namespace) -> pd.DataFrame:
    frames = [load_trials_path(path, args) for path in paths]
    if not frames:
        return pd.DataFrame()

    trials = pd.concat(frames, ignore_index=True)
    numeric_cols = [
        "k_brake",
        "stop_time_s",
        "stop_distance_m",
        "max_reverse_speed_mps",
        "max_current_est",
        "oscillation_count",
        "peak_speed_mps",
        "score",
    ]
    for col in numeric_cols:
        if col in trials.columns:
            trials[col] = pd.to_numeric(trials[col], errors="coerce")
    trials = trials.dropna(subset=["k_brake", "score", "peak_speed_mps"]).reset_index(drop=True)
    return trials


def add_cost_column(trials: pd.DataFrame, cost_mode: str) -> pd.DataFrame:
    trials = trials.copy()
    if cost_mode == "score":
        trials["opt_cost"] = trials["score"]
    elif cost_mode == "peak-normalized":
        trials["opt_cost"] = trials["score"] / trials["peak_speed_mps"].clip(lower=0.10)
    elif cost_mode == "direction-normalized":
        valid = trials[trials["valid"]].copy()
        med = valid.groupby("direction")["score"].median()
        fallback = float(valid["score"].median()) if not valid.empty else 1.0
        trials["opt_cost"] = [
            score / max(float(med.get(direction, fallback)), 1e-6)
            for score, direction in zip(trials["score"], trials["direction"])
        ]
    else:
        raise ValueError(f"unknown cost mode: {cost_mode}")
    return trials


def summarize_cost(trials: pd.DataFrame) -> pd.DataFrame:
    valid = trials[trials["valid"]].copy()
    if valid.empty:
        return pd.DataFrame()

    summary = (
        valid.groupby("k_brake")
        .agg(
            valid_trials=("opt_cost", "size"),
            mean_cost=("opt_cost", "mean"),
            std_cost=("opt_cost", "std"),
            median_cost=("opt_cost", "median"),
            mean_score=("score", "mean"),
            mean_stop_time_s=("stop_time_s", "mean"),
            mean_stop_distance_m=("stop_distance_m", "mean"),
            mean_reverse_mps=("max_reverse_speed_mps", "mean"),
            mean_current_est=("max_current_est", "mean"),
            mean_oscillation=("oscillation_count", "mean"),
            mean_peak_speed_mps=("peak_speed_mps", "mean"),
        )
        .reset_index()
        .sort_values("k_brake")
    )
    summary["std_cost"] = summary["std_cost"].fillna(0.0)
    summary["sem_cost"] = summary["std_cost"] / np.sqrt(summary["valid_trials"].clip(lower=1))
    return summary


def r2_score(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y_true - y_pred) ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if ss_tot <= 1e-12:
        return 1.0 if ss_res <= 1e-12 else 0.0
    return 1.0 - (ss_res / ss_tot)


def fit_quadratic(summary: pd.DataFrame, k_min: float, k_max: float) -> dict:
    if len(summary) < 3:
        return {"valid": False, "reason": "need at least 3 distinct K values"}

    x = summary["k_brake"].to_numpy(dtype=float)
    y = summary["mean_cost"].to_numpy(dtype=float)
    weights = np.sqrt(summary["valid_trials"].to_numpy(dtype=float).clip(min=1.0))
    coeff = np.polyfit(x, y, deg=2, w=weights)
    y_hat = np.polyval(coeff, x)
    a, b, c = (float(coeff[0]), float(coeff[1]), float(coeff[2]))
    if abs(a) <= 1e-12:
        return {"valid": False, "reason": "quadratic curvature too small", "coeff": [a, b, c]}

    if a <= 0.0:
        return {
            "valid": False,
            "reason": "quadratic fit is concave, no stable interior minimum",
            "coeff": [a, b, c],
            "r2": r2_score(y, y_hat),
        }

    optimum = float(np.clip(-b / (2.0 * a), k_min, k_max))
    return {
        "valid": True,
        "method": "quadratic",
        "k_opt": optimum,
        "cost_opt": float(np.polyval(coeff, optimum)),
        "coeff": [a, b, c],
        "r2": r2_score(y, y_hat),
    }


def pchip_values(x: np.ndarray, y: np.ndarray, grid: np.ndarray) -> np.ndarray:
    try:
        from scipy.interpolate import PchipInterpolator
    except ImportError:
        return np.interp(grid, x, y)

    if len(x) < 3:
        return np.interp(grid, x, y)
    return PchipInterpolator(x, y)(grid)


def fit_pchip(summary: pd.DataFrame, k_min: float, k_max: float, grid_count: int) -> dict:
    if len(summary) < 2:
        return {"valid": False, "reason": "need at least 2 distinct K values"}

    x = summary["k_brake"].to_numpy(dtype=float)
    y = summary["mean_cost"].to_numpy(dtype=float)
    grid = np.linspace(k_min, k_max, max(grid_count, 101))
    curve = pchip_values(x, y, grid)
    idx = int(np.argmin(curve))
    return {
        "valid": True,
        "method": "pchip",
        "k_opt": float(grid[idx]),
        "cost_opt": float(curve[idx]),
    }


def best_measured(summary: pd.DataFrame) -> dict:
    if summary.empty:
        return {"valid": False, "reason": "no valid trials"}
    row = summary.loc[summary["mean_cost"].idxmin()]
    return {
        "valid": True,
        "method": "measured",
        "k_opt": float(row["k_brake"]),
        "cost_opt": float(row["mean_cost"]),
    }


def choose_fit(summary: pd.DataFrame, args: argparse.Namespace, k_min: float, k_max: float) -> dict:
    measured = best_measured(summary)
    quadratic = fit_quadratic(summary, k_min, k_max)
    pchip = fit_pchip(summary, k_min, k_max, args.grid_count)

    if args.method == "measured":
        chosen = measured
    elif args.method == "quadratic":
        chosen = quadratic if quadratic.get("valid") else measured
    elif args.method == "pchip":
        chosen = pchip if pchip.get("valid") else measured
    else:
        if quadratic.get("valid") and (quadratic.get("r2", 0.0) >= args.min_fit_r2):
            chosen = quadratic
        elif pchip.get("valid"):
            chosen = pchip
        else:
            chosen = measured

    chosen = dict(chosen)
    chosen["best_measured_k_brake"] = measured.get("k_opt")
    chosen["best_measured_cost"] = measured.get("cost_opt")
    chosen["quadratic"] = quadratic
    chosen["pchip"] = pchip
    return chosen


def bootstrap_optimum(trials: pd.DataFrame, args: argparse.Namespace, k_min: float, k_max: float) -> dict:
    valid = trials[trials["valid"]].copy()
    if len(valid) < 4 or args.bootstrap <= 0:
        return {"count": 0, "k_median": None, "k_ci95": None}

    rng = np.random.default_rng(args.seed)
    values: list[float] = []
    for _ in range(args.bootstrap):
        sample = valid.iloc[rng.integers(0, len(valid), size=len(valid))].copy()
        summary = summarize_cost(sample)
        if summary["k_brake"].nunique() < 2:
            continue
        fit = choose_fit(summary, args, k_min, k_max)
        if fit.get("valid"):
            values.append(float(fit["k_opt"]))

    if not values:
        return {"count": 0, "k_median": None, "k_ci95": None}

    arr = np.asarray(values, dtype=float)
    return {
        "count": int(len(arr)),
        "k_median": float(np.median(arr)),
        "k_ci95": [float(np.percentile(arr, 2.5)), float(np.percentile(arr, 97.5))],
        "k_std": float(np.std(arr)),
    }


def make_curve(summary: pd.DataFrame, fit: dict, k_min: float, k_max: float, grid_count: int) -> tuple[np.ndarray, np.ndarray]:
    grid = np.linspace(k_min, k_max, max(grid_count, 101))
    method = fit.get("method")
    if method == "quadratic" and fit.get("valid"):
        curve = np.polyval(np.asarray(fit["coeff"], dtype=float), grid)
        return grid, curve
    if method == "pchip" and fit.get("valid"):
        x = summary["k_brake"].to_numpy(dtype=float)
        y = summary["mean_cost"].to_numpy(dtype=float)
        return grid, pchip_values(x, y, grid)
    x = summary["k_brake"].to_numpy(dtype=float)
    y = summary["mean_cost"].to_numpy(dtype=float)
    return grid, np.interp(grid, x, y)


def plot_optimization(trials: pd.DataFrame, summary: pd.DataFrame, fit: dict, bootstrap: dict,
                      args: argparse.Namespace, output: Path, k_min: float, k_max: float) -> None:
    valid = trials[trials["valid"]].copy()
    grid, curve = make_curve(summary, fit, k_min, k_max, args.grid_count)

    fig, ax = plt.subplots(figsize=(11, 6), constrained_layout=True)
    ax.scatter(valid["k_brake"], valid["opt_cost"], alpha=0.35, label="valid trials", color="tab:blue")
    ax.errorbar(
        summary["k_brake"],
        summary["mean_cost"],
        yerr=summary["sem_cost"],
        fmt="o",
        color="tab:red",
        capsize=4,
        label="mean +/- SEM",
    )
    ax.plot(grid, curve, color="tab:green", linewidth=2.0, label=f"{fit.get('method')} fit")

    k_opt = fit.get("k_opt")
    if k_opt is not None:
        ax.axvline(k_opt, color="tab:green", linestyle="--", label=f"recommended K={k_opt:.1f}")

    ci = bootstrap.get("k_ci95")
    if ci is not None:
        ax.axvspan(ci[0], ci[1], color="tab:green", alpha=0.12, label="bootstrap 95% CI")

    measured = fit.get("best_measured_k_brake")
    if measured is not None:
        ax.axvline(measured, color="tab:red", linestyle=":", label=f"best measured K={measured:.1f}")

    ax.set_title("Continuous K_brake optimization")
    ax.set_xlabel("K_brake")
    ax.set_ylabel(f"cost ({args.cost_mode}, lower is better)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main() -> int:
    args = build_arg_parser().parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path("data/brake") / f"optimize_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    trials = load_trials(args.csv, args)
    if trials.empty:
        print("[error] no trials detected")
        return 1

    trials = add_cost_column(trials, args.cost_mode)
    summary = summarize_cost(trials)
    if summary.empty:
        print("[error] no valid trials for optimization")
        return 1

    measured_min = float(summary["k_brake"].min())
    measured_max = float(summary["k_brake"].max())
    k_min = measured_min if args.k_min is None else float(args.k_min)
    k_max = measured_max if args.k_max is None else float(args.k_max)
    if k_min >= k_max:
        raise ValueError("k-min must be smaller than k-max")

    fit = choose_fit(summary, args, k_min, k_max)
    boot = bootstrap_optimum(trials, args, k_min, k_max)

    trials_csv = output_dir / "optimization_trials.csv"
    summary_csv = output_dir / "optimization_group_summary.csv"
    summary_json = output_dir / "optimization_summary.json"
    plot_png = output_dir / "k_brake_optimization.png"

    trials.to_csv(trials_csv, index=False)
    summary.to_csv(summary_csv, index=False)
    plot_optimization(trials, summary, fit, boot, args, plot_png, k_min, k_max)

    recommendation = {
        "recommended_k_brake": fit.get("k_opt"),
        "recommended_method": fit.get("method"),
        "recommended_cost": fit.get("cost_opt"),
        "best_measured_k_brake": fit.get("best_measured_k_brake"),
        "best_measured_cost": fit.get("best_measured_cost"),
        "bootstrap": boot,
        "cost_mode": args.cost_mode,
        "k_bounds": [k_min, k_max],
        "measured_k_bounds": [measured_min, measured_max],
        "valid_trials": int(trials["valid"].sum()),
        "total_trials": int(len(trials)),
    }

    with summary_json.open("w") as fp:
        json.dump(
            {
                "recommendation": recommendation,
                "fits": {
                    "quadratic": fit.get("quadratic"),
                    "pchip": fit.get("pchip"),
                },
                "args": vars(args) | {"csv": [str(path) for path in args.csv], "output_dir": str(output_dir)},
            },
            fp,
            indent=2,
        )

    print(f"[done] total_trials={len(trials)} valid={int(trials['valid'].sum())}")
    print(f"[done] trials_csv={trials_csv}")
    print(f"[done] group_summary_csv={summary_csv}")
    print(f"[done] summary_json={summary_json}")
    print(f"[done] plot={plot_png}")
    print(f"[recommendation] {recommendation}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
