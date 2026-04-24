#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class TrialMetrics:
    source: str
    trial_index: int
    k_brake: float
    direction: str
    valid: bool
    reject_reason: str
    start_t_s: float
    peak_t_s: float
    stop_t_s: float
    peak_speed_mps: float
    stop_time_s: float
    stop_distance_m: float
    max_reverse_speed_mps: float
    max_current_est: float
    oscillation_count: int
    flag_bad_fraction: float
    yaw_equiv_ratio: float
    score: float


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze active-brake capture CSV files.")
    parser.add_argument("csv", nargs="+", type=Path, help="CSV files produced by brake_capture.py")
    parser.add_argument("-o", "--output-dir", type=Path, help="Output directory")
    parser.add_argument("--min-start-speed", type=float, default=0.25, help="Minimum planar speed to detect a trial")
    parser.add_argument("--stop-speed", type=float, default=0.05, help="Speed regarded as stopped")
    parser.add_argument("--stop-hold-s", type=float, default=0.25, help="Stopped speed must hold this long")
    parser.add_argument("--min-moving-s", type=float, default=0.35, help="Minimum moving segment duration")
    parser.add_argument("--max-stop-time-s", type=float, default=3.0, help="Reject if braking takes longer than this")
    parser.add_argument("--max-bad-flag-frac", type=float, default=0.05, help="Reject if BAD frame ratio exceeds this")
    parser.add_argument("--max-yaw-equiv-ratio", type=float, default=0.65, help="Reject if rotation dominates translation")
    parser.add_argument("--current-limit", type=float, default=4000.0, help="Firmware brake current clamp")
    parser.add_argument("--min-trials-per-k", type=int, default=2, help="Minimum valid trials for recommendation")
    return parser


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if "rel_t_s" not in df.columns:
        raise ValueError(f"{path}: missing rel_t_s column")

    numeric_cols = [
        "rel_t_s",
        "k_brake",
        "w_fl_radps",
        "w_fr_radps",
        "w_rl_radps",
        "w_rr_radps",
        "vx_mps",
        "vy_mps",
        "wz_radps",
        "speed_planar_mps",
        "yaw_equiv_mps",
        "max_abs_wheel_radps",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["rel_t_s", "speed_planar_mps", "vx_mps", "vy_mps", "k_brake"]).copy()
    df["source"] = str(path)
    if "judge" not in df.columns:
        df["judge"] = "UNKNOWN"
    if "direction_label" not in df.columns:
        df["direction_label"] = "unknown"
    return df.reset_index(drop=True)


def median_dt_s(t_s: np.ndarray) -> float:
    if len(t_s) < 2:
        return 0.005
    diff = np.diff(t_s)
    diff = diff[diff > 0.0]
    if len(diff) == 0:
        return 0.005
    return float(np.median(diff))


def find_stop_index(speed: np.ndarray, start_idx: int, hold_samples: int, stop_speed: float) -> int | None:
    below = speed <= stop_speed
    last = len(speed) - hold_samples
    idx = start_idx
    while idx <= last:
        if bool(np.all(below[idx:idx + hold_samples])):
            return idx
        idx += 1
    return None


def classify_direction(vx: float, vy: float) -> str:
    if abs(vx) >= abs(vy):
        return "forward" if vx >= 0.0 else "back"
    return "left" if vy >= 0.0 else "right"


def count_oscillations(projection: np.ndarray, threshold: float) -> int:
    filtered = projection[np.abs(projection) >= threshold]
    if len(filtered) < 2:
        return 0
    signs = np.sign(filtered)
    return int(np.sum(signs[1:] * signs[:-1] < 0.0))


def score_trial(stop_time_s: float, stop_distance_m: float, max_reverse_speed_mps: float,
                max_current_est: float, oscillation_count: int) -> float:
    return (
        stop_time_s
        + (2.0 * stop_distance_m)
        + (2.0 * max_reverse_speed_mps)
        + (0.00015 * max_current_est)
        + (0.10 * oscillation_count)
    )


def analyze_one_source(df: pd.DataFrame, args: argparse.Namespace) -> list[TrialMetrics]:
    t = df["rel_t_s"].to_numpy(dtype=float)
    speed = df["speed_planar_mps"].to_numpy(dtype=float)
    vx = df["vx_mps"].to_numpy(dtype=float)
    vy = df["vy_mps"].to_numpy(dtype=float)
    yaw_equiv = df.get("yaw_equiv_mps", pd.Series(np.zeros(len(df)))).to_numpy(dtype=float)
    max_wheel = df.get("max_abs_wheel_radps", pd.Series(np.zeros(len(df)))).to_numpy(dtype=float)
    k_values = df["k_brake"].to_numpy(dtype=float)
    dt = median_dt_s(t)
    hold_samples = max(1, int(round(args.stop_hold_s / dt)))
    min_moving_samples = max(1, int(round(args.min_moving_s / dt)))

    trials: list[TrialMetrics] = []
    idx = 0
    trial_index = 0

    while idx < len(df):
        while idx < len(df) and speed[idx] < args.min_start_speed:
            idx += 1
        if idx >= len(df):
            break

        move_start = idx
        while idx < len(df) and speed[idx] >= args.min_start_speed:
            idx += 1
        move_end = idx

        if (move_end - move_start) < min_moving_samples:
            continue

        peak_idx = move_start + int(np.argmax(speed[move_start:move_end]))
        stop_idx = find_stop_index(speed, peak_idx, hold_samples, args.stop_speed)
        if stop_idx is None:
            stop_idx = min(len(df) - 1, peak_idx + int(round(args.max_stop_time_s / dt)))
            stop_found = False
        else:
            stop_found = True

        eval_slice = slice(peak_idx, stop_idx + 1)
        peak_window_start = max(move_start, peak_idx - int(round(0.20 / dt)))
        mean_vx = float(np.mean(vx[peak_window_start:peak_idx + 1]))
        mean_vy = float(np.mean(vy[peak_window_start:peak_idx + 1]))
        norm = math.hypot(mean_vx, mean_vy)
        if norm <= 1e-6:
            mean_vx = float(vx[peak_idx])
            mean_vy = float(vy[peak_idx])
            norm = max(math.hypot(mean_vx, mean_vy), 1e-6)

        unit_x = mean_vx / norm
        unit_y = mean_vy / norm
        projection = vx[eval_slice] * unit_x + vy[eval_slice] * unit_y
        reverse_speed = float(max(0.0, -np.min(projection)))
        stop_time = float(t[stop_idx] - t[peak_idx])
        stop_distance = float(np.trapezoid(speed[eval_slice], t[eval_slice])) if len(speed[eval_slice]) > 1 else 0.0
        max_current_est = float(min(args.current_limit, np.max(k_values[eval_slice] * max_wheel[eval_slice])))
        oscillations = count_oscillations(projection, args.stop_speed)
        yaw_ratio = float(np.max(yaw_equiv[eval_slice]) / max(float(speed[peak_idx]), 1e-6))

        judge = df["judge"].iloc[eval_slice].astype(str)
        bad_fraction = float(np.mean(judge.str.startswith("BAD").to_numpy(dtype=bool)))
        direction = classify_direction(mean_vx, mean_vy)

        reject: list[str] = []
        if not stop_found:
            reject.append("no_stop")
        if speed[peak_idx] < args.min_start_speed:
            reject.append("peak_too_low")
        if stop_time > args.max_stop_time_s:
            reject.append("stop_too_slow")
        if bad_fraction > args.max_bad_flag_frac:
            reject.append("bad_flags")
        if yaw_ratio > args.max_yaw_equiv_ratio:
            reject.append("too_much_rotation")

        score = score_trial(stop_time, stop_distance, reverse_speed, max_current_est, oscillations)
        trials.append(
            TrialMetrics(
                source=str(df["source"].iloc[0]),
                trial_index=trial_index,
                k_brake=float(np.median(k_values[eval_slice])),
                direction=direction,
                valid=len(reject) == 0,
                reject_reason=",".join(reject),
                start_t_s=float(t[move_start]),
                peak_t_s=float(t[peak_idx]),
                stop_t_s=float(t[stop_idx]),
                peak_speed_mps=float(speed[peak_idx]),
                stop_time_s=stop_time,
                stop_distance_m=stop_distance,
                max_reverse_speed_mps=reverse_speed,
                max_current_est=max_current_est,
                oscillation_count=oscillations,
                flag_bad_fraction=bad_fraction,
                yaw_equiv_ratio=yaw_ratio,
                score=score,
            )
        )
        trial_index += 1
        idx = max(stop_idx + 1, move_end + 1)

    return trials


def summarize_trials(trials_df: pd.DataFrame, min_trials_per_k: int) -> tuple[pd.DataFrame, dict]:
    valid = trials_df[trials_df["valid"]].copy()
    if valid.empty:
        return pd.DataFrame(), {"recommended_k_brake": None, "reason": "no valid trials"}

    summary = (
        valid.groupby("k_brake")
        .agg(
            valid_trials=("score", "size"),
            mean_score=("score", "mean"),
            mean_stop_time_s=("stop_time_s", "mean"),
            mean_stop_distance_m=("stop_distance_m", "mean"),
            mean_reverse_mps=("max_reverse_speed_mps", "mean"),
            mean_current_est=("max_current_est", "mean"),
            mean_oscillation=("oscillation_count", "mean"),
            mean_peak_speed_mps=("peak_speed_mps", "mean"),
        )
        .reset_index()
        .sort_values(["mean_score", "k_brake"])
    )

    candidates = summary[summary["valid_trials"] >= min_trials_per_k]
    if candidates.empty:
        candidates = summary

    best = candidates.iloc[0].to_dict()
    recommendation = {
        "recommended_k_brake": float(best["k_brake"]),
        "mean_score": float(best["mean_score"]),
        "valid_trials": int(best["valid_trials"]),
        "reason": "lowest mean score among valid K_brake groups",
    }
    return summary, recommendation


def plot_summary(trials_df: pd.DataFrame, summary: pd.DataFrame, recommendation: dict, output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    fig.suptitle(f"Active brake tuning, recommended K={recommendation.get('recommended_k_brake')}")

    metrics = [
        ("stop_time_s", "Stop time (s)"),
        ("stop_distance_m", "Stop distance (m)"),
        ("max_reverse_speed_mps", "Max reverse speed (m/s)"),
        ("score", "Score (lower is better)"),
    ]

    valid = trials_df[trials_df["valid"]]
    for ax, (col, ylabel) in zip(axes.ravel(), metrics):
        if not valid.empty:
            ax.scatter(valid["k_brake"], valid[col], alpha=0.65, label="trial")
        if not summary.empty:
            mean_col = {
                "stop_time_s": "mean_stop_time_s",
                "stop_distance_m": "mean_stop_distance_m",
                "max_reverse_speed_mps": "mean_reverse_mps",
                "score": "mean_score",
            }[col]
            ax.plot(summary["k_brake"], summary[mean_col], marker="o", color="tab:red", label="mean")
            for _, row in summary.iterrows():
                ax.annotate(f"n={int(row['valid_trials'])}", (row["k_brake"], row[mean_col]),
                            textcoords="offset points", xytext=(4, 4), fontsize=8)
        ax.set_xlabel("K_brake")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    args = build_arg_parser().parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path("data/brake") / f"analysis_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    all_trials: list[TrialMetrics] = []
    for path in args.csv:
        df = load_csv(path)
        all_trials.extend(analyze_one_source(df, args))

    trials_df = pd.DataFrame([asdict(trial) for trial in all_trials])
    if trials_df.empty:
        print("[error] no trials detected")
        return 1

    summary, recommendation = summarize_trials(trials_df, args.min_trials_per_k)

    trials_csv = output_dir / "trials.csv"
    summary_csv = output_dir / "summary.csv"
    summary_json = output_dir / "summary.json"
    plot_png = output_dir / "brake_tuning_summary.png"

    trials_df.to_csv(trials_csv, index=False)
    summary.to_csv(summary_csv, index=False)
    with summary_json.open("w") as fp:
        json.dump(
            {
                "recommendation": recommendation,
                "args": vars(args) | {"csv": [str(path) for path in args.csv], "output_dir": str(output_dir)},
            },
            fp,
            indent=2,
        )
    plot_summary(trials_df, summary, recommendation, plot_png)

    print(f"[done] trials={len(trials_df)} valid={int(trials_df['valid'].sum())}")
    print(f"[done] trials_csv={trials_csv}")
    print(f"[done] summary_csv={summary_csv}")
    print(f"[done] summary_json={summary_json}")
    print(f"[done] plot={plot_png}")
    print(f"[recommendation] {recommendation}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
