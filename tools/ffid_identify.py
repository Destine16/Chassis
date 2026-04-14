#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize, signal


SPECIAL_STEPS = {"pre_hold", "mid_hold", "post_hold", "done"}


@dataclass
class Dataset:
    path: Path
    meta: dict[str, float]
    sample_idx: np.ndarray
    t: np.ndarray
    u: np.ndarray
    step_id: np.ndarray
    step_name: np.ndarray
    speed_ref: np.ndarray
    speed: np.ndarray
    speed_smooth: np.ndarray
    sample_hz: float


@dataclass
class StepSummary:
    step_id: int
    step_name: str
    speed_ref_radps: float
    count_total: int
    count_used: int
    u_mean: float
    u_std: float
    speed_mean: float
    speed_std: float

    def as_dict(self) -> dict[str, object]:
        return {
            "step_id": self.step_id,
            "step_name": self.step_name,
            "speed_ref_radps": self.speed_ref_radps,
            "count_total": self.count_total,
            "count_used": self.count_used,
            "u_mean": self.u_mean,
            "u_std": self.u_std,
            "speed_mean": self.speed_mean,
            "speed_std": self.speed_std,
        }


@dataclass
class BreakawayResult:
    direction: str
    found: bool
    step_id: int | None
    step_name: str | None
    speed_ref_radps: float | None
    u_mean: float | None
    speed_mean: float | None

    def as_dict(self) -> dict[str, object]:
        return {
            "direction": self.direction,
            "found": self.found,
            "step_id": self.step_id,
            "step_name": self.step_name,
            "speed_ref_radps": self.speed_ref_radps,
            "u_mean": self.u_mean,
            "speed_mean": self.speed_mean,
        }


@dataclass
class FitMetrics:
    rmse_u: float
    mae_u: float
    r2_u: float
    nrmse_u_pct: float
    p95_abs_err_u: float
    mae_u_pos: float
    mae_u_neg: float
    used_samples: int

    def as_dict(self) -> dict[str, float | int]:
        return {
            "rmse_u": self.rmse_u,
            "mae_u": self.mae_u,
            "r2_u": self.r2_u,
            "nrmse_u_pct": self.nrmse_u_pct,
            "p95_abs_err_u": self.p95_abs_err_u,
            "mae_u_pos": self.mae_u_pos,
            "mae_u_neg": self.mae_u_neg,
            "used_samples": self.used_samples,
        }


@dataclass
class FitResult:
    name: str
    params: dict[str, float]
    metrics: FitMetrics
    u_hat: np.ndarray
    residual_u: np.ndarray

    def score(self) -> float:
        return self.metrics.nrmse_u_pct

    def as_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "params": self.params,
            "metrics": self.metrics.as_dict(),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Identify single-wheel feedforward parameters from FFID speed-platform CSV data. "
            "Fits kS/kV on steady-state samples from positive/negative speed plateaus."
        )
    )
    parser.add_argument("csv", type=Path, help="CSV produced by ffid_dump_to_csv.py")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for JSON summary and plots. Default: alongside CSV.",
    )
    parser.add_argument(
        "--speed-threshold",
        type=float,
        default=0.8,
        help="Breakaway speed threshold in rad/s. Default: 0.8",
    )
    parser.add_argument(
        "--fit-speed-min",
        type=float,
        default=0.5,
        help="Minimum absolute steady speed to include in regression. Default: 0.5",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=21,
        help="Odd Savitzky-Golay window for speed smoothing. Default: 21",
    )
    parser.add_argument(
        "--smooth-poly",
        type=int,
        default=2,
        help="Savitzky-Golay polynomial order. Default: 2",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        choices=["symmetric", "asymmetric"],
        default=["symmetric", "asymmetric"],
        help="Candidate feedforward models to evaluate. Default: symmetric asymmetric",
    )
    return parser.parse_args()


def default_output_dir(csv_path: Path) -> Path:
    return csv_path.with_suffix("").parent / f"{csv_path.stem}_ffid"


def parse_meta_value(value: str) -> float:
    value = value.strip()
    if any(ch in value for ch in (".", "e", "E")):
        return float(value)
    return float(int(value))


def smooth_speed(speed: np.ndarray, window: int, poly: int) -> np.ndarray:
    if len(speed) < 5:
        return speed.copy()

    window = max(5, window)
    if window % 2 == 0:
        window += 1
    if window > len(speed):
        window = len(speed) if len(speed) % 2 == 1 else len(speed) - 1
    if window < 5:
        return speed.copy()
    poly = min(poly, window - 2)
    return signal.savgol_filter(speed, window_length=window, polyorder=poly, mode="interp")


def load_ffid_csv(path: Path, smooth_window: int, smooth_poly: int) -> Dataset:
    meta: dict[str, float] = {}
    rows: list[tuple[int, float, float, int, str, float, float]] = []
    found_sample_header = False

    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.reader(fp)
        for row in reader:
            if not row:
                continue
            if row[0] == "meta_key":
                continue
            if row[0] == "sample_idx":
                found_sample_header = True
                continue

            if not found_sample_header:
                if len(row) >= 2:
                    meta[row[0].strip()] = parse_meta_value(row[1])
                continue

            if len(row) < 7:
                continue
            rows.append(
                (
                    int(row[0]),
                    float(row[1]),
                    float(row[2]),
                    int(row[3]),
                    row[4].strip(),
                    float(row[5]),
                    float(row[6]),
                )
            )

    if not found_sample_header:
        raise ValueError(f"{path} does not contain a sample table header")
    if not rows:
        raise ValueError(f"{path} does not contain sample rows")

    sample_hz = meta.get("sample_hz", 0.0)
    if sample_hz <= 0.0:
        raise ValueError(f"{path} has invalid sample_hz={sample_hz}")

    sample_idx = np.array([row[0] for row in rows], dtype=np.int64)
    t = np.array([row[1] for row in rows], dtype=np.float64)
    u = np.array([row[2] for row in rows], dtype=np.float64)
    step_id = np.array([row[3] for row in rows], dtype=np.int64)
    step_name = np.array([row[4] for row in rows], dtype=object)
    speed_ref = np.array([row[5] for row in rows], dtype=np.float64)
    speed = np.array([row[6] for row in rows], dtype=np.float64)
    speed_smooth = smooth_speed(speed, smooth_window, smooth_poly)

    return Dataset(
        path=path,
        meta=meta,
        sample_idx=sample_idx,
        t=t,
        u=u,
        step_id=step_id,
        step_name=step_name,
        speed_ref=speed_ref,
        speed=speed,
        speed_smooth=speed_smooth,
        sample_hz=sample_hz,
    )


def summarize_steps(dataset: Dataset) -> list[StepSummary]:
    settle_skip_ms = dataset.meta.get("settle_skip_ms", 0.0)
    settle_skip_n = int(round(settle_skip_ms * dataset.sample_hz / 1000.0))
    summaries: list[StepSummary] = []

    unique_steps = list(dict.fromkeys(dataset.step_id.tolist()))
    for sid in unique_steps:
        idx = np.flatnonzero(dataset.step_id == sid)
        if idx.size == 0:
            continue

        start = min(settle_skip_n, max(0, idx.size - 1))
        used_idx = idx[start:]
        if used_idx.size == 0:
            used_idx = idx[-1:]

        summaries.append(
            StepSummary(
                step_id=int(sid),
                step_name=str(dataset.step_name[idx[0]]),
                speed_ref_radps=float(dataset.speed_ref[idx[0]]),
                count_total=int(idx.size),
                count_used=int(used_idx.size),
                u_mean=float(np.mean(dataset.u[used_idx])),
                u_std=float(np.std(dataset.u[used_idx])),
                speed_mean=float(np.mean(dataset.speed_smooth[used_idx])),
                speed_std=float(np.std(dataset.speed_smooth[used_idx])),
            )
        )

    return summaries


def build_fit_mask(dataset: Dataset) -> np.ndarray:
    settle_skip_ms = dataset.meta.get("settle_skip_ms", 0.0)
    settle_skip_n = int(round(settle_skip_ms * dataset.sample_hz / 1000.0))
    mask = np.zeros(len(dataset.sample_idx), dtype=bool)

    unique_steps = list(dict.fromkeys(dataset.step_id.tolist()))
    for sid in unique_steps:
        idx = np.flatnonzero(dataset.step_id == sid)
        if idx.size == 0:
            continue
        if str(dataset.step_name[idx[0]]) in SPECIAL_STEPS:
            continue
        start = min(settle_skip_n, max(0, idx.size - 1))
        mask[idx[start:]] = True

    return mask


def detect_breakaway(
    step_summaries: list[StepSummary],
    direction: str,
    speed_threshold: float,
) -> BreakawayResult:
    if direction == "positive":
        candidates = [item for item in step_summaries if item.speed_ref_radps > 0.0]
        candidates.sort(key=lambda item: item.speed_ref_radps)
        for item in candidates:
            if item.speed_mean >= speed_threshold:
                return BreakawayResult(
                    direction=direction,
                    found=True,
                    step_id=item.step_id,
                    step_name=item.step_name,
                    speed_ref_radps=item.speed_ref_radps,
                    u_mean=item.u_mean,
                    speed_mean=item.speed_mean,
                )
    else:
        candidates = [item for item in step_summaries if item.speed_ref_radps < 0.0]
        candidates.sort(key=lambda item: item.speed_ref_radps, reverse=True)
        for item in candidates:
            if item.speed_mean <= -speed_threshold:
                return BreakawayResult(
                    direction=direction,
                    found=True,
                    step_id=item.step_id,
                    step_name=item.step_name,
                    speed_ref_radps=item.speed_ref_radps,
                    u_mean=item.u_mean,
                    speed_mean=item.speed_mean,
                )

    return BreakawayResult(direction=direction, found=False, step_id=None, step_name=None, speed_ref_radps=None, u_mean=None, speed_mean=None)


def predict_symmetric(params: np.ndarray, w: np.ndarray) -> np.ndarray:
    k_s, k_v = params
    return (k_s * np.sign(w)) + (k_v * w)


def predict_asymmetric(params: np.ndarray, w: np.ndarray) -> np.ndarray:
    k_s_pos, k_v_pos, k_s_neg, k_v_neg = params
    u_hat = np.empty_like(w)
    pos = w >= 0.0
    neg = ~pos
    u_hat[pos] = k_s_pos + (k_v_pos * w[pos])
    u_hat[neg] = -k_s_neg + (k_v_neg * w[neg])
    return u_hat


def compute_metrics(u: np.ndarray, u_hat: np.ndarray, w: np.ndarray) -> FitMetrics:
    err = u - u_hat
    rmse = float(np.sqrt(np.mean(err * err)))
    mae = float(np.mean(np.abs(err)))
    p95_abs_err_u = float(np.percentile(np.abs(err), 95))
    u_mean = float(np.mean(u))
    ss_res = float(np.sum(err * err))
    ss_tot = float(np.sum((u - u_mean) ** 2))
    u_span = float(np.max(u) - np.min(u))
    if ss_tot <= 1e-12:
        r2 = 1.0 if ss_res <= 1e-12 else 0.0
    else:
        r2 = 1.0 - (ss_res / ss_tot)
    if u_span <= 1e-12:
        nrmse_u_pct = 0.0 if rmse <= 1e-12 else math.inf
    else:
        nrmse_u_pct = float(100.0 * rmse / u_span)

    pos = w >= 0.0
    neg = ~pos
    mae_u_pos = float(np.mean(np.abs(err[pos]))) if np.any(pos) else 0.0
    mae_u_neg = float(np.mean(np.abs(err[neg]))) if np.any(neg) else 0.0
    return FitMetrics(
        rmse_u=rmse,
        mae_u=mae,
        r2_u=float(r2),
        nrmse_u_pct=nrmse_u_pct,
        p95_abs_err_u=p95_abs_err_u,
        mae_u_pos=mae_u_pos,
        mae_u_neg=mae_u_neg,
        used_samples=len(u),
    )


def fit_symmetric(w: np.ndarray, u: np.ndarray, breakaway: dict[str, BreakawayResult]) -> FitResult:
    k_s_guess = 0.0
    for key in ("positive", "negative"):
        result = breakaway[key]
        if result.found and result.u_mean is not None:
            k_s_guess += abs(result.u_mean)
    if k_s_guess > 0.0:
        k_s_guess *= 0.5
    else:
        k_s_guess = max(1.0, float(np.percentile(np.abs(u), 10)))

    k_v_guess = max(1e-3, float(np.mean(np.abs(u)) / max(np.mean(np.abs(w)), 1e-3)))
    x0 = np.array([k_s_guess, k_v_guess], dtype=np.float64)

    result = optimize.least_squares(
        fun=lambda x: predict_symmetric(x, w) - u,
        x0=x0,
        bounds=([0.0, 0.0], [np.inf, np.inf]),
        loss="soft_l1",
    )

    params = {"kS": float(result.x[0]), "kV": float(result.x[1])}
    u_hat = predict_symmetric(result.x, w)
    metrics = compute_metrics(u, u_hat, w)
    return FitResult(
        name="symmetric",
        params=params,
        metrics=metrics,
        u_hat=u_hat,
        residual_u=u - u_hat,
    )


def fit_asymmetric(w: np.ndarray, u: np.ndarray, breakaway: dict[str, BreakawayResult]) -> FitResult:
    k_s_pos_guess = abs(breakaway["positive"].u_mean) if breakaway["positive"].found and breakaway["positive"].u_mean is not None else 1.0
    k_s_neg_guess = abs(breakaway["negative"].u_mean) if breakaway["negative"].found and breakaway["negative"].u_mean is not None else 1.0
    pos = w >= 0.0
    neg = ~pos
    k_v_pos_guess = max(1e-3, float(np.mean(np.abs(u[pos])) / max(np.mean(np.abs(w[pos])), 1e-3))) if np.any(pos) else 1.0
    k_v_neg_guess = max(1e-3, float(np.mean(np.abs(u[neg])) / max(np.mean(np.abs(w[neg])), 1e-3))) if np.any(neg) else 1.0
    x0 = np.array([k_s_pos_guess, k_v_pos_guess, k_s_neg_guess, k_v_neg_guess], dtype=np.float64)

    result = optimize.least_squares(
        fun=lambda x: predict_asymmetric(x, w) - u,
        x0=x0,
        bounds=([0.0, 0.0, 0.0, 0.0], [np.inf, np.inf, np.inf, np.inf]),
        loss="soft_l1",
    )

    params = {
        "kS_pos": float(result.x[0]),
        "kV_pos": float(result.x[1]),
        "kS_neg": float(result.x[2]),
        "kV_neg": float(result.x[3]),
    }
    u_hat = predict_asymmetric(result.x, w)
    metrics = compute_metrics(u, u_hat, w)
    return FitResult(
        name="asymmetric",
        params=params,
        metrics=metrics,
        u_hat=u_hat,
        residual_u=u - u_hat,
    )


def choose_recommended(results: list[FitResult]) -> str:
    if not results:
        raise ValueError("no fit results available")

    by_name = {result.name: result for result in results}
    if "symmetric" not in by_name or "asymmetric" not in by_name:
        return min(results, key=lambda item: item.score()).name

    sym = by_name["symmetric"]
    asym = by_name["asymmetric"]
    improvement = (sym.metrics.nrmse_u_pct - asym.metrics.nrmse_u_pct) / max(sym.metrics.nrmse_u_pct, 1e-9)
    if improvement >= 0.08:
        return "asymmetric"
    return "symmetric"


def _format_metric_box(result: FitResult, recommended_model: str, breakaway: dict[str, BreakawayResult]) -> str:
    lines = [f"{result.name}{'  [recommended]' if result.name == recommended_model else ''}"]

    if result.name == "symmetric":
        lines.append(f"kS = {result.params['kS']:.2f}")
        lines.append(f"kV = {result.params['kV']:.3f}")
    else:
        lines.append(f"kS+ = {result.params['kS_pos']:.2f}")
        lines.append(f"kV+ = {result.params['kV_pos']:.3f}")
        lines.append(f"kS- = {result.params['kS_neg']:.2f}")
        lines.append(f"kV- = {result.params['kV_neg']:.3f}")

    lines.append(f"RMSE = {result.metrics.rmse_u:.2f}")
    lines.append(f"NRMSE = {result.metrics.nrmse_u_pct:.2f}%")
    lines.append(f"P95 = {result.metrics.p95_abs_err_u:.2f}")
    lines.append(f"R2 = {result.metrics.r2_u:.5f}")
    lines.append(f"MAE+ = {result.metrics.mae_u_pos:.2f}")
    lines.append(f"MAE- = {result.metrics.mae_u_neg:.2f}")

    pos_break = breakaway["positive"]
    neg_break = breakaway["negative"]
    if pos_break.found and pos_break.u_mean is not None:
        lines.append(f"u_break+ = {pos_break.u_mean:.1f}")
    if neg_break.found and neg_break.u_mean is not None:
        lines.append(f"u_break- = {neg_break.u_mean:.1f}")

    return "\n".join(lines)


def plot_overview(dataset: Dataset, output_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(dataset.t, dataset.speed_ref, color="tab:blue", linewidth=1.2, label="speed ref")
    axes[0].plot(dataset.t, dataset.speed_smooth, color="tab:red", linewidth=1.0, alpha=0.9, label="speed meas")
    axes[0].set_ylabel("speed (rad/s)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].plot(dataset.t, dataset.u, color="tab:orange", linewidth=1.2, label="current cmd")
    axes[1].set_ylabel("current raw")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")

    speed_err = dataset.speed_smooth - dataset.speed_ref
    axes[2].plot(dataset.t, speed_err, color="tab:green", linewidth=1.0, label="speed error")
    axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[2].set_ylabel("speed err")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(loc="upper right")

    fig.suptitle("FFID speed-platform overview")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_step_summary(step_summaries: list[StepSummary], output_path: Path) -> None:
    nonzero = [item for item in step_summaries if item.step_name not in SPECIAL_STEPS]
    nonzero.sort(key=lambda item: item.speed_ref_radps)

    x = np.array([item.speed_mean for item in nonzero], dtype=np.float64)
    y = np.array([item.u_mean for item in nonzero], dtype=np.float64)
    xerr = np.array([item.speed_std for item in nonzero], dtype=np.float64)
    yerr = np.array([item.u_std for item in nonzero], dtype=np.float64)
    labels = [item.step_name for item in nonzero]

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.errorbar(x, y, xerr=xerr, yerr=yerr, fmt="o", capsize=3, color="tab:purple", ecolor="tab:gray")
    for item, label in zip(nonzero, labels):
        ax.annotate(label, (item.speed_mean, item.u_mean), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.set_xlabel("steady speed mean (rad/s)")
    ax.set_ylabel("steady current mean")
    ax.set_title("FFID step summary")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_fit(
    fit_w: np.ndarray,
    fit_u: np.ndarray,
    step_summaries: list[StepSummary],
    results: list[FitResult],
    recommended_model: str,
    breakaway: dict[str, BreakawayResult],
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.scatter(fit_w, fit_u, s=8, alpha=0.20, color="tab:gray", label="steady samples")

    nonzero = [item for item in step_summaries if item.step_name not in SPECIAL_STEPS]
    x = np.array([item.speed_mean for item in nonzero], dtype=np.float64)
    y = np.array([item.u_mean for item in nonzero], dtype=np.float64)
    ax.scatter(x, y, s=40, color="black", marker="o", label="step means")

    if fit_w.size > 0:
        w_min = float(np.min(fit_w))
        w_max = float(np.max(fit_w))
        w_grid = np.linspace(w_min, w_max, 500)
        for result in results:
            if result.name == "symmetric":
                curve = predict_symmetric(np.array([result.params["kS"], result.params["kV"]]), w_grid)
            else:
                curve = predict_asymmetric(
                    np.array(
                        [
                            result.params["kS_pos"],
                            result.params["kV_pos"],
                            result.params["kS_neg"],
                            result.params["kV_neg"],
                        ]
                    ),
                    w_grid,
                )
            ax.plot(w_grid, curve, linewidth=2.0, label=f"{result.name} fit")

    colors = {"symmetric": "tab:blue", "asymmetric": "tab:orange"}
    for box_idx, result in enumerate(results):
        ax.text(
            1.02,
            0.98 - (0.43 * box_idx),
            _format_metric_box(result, recommended_model, breakaway),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox={
                "boxstyle": "round",
                "facecolor": "white",
                "edgecolor": colors.get(result.name, "black"),
                "alpha": 0.92,
            },
        )

    for key, color in (("positive", "tab:green"), ("negative", "tab:red")):
        item = breakaway[key]
        if item.found and item.speed_mean is not None and item.u_mean is not None:
            ax.scatter([item.speed_mean], [item.u_mean], color=color, s=70, marker="x", linewidths=2.0, label=f"{key} breakaway")

    ax.set_xlabel("wheel speed (rad/s)")
    ax.set_ylabel("current raw")
    ax.set_title(f"FFID steady current-vs-speed fit (recommended: {recommended_model})")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout(rect=(0.0, 0.0, 0.76, 1.0))
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_residuals(dataset: Dataset, fit_mask: np.ndarray, results: list[FitResult], output_path: Path) -> None:
    fig, axes = plt.subplots(len(results), 1, figsize=(12, 4 * len(results)), sharex=True)
    if len(results) == 1:
        axes = [axes]

    t = dataset.t[fit_mask]
    for ax, result in zip(axes, results):
        ax.plot(t, result.residual_u, linewidth=1.0, label=f"{result.name} residual")
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        ax.set_ylabel("u residual")
        ax.set_title(
            f"{result.name} residuals | RMSE={result.metrics.rmse_u:.2f}, "
            f"NRMSE={result.metrics.nrmse_u_pct:.2f}%, P95={result.metrics.p95_abs_err_u:.2f}"
        )
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_summary(
    dataset: Dataset,
    step_summaries: list[StepSummary],
    breakaway: dict[str, BreakawayResult],
    results: list[FitResult],
    recommended_model: str,
    fit_speed_min: float,
    fit_mask: np.ndarray,
) -> dict[str, object]:
    asymmetry: dict[str, float] = {}
    pos_break = breakaway["positive"]
    neg_break = breakaway["negative"]
    if pos_break.found and neg_break.found and pos_break.u_mean is not None and neg_break.u_mean is not None:
        asymmetry["breakaway_abs_delta_u_raw"] = float(abs(abs(pos_break.u_mean) - abs(neg_break.u_mean)))

    for result in results:
        if result.name == "asymmetric":
            k_s_pos = result.params["kS_pos"]
            k_s_neg = result.params["kS_neg"]
            k_v_pos = result.params["kV_pos"]
            k_v_neg = result.params["kV_neg"]
            if k_s_pos > 1e-9:
                asymmetry["kS_ratio_neg_over_pos"] = float(k_s_neg / k_s_pos)
            if k_v_pos > 1e-9:
                asymmetry["kV_ratio_neg_over_pos"] = float(k_v_neg / k_v_pos)

    return {
        "source_csv": str(dataset.path),
        "sample_hz": dataset.sample_hz,
        "meta": dataset.meta,
        "fit_speed_min_radps": fit_speed_min,
        "fit_sample_count": int(np.count_nonzero(fit_mask)),
        "breakaway": {
            "positive": breakaway["positive"].as_dict(),
            "negative": breakaway["negative"].as_dict(),
        },
        "asymmetry": asymmetry,
        "step_summaries": [item.as_dict() for item in step_summaries],
        "models": [result.as_dict() for result in results],
        "recommended_model": recommended_model,
        "recommended_formula": (
            "u_ff = kS * sign(w_ref) + kV * w_ref"
            if recommended_model == "symmetric"
            else "u_ff = (w_ref >= 0 ? kS_pos + kV_pos*w_ref : -kS_neg + kV_neg*w_ref)"
        ),
    }


def write_summary(summary: dict[str, object], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8") as fp:
        json.dump(summary, fp, indent=2, ensure_ascii=True)
        fp.write("\n")


def main() -> int:
    args = parse_args()

    try:
        dataset = load_ffid_csv(args.csv, args.smooth_window, args.smooth_poly)
        output_dir = args.output_dir if args.output_dir is not None else default_output_dir(args.csv)
        output_dir.mkdir(parents=True, exist_ok=True)

        step_summaries = summarize_steps(dataset)
        fit_mask = build_fit_mask(dataset)
        fit_mask &= np.abs(dataset.speed_smooth) >= args.fit_speed_min
        if np.count_nonzero(fit_mask) == 0:
            raise ValueError("no steady-state samples remain after applying fit_speed_min filter")

        breakaway = {
            "positive": detect_breakaway(step_summaries, "positive", args.speed_threshold),
            "negative": detect_breakaway(step_summaries, "negative", args.speed_threshold),
        }

        fit_w = dataset.speed_smooth[fit_mask]
        fit_u = dataset.u[fit_mask]

        results: list[FitResult] = []
        if "symmetric" in args.models:
            results.append(fit_symmetric(fit_w, fit_u, breakaway))
        if "asymmetric" in args.models:
            results.append(fit_asymmetric(fit_w, fit_u, breakaway))

        recommended_model = choose_recommended(results)
        summary = build_summary(dataset, step_summaries, breakaway, results, recommended_model, args.fit_speed_min, fit_mask)

        write_summary(summary, output_dir / "summary.json")
        plot_overview(dataset, output_dir / "overview.png")
        plot_step_summary(step_summaries, output_dir / "step_summary.png")
        plot_fit(fit_w, fit_u, step_summaries, results, recommended_model, breakaway, output_dir / "fit_scatter.png")
        plot_residuals(dataset, fit_mask, results, output_dir / "residuals.png")
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    recommended = next(result for result in results if result.name == recommended_model)
    print(f"wrote {output_dir / 'summary.json'}")
    print(f"wrote {output_dir / 'overview.png'}")
    print(f"wrote {output_dir / 'step_summary.png'}")
    print(f"wrote {output_dir / 'fit_scatter.png'}")
    print(f"wrote {output_dir / 'residuals.png'}")
    print(
        "recommended:"
        f" model={recommended_model}"
        f" params={json.dumps(recommended.params, ensure_ascii=True)}"
        f" nrmse_u_pct={recommended.metrics.nrmse_u_pct:.3f}"
        f" used_samples={recommended.metrics.used_samples}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
