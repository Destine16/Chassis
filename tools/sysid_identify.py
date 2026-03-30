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
from scipy import optimize


@dataclass
class Dataset:
    path: Path
    meta: dict[str, float]
    t: np.ndarray
    u: np.ndarray
    y: np.ndarray
    sample_hz: float
    dt: float


@dataclass
class Metrics:
    rmse: float
    mae: float
    r2: float
    fit_pct: float

    def as_dict(self) -> dict[str, float]:
        return {
            "rmse": self.rmse,
            "mae": self.mae,
            "r2": self.r2,
            "fit_pct": self.fit_pct,
        }


@dataclass
class ModelResult:
    name: str
    complexity: int
    params: dict[str, float | list[float] | None]
    yhat_train: np.ndarray
    metrics_train: Metrics
    yhat_validate: np.ndarray | None
    metrics_validate: Metrics | None

    def score(self) -> float:
        if self.metrics_validate is not None:
            return self.metrics_validate.fit_pct
        return self.metrics_train.fit_pct


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Identify a low-order wheel-speed model from sysid CSV data. "
            "Supports first-order, first-order+delay, and second-order+delay models."
        )
    )
    parser.add_argument("train", type=Path, help="Training CSV produced by sysid_dump_to_csv.py")
    parser.add_argument(
        "-v",
        "--validate",
        type=Path,
        default=None,
        help="Optional independent validation CSV. If omitted, only the training set is scored.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for plots and JSON summary. Default: alongside train CSV.",
    )
    parser.add_argument(
        "--max-delay-ms",
        type=float,
        default=120.0,
        help="Maximum pure-delay search range in milliseconds. Default: 120 ms.",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        choices=["first_order", "first_order_delay", "second_order_delay"],
        default=["first_order", "first_order_delay", "second_order_delay"],
        help="Candidate model set to evaluate.",
    )
    return parser.parse_args()


def parse_meta_value(value: str) -> float:
    value = value.strip()
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return float(int(value))
    except ValueError as exc:
        raise ValueError(f"invalid meta value: {value}") from exc


def load_sysid_csv(path: Path) -> Dataset:
    meta: dict[str, float] = {}
    samples: list[tuple[float, float, float]] = []
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

            if len(row) < 4:
                continue
            samples.append((float(row[1]), float(row[2]), float(row[3])))

    if not found_sample_header:
        raise ValueError(f"{path} does not contain a sample table header")
    if not samples:
        raise ValueError(f"{path} does not contain sample rows")

    sample_hz = meta.get("sample_hz", 0.0)
    if sample_hz <= 0.0:
        raise ValueError(f"{path} has invalid sample_hz={sample_hz}")

    t = np.array([item[0] for item in samples], dtype=np.float64)
    u = np.array([item[1] for item in samples], dtype=np.float64)
    y = np.array([item[2] for item in samples], dtype=np.float64)
    dt = 1.0 / sample_hz

    return Dataset(path=path, meta=meta, t=t, u=u, y=y, sample_hz=sample_hz, dt=dt)


def compute_metrics(y: np.ndarray, yhat: np.ndarray) -> Metrics:
    err = y - yhat
    rmse = float(np.sqrt(np.mean(err * err)))
    mae = float(np.mean(np.abs(err)))
    y_mean = float(np.mean(y))
    ss_res = float(np.sum(err * err))
    ss_tot = float(np.sum((y - y_mean) ** 2))
    if ss_tot <= 1e-12:
        r2 = 1.0 if ss_res <= 1e-12 else 0.0
        fit_pct = 100.0 if ss_res <= 1e-12 else 0.0
    else:
        r2 = 1.0 - (ss_res / ss_tot)
        fit_pct = 100.0 * (1.0 - np.linalg.norm(err) / np.linalg.norm(y - y_mean))
    return Metrics(rmse=rmse, mae=mae, r2=r2, fit_pct=float(fit_pct))


def simulate_first_order_continuous(
    u: np.ndarray,
    y0: float,
    gain: float,
    tau_s: float,
    bias: float,
    dt: float,
    delay_samples: int,
) -> np.ndarray:
    n = len(u)
    yhat = np.zeros(n, dtype=np.float64)
    yhat[0] = y0
    tau_s = max(float(tau_s), 1e-6)
    a = math.exp(-dt / tau_s)
    b = gain * (1.0 - a)
    x_prev = y0 - bias
    for k in range(1, n):
        u_idx = k - 1 - delay_samples
        uk = u[u_idx] if u_idx >= 0 else 0.0
        x_prev = (a * x_prev) + (b * uk)
        yhat[k] = x_prev + bias
    return yhat


def simulate_first_order_discrete(
    u: np.ndarray,
    y0: float,
    a: float,
    b: float,
    c: float,
    delay_samples: int,
) -> np.ndarray:
    n = len(u)
    yhat = np.zeros(n, dtype=np.float64)
    yhat[0] = y0
    for k in range(1, n):
        u_idx = k - 1 - delay_samples
        uk = u[u_idx] if u_idx >= 0 else 0.0
        yhat[k] = (a * yhat[k - 1]) + (b * uk) + c
    return yhat


def quick_first_order_delay_score(u: np.ndarray, y: np.ndarray, delay_samples: int) -> float:
    start = max(1, delay_samples + 1)
    if len(y) <= start:
        return -np.inf

    rows = []
    target = []
    for k in range(start, len(y)):
        rows.append([y[k - 1], u[k - 1 - delay_samples], 1.0])
        target.append(y[k])

    phi = np.array(rows, dtype=np.float64)
    target_arr = np.array(target, dtype=np.float64)
    theta, *_ = np.linalg.lstsq(phi, target_arr, rcond=None)
    a, b, c = [float(val) for val in theta]
    yhat = simulate_first_order_discrete(u, float(y[0]), a, b, c, delay_samples)
    return compute_metrics(y, yhat).fit_pct


def estimate_gain_guess(u: np.ndarray, y: np.ndarray) -> float:
    var_u = float(np.var(u))
    if var_u <= 1e-12:
        return 0.001
    cov_uy = float(np.mean((u - np.mean(u)) * (y - np.mean(y))))
    gain = cov_uy / var_u
    if abs(gain) < 1e-6:
        gain = 0.001
    return float(gain)


def candidate_first_order_initial_guesses(y: np.ndarray, u: np.ndarray) -> list[tuple[float, float, float]]:
    base_gain = estimate_gain_guess(u, y)
    y_mean = float(np.mean(y))
    gain_scales = [0.5, 1.0, 2.0]
    tau_guesses = [0.02, 0.06, 0.16]

    guesses: list[tuple[float, float, float]] = []
    seen: set[tuple[int, int, int]] = set()
    for scale in gain_scales:
        gain = base_gain * scale
        for tau_s in tau_guesses:
            guess = (gain, tau_s, y_mean)
            key = (
                int(round(guess[0] * 1_000_000)),
                int(round(guess[1] * 1_000_000)),
                int(round(guess[2] * 1_000_000)),
            )
            if key not in seen:
                seen.add(key)
                guesses.append(guess)
    guesses.append((base_gain, 0.06, 0.0))
    guesses.append((0.0, 0.06, y_mean))
    return guesses


def candidate_delay_samples_for_first_order(
    u: np.ndarray,
    y: np.ndarray,
    max_delay_samples: int,
) -> list[int]:
    if max_delay_samples <= 0:
        return [0]

    scores = [
        (delay_samples, quick_first_order_delay_score(u, y, delay_samples))
        for delay_samples in range(0, max_delay_samples + 1)
    ]
    scores.sort(key=lambda item: item[1], reverse=True)

    selected: set[int] = set()
    for delay_samples, _score in scores[:5]:
        for neighbor in (delay_samples - 1, delay_samples, delay_samples + 1):
            if 0 <= neighbor <= max_delay_samples:
                selected.add(neighbor)

    selected.add(0)
    return sorted(selected)


def fit_first_order_for_delay(
    u: np.ndarray,
    y: np.ndarray,
    dt: float,
    delay_samples: int,
) -> tuple[dict[str, float | None], np.ndarray, Metrics]:
    if len(y) < 2:
        raise ValueError("not enough samples for first-order fit")

    y0 = float(y[0])
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    y_span = max(abs(y_max - y_min), 1.0)
    lower_bounds = np.array([-1.0, 1e-4, y_min - y_span], dtype=np.float64)
    upper_bounds = np.array([1.0, 2.0, y_max + y_span], dtype=np.float64)

    best_cost = np.inf
    best_theta: np.ndarray | None = None

    for guess in candidate_first_order_initial_guesses(y, u):
        x0 = np.array(guess, dtype=np.float64)

        def residual(theta: np.ndarray) -> np.ndarray:
            yhat = simulate_first_order_continuous(
                u=u,
                y0=y0,
                gain=float(theta[0]),
                tau_s=float(theta[1]),
                bias=float(theta[2]),
                dt=dt,
                delay_samples=delay_samples,
            )
            return yhat - y

        result = optimize.least_squares(
            residual,
            x0=x0,
            bounds=(lower_bounds, upper_bounds),
            method="trf",
            max_nfev=300,
        )
        if result.cost < best_cost:
            best_cost = float(result.cost)
            best_theta = result.x.copy()

    assert best_theta is not None
    gain = float(best_theta[0])
    tau_s = float(best_theta[1])
    bias = float(best_theta[2])
    yhat = simulate_first_order_continuous(
        u=u,
        y0=y0,
        gain=gain,
        tau_s=tau_s,
        bias=bias,
        dt=dt,
        delay_samples=delay_samples,
    )
    metrics = compute_metrics(y, yhat)

    a = math.exp(-dt / tau_s)
    b = gain * (1.0 - a)
    c = bias * (1.0 - a)
    params: dict[str, float | None] = {
        "gain": gain,
        "tau_s": tau_s,
        "bias": bias,
        "delay_s": delay_samples * dt,
        "delay_samples": float(delay_samples),
        "a": a,
        "b": b,
        "c": c,
    }
    return params, yhat, metrics


def fit_first_order_model(
    name: str,
    complexity: int,
    train: Dataset,
    validate: Dataset | None,
    max_delay_samples: int,
    allow_delay: bool,
) -> ModelResult:
    if allow_delay:
        delay_candidates = candidate_delay_samples_for_first_order(
            train.u, train.y, max_delay_samples
        )
    else:
        delay_candidates = [0]
    best_params: dict[str, float | None] | None = None
    best_yhat_train: np.ndarray | None = None
    best_metrics_train: Metrics | None = None
    best_score = -np.inf

    for delay_samples in delay_candidates:
        params, yhat_train, metrics_train = fit_first_order_for_delay(
            train.u, train.y, train.dt, delay_samples
        )
        score = metrics_train.fit_pct
        if score > best_score:
            best_score = score
            best_params = params
            best_yhat_train = yhat_train
            best_metrics_train = metrics_train

    assert best_params is not None
    assert best_yhat_train is not None
    assert best_metrics_train is not None

    yhat_validate = None
    metrics_validate = None
    if validate is not None:
        yhat_validate = simulate_first_order_continuous(
            validate.u,
            float(validate.y[0]),
            float(best_params["gain"]),
            float(best_params["tau_s"]),
            float(best_params["bias"]),
            validate.dt,
            int(best_params["delay_samples"]),
        )
        metrics_validate = compute_metrics(validate.y, yhat_validate)

    return ModelResult(
        name=name,
        complexity=complexity,
        params=best_params,
        yhat_train=best_yhat_train,
        metrics_train=best_metrics_train,
        yhat_validate=yhat_validate,
        metrics_validate=metrics_validate,
    )


def simulate_second_order(
    u: np.ndarray,
    y0: float,
    gain: float,
    tau1_s: float,
    tau2_s: float,
    bias: float,
    dt: float,
    delay_samples: int,
) -> np.ndarray:
    n = len(u)
    yhat = np.zeros(n, dtype=np.float64)
    if n == 0:
        return yhat

    tau1_s = max(float(tau1_s), 1e-6)
    tau2_s = max(float(tau2_s), 1e-6)
    a1 = math.exp(-dt / tau1_s)
    a2 = math.exp(-dt / tau2_s)

    x1 = 0.0
    if abs(gain) > 1e-9:
        x2 = (y0 - bias) / gain
    else:
        x2 = 0.0
    yhat[0] = y0

    for k in range(1, n):
        u_idx = k - 1 - delay_samples
        uk = u[u_idx] if u_idx >= 0 else 0.0
        x1 = (a1 * x1) + ((1.0 - a1) * uk)
        x2 = (a2 * x2) + ((1.0 - a2) * x1)
        yhat[k] = (gain * x2) + bias
    return yhat


def quick_second_order_delay_score(u: np.ndarray, y: np.ndarray, delay_samples: int) -> float:
    start = max(2, delay_samples + 2)
    if len(y) <= start:
        return -np.inf

    rows = []
    target = []
    for k in range(start, len(y)):
        rows.append(
            [
                y[k - 1],
                y[k - 2],
                u[k - 1 - delay_samples],
                u[k - 2 - delay_samples],
                1.0,
            ]
        )
        target.append(y[k])

    phi = np.array(rows, dtype=np.float64)
    target_arr = np.array(target, dtype=np.float64)
    theta, *_ = np.linalg.lstsq(phi, target_arr, rcond=None)
    a1, a2, b1, b2, c = [float(val) for val in theta]
    yhat = np.zeros(len(u), dtype=np.float64)
    if len(yhat) >= 1:
        yhat[0] = float(y[0])
    if len(yhat) >= 2:
        yhat[1] = float(y[1])
    for k in range(2, len(u)):
        u1_idx = k - 1 - delay_samples
        u2_idx = k - 2 - delay_samples
        u1 = u[u1_idx] if u1_idx >= 0 else 0.0
        u2 = u[u2_idx] if u2_idx >= 0 else 0.0
        yhat[k] = (
            (a1 * yhat[k - 1])
            + (a2 * yhat[k - 2])
            + (b1 * u1)
            + (b2 * u2)
            + c
        )
    return compute_metrics(y, yhat).fit_pct


def candidate_delay_samples_for_second_order(
    u: np.ndarray,
    y: np.ndarray,
    max_delay_samples: int,
) -> list[int]:
    if max_delay_samples <= 0:
        return [0]

    scores = [
        (delay_samples, quick_second_order_delay_score(u, y, delay_samples))
        for delay_samples in range(0, max_delay_samples + 1)
    ]
    scores.sort(key=lambda item: item[1], reverse=True)

    selected: set[int] = set()
    for delay_samples, _score in scores[:5]:
        for neighbor in (delay_samples - 1, delay_samples, delay_samples + 1):
            if 0 <= neighbor <= max_delay_samples:
                selected.add(neighbor)

    selected.add(0)
    return sorted(selected)


def candidate_second_order_initial_guesses(
    y: np.ndarray,
    u: np.ndarray,
) -> list[tuple[float, float, float, float]]:
    base_gain = estimate_gain_guess(u, y)
    y_mean = float(np.mean(y))
    gain_scales = [0.5, 1.0, 1.5]
    tau1_guesses = [0.01, 0.03, 0.08]
    tau2_guesses = [0.03, 0.08, 0.2]

    guesses: list[tuple[float, float, float, float]] = []
    seen: set[tuple[int, int, int, int]] = set()
    for scale in gain_scales:
        gain = base_gain * scale
        for tau1_s in tau1_guesses:
            for tau2_s in tau2_guesses:
                guess = (gain, tau1_s, tau2_s, y_mean)
                key = (
                    int(round(guess[0] * 1_000_000)),
                    int(round(guess[1] * 1_000_000)),
                    int(round(guess[2] * 1_000_000)),
                    int(round(guess[3] * 1_000_000)),
                )
                if key not in seen:
                    seen.add(key)
                    guesses.append(guess)
    guesses.append((base_gain, 0.03, 0.1, 0.0))
    return guesses


def fit_second_order_for_delay(
    u: np.ndarray,
    y: np.ndarray,
    dt: float,
    delay_samples: int,
) -> tuple[dict[str, float | list[float]], np.ndarray, Metrics]:
    if len(y) < 3:
        raise ValueError("not enough samples for second-order fit")

    y0 = float(y[0])
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    y_span = max(abs(y_max - y_min), 1.0)
    lower_bounds = np.array([-1.0, 1e-4, 1e-4, y_min - y_span], dtype=np.float64)
    upper_bounds = np.array([1.0, 2.0, 2.0, y_max + y_span], dtype=np.float64)

    best_cost = np.inf
    best_theta: np.ndarray | None = None

    for guess in candidate_second_order_initial_guesses(y, u):
        x0 = np.array(guess, dtype=np.float64)

        def residual(theta: np.ndarray) -> np.ndarray:
            yhat = simulate_second_order(
                u=u,
                y0=y0,
                gain=float(theta[0]),
                tau1_s=float(theta[1]),
                tau2_s=float(theta[2]),
                bias=float(theta[3]),
                dt=dt,
                delay_samples=delay_samples,
            )
            return yhat - y

        result = optimize.least_squares(
            residual,
            x0=x0,
            bounds=(lower_bounds, upper_bounds),
            method="trf",
            max_nfev=400,
        )
        if result.cost < best_cost:
            best_cost = float(result.cost)
            best_theta = result.x.copy()

    assert best_theta is not None
    gain = float(best_theta[0])
    tau1_s = float(best_theta[1])
    tau2_s = float(best_theta[2])
    bias = float(best_theta[3])

    yhat = simulate_second_order(
        u=u,
        y0=y0,
        gain=gain,
        tau1_s=tau1_s,
        tau2_s=tau2_s,
        bias=bias,
        dt=dt,
        delay_samples=delay_samples,
    )
    metrics = compute_metrics(y, yhat)

    a1 = math.exp(-dt / tau1_s)
    a2 = math.exp(-dt / tau2_s)
    poles = [
        {"real": a1, "imag": 0.0},
        {"real": a2, "imag": 0.0},
    ]

    params: dict[str, float | list[float]] = {
        "gain": gain,
        "tau1_s": tau1_s,
        "tau2_s": tau2_s,
        "bias": bias,
        "delay_s": delay_samples * dt,
        "delay_samples": float(delay_samples),
        "poles": poles,
    }
    return params, yhat, metrics


def fit_second_order_model(
    train: Dataset,
    validate: Dataset | None,
    max_delay_samples: int,
) -> ModelResult:
    best_params: dict[str, float | list[float]] | None = None
    best_yhat_train: np.ndarray | None = None
    best_metrics_train: Metrics | None = None
    best_score = -np.inf

    delay_candidates = candidate_delay_samples_for_second_order(
        train.u, train.y, max_delay_samples
    )

    for delay_samples in delay_candidates:
        params, yhat_train, metrics_train = fit_second_order_for_delay(
            train.u, train.y, train.dt, delay_samples
        )
        score = metrics_train.fit_pct
        if score > best_score:
            best_score = score
            best_params = params
            best_yhat_train = yhat_train
            best_metrics_train = metrics_train

    assert best_params is not None
    assert best_yhat_train is not None
    assert best_metrics_train is not None

    yhat_validate = None
    metrics_validate = None
    if validate is not None:
        yhat_validate = simulate_second_order(
            validate.u,
            float(validate.y[0]),
            float(best_params["gain"]),
            float(best_params["tau1_s"]),
            float(best_params["tau2_s"]),
            float(best_params["bias"]),
            validate.dt,
            int(best_params["delay_samples"]),
        )
        metrics_validate = compute_metrics(validate.y, yhat_validate)

    return ModelResult(
        name="second_order_delay",
        complexity=3,
        params=best_params,
        yhat_train=best_yhat_train,
        metrics_train=best_metrics_train,
        yhat_validate=yhat_validate,
        metrics_validate=metrics_validate,
    )


def choose_best_model(results: list[ModelResult]) -> ModelResult:
    best_score = max(result.score() for result in results)
    near_best = [result for result in results if (best_score - result.score()) <= 1.0]
    near_best.sort(key=lambda item: (item.complexity, -item.score()))
    return near_best[0]


def format_metrics(metrics: Metrics) -> str:
    return (
        f"fit={metrics.fit_pct:.2f}%  "
        f"R2={metrics.r2:.4f}  "
        f"RMSE={metrics.rmse:.6f}  "
        f"MAE={metrics.mae:.6f}"
    )


def print_summary(results: list[ModelResult], best: ModelResult) -> None:
    print("candidate models:")
    for result in results:
        train_summary = format_metrics(result.metrics_train)
        if result.metrics_validate is not None:
            val_summary = format_metrics(result.metrics_validate)
            print(f"- {result.name}:")
            print(f"  train    {train_summary}")
            print(f"  validate {val_summary}")
        else:
            print(f"- {result.name}: {train_summary}")
        print(f"  params   {json.dumps(result.params, ensure_ascii=False)}")
    print()
    print(f"recommended model: {best.name}")


def save_json_summary(output_dir: Path, train: Dataset, validate: Dataset | None, results: list[ModelResult], best: ModelResult) -> None:
    payload = {
        "train_csv": str(train.path),
        "validate_csv": str(validate.path) if validate is not None else None,
        "sample_hz": train.sample_hz,
        "dt_s": train.dt,
        "recommended_model": best.name,
        "models": [
            {
                "name": result.name,
                "complexity": result.complexity,
                "params": result.params,
                "metrics_train": result.metrics_train.as_dict(),
                "metrics_validate": result.metrics_validate.as_dict()
                if result.metrics_validate is not None
                else None,
            }
            for result in results
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def plot_comparison(
    output_path: Path,
    title: str,
    dataset: Dataset,
    series: list[tuple[str, np.ndarray]],
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ax_u, ax_y = axes

    ax_u.plot(dataset.t, dataset.u, color="black", linewidth=1.0)
    ax_u.set_ylabel("u_raw")
    ax_u.set_title(title)
    ax_u.grid(True, alpha=0.3)

    ax_y.plot(dataset.t, dataset.y, label="actual", linewidth=1.5, color="black")
    for name, yhat in series:
        ax_y.plot(dataset.t, yhat, label=name, linewidth=1.0)
    ax_y.set_xlabel("time (s)")
    ax_y.set_ylabel("speed (rad/s)")
    ax_y.grid(True, alpha=0.3)
    ax_y.legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_best_residuals(
    output_path: Path,
    best: ModelResult,
    train: Dataset,
    validate: Dataset | None,
) -> None:
    rows = 2 if validate is not None and best.yhat_validate is not None else 1
    fig, axes = plt.subplots(rows, 1, figsize=(12, 4 * rows), sharex=False)
    if rows == 1:
        axes = [axes]

    train_res = train.y - best.yhat_train
    axes[0].plot(train.t, train_res, color="tab:red", linewidth=1.0)
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_title(f"{best.name} residuals (train)")
    axes[0].set_ylabel("error")
    axes[0].grid(True, alpha=0.3)

    if validate is not None and best.yhat_validate is not None:
        val_res = validate.y - best.yhat_validate
        axes[1].plot(validate.t, val_res, color="tab:red", linewidth=1.0)
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set_title(f"{best.name} residuals (validate)")
        axes[1].set_ylabel("error")
        axes[1].set_xlabel("time (s)")
        axes[1].grid(True, alpha=0.3)
    else:
        axes[0].set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def default_output_dir(train_path: Path) -> Path:
    return train_path.with_suffix("").parent / f"{train_path.stem}_identify"


def main() -> int:
    args = parse_args()

    try:
        train = load_sysid_csv(args.train)
        validate = load_sysid_csv(args.validate) if args.validate is not None else None
    except Exception as exc:
        print(f"error: failed to load csv: {exc}", file=sys.stderr)
        return 1

    if validate is not None and abs(validate.sample_hz - train.sample_hz) > 1e-9:
        print(
            "error: training and validation datasets must have the same sample_hz",
            file=sys.stderr,
        )
        return 1

    dt = train.dt
    max_delay_samples = max(0, int(round((args.max_delay_ms / 1000.0) / dt)))
    output_dir = args.output_dir if args.output_dir is not None else default_output_dir(args.train)
    output_dir.mkdir(parents=True, exist_ok=True)

    results: list[ModelResult] = []

    if "first_order" in args.models:
        results.append(
            fit_first_order_model(
                name="first_order",
                complexity=1,
                train=train,
                validate=validate,
                max_delay_samples=0,
                allow_delay=False,
            )
        )
    if "first_order_delay" in args.models:
        results.append(
            fit_first_order_model(
                name="first_order_delay",
                complexity=2,
                train=train,
                validate=validate,
                max_delay_samples=max_delay_samples,
                allow_delay=True,
            )
        )
    if "second_order_delay" in args.models:
        results.append(fit_second_order_model(train, validate, max_delay_samples))

    if not results:
        print("error: no candidate models selected", file=sys.stderr)
        return 1

    best = choose_best_model(results)
    print_summary(results, best)

    save_json_summary(output_dir, train, validate, results, best)
    plot_comparison(
        output_dir / "train_comparison.png",
        "Training Set: actual vs model outputs",
        train,
        [(result.name, result.yhat_train) for result in results],
    )
    if validate is not None:
        plot_comparison(
            output_dir / "validate_comparison.png",
            "Validation Set: actual vs model outputs",
            validate,
            [
                (result.name, result.yhat_validate)
                for result in results
                if result.yhat_validate is not None
            ],
        )
    plot_best_residuals(output_dir / "best_model_residuals.png", best, train, validate)

    print()
    print(f"wrote {output_dir / 'summary.json'}")
    print(f"wrote {output_dir / 'train_comparison.png'}")
    if validate is not None:
        print(f"wrote {output_dir / 'validate_comparison.png'}")
    print(f"wrote {output_dir / 'best_model_residuals.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
