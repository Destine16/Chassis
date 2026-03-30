#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
class StepProfile:
    t: np.ndarray
    ref: np.ndarray
    step_start_idx: int
    step_end_idx: int
    release_end_idx: int
    step_value: float


@dataclass
class TuneResult:
    style: str
    kp: float
    ki: float
    kd: float
    cost: float
    metrics: dict[str, float]
    t: np.ndarray
    ref: np.ndarray
    y: np.ndarray
    u: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Tune wheel-speed PID gains from an identified model summary. "
            "Uses the same PID form as the firmware."
        )
    )
    parser.add_argument("summary", type=Path, help="summary.json produced by sysid_identify.py")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory for plots and JSON summary. Default: alongside summary.json.",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="Override model name. Default: use recommended_model from summary.json.",
    )
    parser.add_argument(
        "--step-radps",
        type=float,
        default=10.0,
        help="Step reference amplitude in rad/s. Default: 10.0",
    )
    parser.add_argument(
        "--pre-s",
        type=float,
        default=0.1,
        help="Initial zero-reference duration in seconds. Default: 0.1",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.8,
        help="Step hold duration in seconds. Default: 0.8",
    )
    parser.add_argument(
        "--post-s",
        type=float,
        default=0.6,
        help="Return-to-zero duration in seconds. Default: 0.6",
    )
    parser.add_argument(
        "--integral-limit",
        type=float,
        default=3000.0,
        help="Firmware integral_limit. Default: 3000",
    )
    parser.add_argument(
        "--output-limit",
        type=float,
        default=10000.0,
        help="Firmware output_limit. Default: 10000",
    )
    parser.add_argument(
        "--population",
        type=int,
        default=12,
        help="Differential evolution population size multiplier. Default: 12",
    )
    parser.add_argument(
        "--maxiter",
        type=int,
        default=40,
        help="Differential evolution max iterations. Default: 40",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible tuning. Default: 42",
    )
    parser.add_argument(
        "--styles",
        nargs="+",
        choices=["conservative", "balanced", "aggressive"],
        default=["conservative", "balanced", "aggressive"],
        help="Tuning styles to generate. Default: conservative balanced aggressive",
    )
    return parser.parse_args()


def load_summary(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fp:
        data = json.load(fp)

    if "models" not in data or not isinstance(data["models"], list):
        raise ValueError("summary.json missing models list")
    if "dt_s" not in data:
        raise ValueError("summary.json missing dt_s")
    return data


def select_model(summary: dict, model_name: str | None) -> tuple[str, dict]:
    target_name = model_name if model_name else summary.get("recommended_model")
    if not target_name:
        raise ValueError("no model selected and summary has no recommended_model")

    for model in summary["models"]:
        if model.get("name") == target_name:
            return target_name, model
    raise ValueError(f"model not found in summary: {target_name}")


def default_output_dir(summary_path: Path) -> Path:
    return summary_path.with_suffix("").parent / f"{summary_path.stem}_pid_tune"


def build_step_profile(dt: float, step_value: float, pre_s: float, hold_s: float, post_s: float) -> StepProfile:
    pre_n = max(1, int(round(pre_s / dt)))
    hold_n = max(1, int(round(hold_s / dt)))
    post_n = max(1, int(round(post_s / dt)))
    total_n = pre_n + hold_n + post_n

    t = np.arange(total_n, dtype=np.float64) * dt
    ref = np.zeros(total_n, dtype=np.float64)
    ref[pre_n : pre_n + hold_n] = step_value
    return StepProfile(
        t=t,
        ref=ref,
        step_start_idx=pre_n,
        step_end_idx=pre_n + hold_n,
        release_end_idx=total_n,
        step_value=step_value,
    )


class FirstOrderPlant:
    def __init__(self, gain: float, tau_s: float, bias: float, dt: float, delay_samples: int) -> None:
        self.gain = float(gain)
        self.tau_s = max(float(tau_s), 1e-6)
        self.bias = float(bias)
        self.dt = float(dt)
        self.delay_samples = max(int(delay_samples), 0)
        self.a = math.exp(-self.dt / self.tau_s)
        self.b = self.gain * (1.0 - self.a)
        self.u_hist = [0.0] * (self.delay_samples + 1)
        self.x = 0.0

    def output(self) -> float:
        return self.x + self.bias

    def step(self, u_cmd: float) -> float:
        self.u_hist.append(float(u_cmd))
        u_delayed = self.u_hist.pop(0)
        self.x = (self.a * self.x) + (self.b * u_delayed)
        return self.output()


class SecondOrderPlant:
    def __init__(self, gain: float, tau1_s: float, tau2_s: float, bias: float, dt: float, delay_samples: int) -> None:
        self.gain = float(gain)
        self.tau1_s = max(float(tau1_s), 1e-6)
        self.tau2_s = max(float(tau2_s), 1e-6)
        self.bias = float(bias)
        self.dt = float(dt)
        self.delay_samples = max(int(delay_samples), 0)
        self.a1 = math.exp(-self.dt / self.tau1_s)
        self.a2 = math.exp(-self.dt / self.tau2_s)
        self.u_hist = [0.0] * (self.delay_samples + 1)
        self.x1 = 0.0
        self.x2 = 0.0

    def output(self) -> float:
        return (self.gain * self.x2) + self.bias

    def step(self, u_cmd: float) -> float:
        self.u_hist.append(float(u_cmd))
        u_delayed = self.u_hist.pop(0)
        self.x1 = (self.a1 * self.x1) + ((1.0 - self.a1) * u_delayed)
        self.x2 = (self.a2 * self.x2) + ((1.0 - self.a2) * self.x1)
        return self.output()


def create_plant(model_name: str, params: dict, dt: float):
    delay_samples = int(round(float(params.get("delay_samples", 0.0))))
    if model_name in {"first_order", "first_order_delay"}:
        return FirstOrderPlant(
            gain=float(params["gain"]),
            tau_s=float(params["tau_s"]),
            bias=float(params.get("bias", 0.0)),
            dt=dt,
            delay_samples=delay_samples,
        )
    if model_name == "second_order_delay":
        return SecondOrderPlant(
            gain=float(params["gain"]),
            tau1_s=float(params["tau1_s"]),
            tau2_s=float(params["tau2_s"]),
            bias=float(params.get("bias", 0.0)),
            dt=dt,
            delay_samples=delay_samples,
        )
    raise ValueError(f"unsupported model for PID tuning: {model_name}")


def simulate_closed_loop(
    model_name: str,
    params: dict,
    profile: StepProfile,
    dt: float,
    kp: float,
    ki: float,
    kd: float,
    integral_limit: float,
    output_limit: float,
) -> tuple[np.ndarray, np.ndarray]:
    plant = create_plant(model_name, params, dt)
    n = len(profile.t)
    y = np.zeros(n, dtype=np.float64)
    u = np.zeros(n, dtype=np.float64)
    y[0] = plant.output()

    integral = 0.0
    previous_error = 0.0

    for k in range(n - 1):
        error = profile.ref[k] - y[k]
        derivative = (error - previous_error) / dt if dt > 0.0 else 0.0
        integral += error * dt
        integral = float(np.clip(integral, -integral_limit, integral_limit))
        ctrl = (kp * error) + (ki * integral) + (kd * derivative)
        ctrl = float(np.clip(ctrl, -output_limit, output_limit))
        u[k] = ctrl
        previous_error = error
        y[k + 1] = plant.step(ctrl)

    if n >= 2:
        u[-1] = u[-2]
    return y, u


def compute_step_metrics(profile: StepProfile, y: np.ndarray, u: np.ndarray, output_limit: float) -> dict[str, float]:
    step_slice = slice(profile.step_start_idx, profile.step_end_idx)
    release_slice = slice(profile.step_end_idx, profile.release_end_idx)
    step_ref = profile.step_value
    dt = profile.t[1] - profile.t[0] if len(profile.t) > 1 else 0.0

    y_step = y[step_slice]
    y_release = y[release_slice]
    ref_step = profile.ref[step_slice]
    err = profile.ref - y

    if abs(step_ref) > 1e-9:
        overshoot_pct = max(0.0, (np.max(y_step) - step_ref) / step_ref * 100.0)
        target_10 = 0.1 * step_ref
        target_90 = 0.9 * step_ref
        rise_time_s = math.nan
        idx10 = np.where(y_step >= target_10)[0]
        idx90 = np.where(y_step >= target_90)[0]
        if idx10.size and idx90.size and idx90[0] >= idx10[0]:
            rise_time_s = float((idx90[0] - idx10[0]) * dt)
        tolerance = 0.02 * abs(step_ref)
        settle_time_s = math.nan
        for idx in range(len(y_step)):
            if np.all(np.abs(y_step[idx:] - step_ref) <= tolerance):
                settle_time_s = float(idx * dt)
                break
        steady_state_error = float(abs(np.mean(y_step[-max(5, len(y_step) // 10):]) - step_ref))
    else:
        overshoot_pct = 0.0
        rise_time_s = math.nan
        settle_time_s = math.nan
        steady_state_error = 0.0

    iae = float(np.sum(np.abs(err)) * dt)
    itae = float(np.sum(profile.t * np.abs(err)) * dt)
    release_rms = float(np.sqrt(np.mean(y_release * y_release))) if len(y_release) else 0.0
    sat_ratio = float(np.mean(np.abs(u) >= (0.999 * output_limit)))
    control_variation = float(np.mean(np.abs(np.diff(u)))) if len(u) > 1 else 0.0

    return {
        "overshoot_pct": overshoot_pct,
        "rise_time_s": rise_time_s,
        "settle_time_s": settle_time_s,
        "steady_state_error": steady_state_error,
        "iae": iae,
        "itae": itae,
        "release_rms": release_rms,
        "sat_ratio": sat_ratio,
        "control_variation": control_variation,
    }


def style_cost_weights(style: str) -> dict[str, float]:
    if style == "conservative":
        return {
            "iae": 3.0,
            "itae": 2.0,
            "overshoot": 8.0,
            "steady": 3.0,
            "release": 3.5,
            "settle": 1.2,
            "rise": 0.2,
            "sat": 3.5,
            "control": 0.4,
            "ki_penalty": 0.10,
            "kd_reward": -0.01,
        }
    if style == "aggressive":
        return {
            "iae": 3.0,
            "itae": 1.2,
            "overshoot": 2.5,
            "steady": 2.0,
            "release": 1.2,
            "settle": 1.0,
            "rise": 1.3,
            "sat": 0.8,
            "control": 0.05,
            "ki_penalty": 0.01,
            "kd_reward": -0.005,
        }
    return {
        "iae": 3.0,
        "itae": 1.5,
        "overshoot": 4.0,
        "steady": 2.5,
        "release": 2.0,
        "settle": 1.5,
        "rise": 0.5,
        "sat": 1.5,
        "control": 0.1,
        "ki_penalty": 0.04,
        "kd_reward": -0.008,
    }


def tuning_cost(
    metrics: dict[str, float],
    profile: StepProfile,
    output_limit: float,
    style: str,
    kp: float,
    ki: float,
    kd: float,
) -> float:
    step_mag = max(abs(profile.step_value), 1e-6)
    total_time = max(float(profile.t[-1]), 1e-6)
    hold_time = max((profile.step_end_idx - profile.step_start_idx) * (profile.t[1] - profile.t[0]), 1e-6)
    weights = style_cost_weights(style)

    iae_norm = metrics["iae"] / (step_mag * total_time)
    itae_norm = metrics["itae"] / (step_mag * total_time * total_time)
    overshoot_norm = metrics["overshoot_pct"] / 100.0
    steady_norm = metrics["steady_state_error"] / step_mag
    release_norm = metrics["release_rms"] / step_mag
    sat_norm = metrics["sat_ratio"]
    control_norm = metrics["control_variation"] / max(output_limit, 1.0)

    settle = metrics["settle_time_s"]
    if math.isnan(settle):
        settle_norm = 2.0
    else:
        settle_norm = settle / hold_time

    rise = metrics["rise_time_s"]
    if math.isnan(rise):
        rise_norm = 2.0
    else:
        rise_norm = rise / hold_time

    return (
        (weights["iae"] * iae_norm)
        + (weights["itae"] * itae_norm)
        + (weights["overshoot"] * overshoot_norm)
        + (weights["steady"] * steady_norm)
        + (weights["release"] * release_norm)
        + (weights["settle"] * settle_norm)
        + (weights["rise"] * rise_norm)
        + (weights["sat"] * sat_norm)
        + (weights["control"] * control_norm)
        + (weights["ki_penalty"] * (ki / max(output_limit, 1.0)))
        + (weights["kd_reward"] * (kd / max(output_limit, 1.0)))
    )


def derive_pid_bounds(params: dict, step_value: float, style: str) -> list[tuple[float, float]]:
    gain = abs(float(params.get("gain", 0.001)))
    gain = max(gain, 1e-5)
    base_kp = max(20.0, min(1500.0, 1.2 / gain))
    if style == "conservative":
        kp_scale, ki_scale, kd_scale = 4.5, 1.2, 0.8
    elif style == "aggressive":
        kp_scale, ki_scale, kd_scale = 10.0, 8.0, 1.5
    else:
        kp_scale, ki_scale, kd_scale = 8.0, 5.0, 1.0
    kp_max = max(120.0, min(3000.0, kp_scale * base_kp))
    ki_max = max(5.0, min(3000.0, ki_scale * base_kp))
    kd_max = max(0.02, min(80.0, kd_scale * 0.03 * base_kp * max(step_value, 1.0)))
    return [
        (0.0, kp_max),
        (0.0, ki_max),
        (0.0, kd_max),
    ]


def tune_pid(
    style: str,
    model_name: str,
    params: dict,
    profile: StepProfile,
    dt: float,
    integral_limit: float,
    output_limit: float,
    population: int,
    maxiter: int,
    seed: int,
) -> TuneResult:
    bounds = derive_pid_bounds(params, profile.step_value, style)

    def objective(theta: np.ndarray) -> float:
        kp, ki, kd = [float(val) for val in theta]
        y, u = simulate_closed_loop(
            model_name=model_name,
            params=params,
            profile=profile,
            dt=dt,
            kp=kp,
            ki=ki,
            kd=kd,
            integral_limit=integral_limit,
            output_limit=output_limit,
        )
        if np.any(~np.isfinite(y)) or np.any(~np.isfinite(u)):
            return 1e9
        if np.max(np.abs(y)) > 10.0 * max(abs(profile.step_value), 1.0):
            return 1e9
        metrics = compute_step_metrics(profile, y, u, output_limit)
        return tuning_cost(metrics, profile, output_limit, style, kp, ki, kd)

    result = optimize.differential_evolution(
        objective,
        bounds=bounds,
        strategy="best1bin",
        maxiter=maxiter,
        popsize=population,
        tol=0.02,
        mutation=(0.5, 1.0),
        recombination=0.7,
        polish=True,
        seed=seed,
        updating="deferred",
        workers=1,
    )

    kp, ki, kd = [float(val) for val in result.x]
    y, u = simulate_closed_loop(
        model_name=model_name,
        params=params,
        profile=profile,
        dt=dt,
        kp=kp,
        ki=ki,
        kd=kd,
        integral_limit=integral_limit,
        output_limit=output_limit,
    )
    metrics = compute_step_metrics(profile, y, u, output_limit)
    return TuneResult(
        style=style,
        kp=kp,
        ki=ki,
        kd=kd,
        cost=float(result.fun),
        metrics=metrics,
        t=profile.t,
        ref=profile.ref,
        y=y,
        u=u,
    )


def save_summary(
    output_dir: Path,
    source_summary: Path,
    model_name: str,
    model_params: dict,
    results: list[TuneResult],
    recommended_style: str,
    integral_limit: float,
    output_limit: float,
) -> None:
    recommended = next(result for result in results if result.style == recommended_style)
    payload = {
        "source_summary": str(source_summary),
        "model_name": model_name,
        "model_params": model_params,
        "recommended_style": recommended_style,
        "recommended_pid": {
            "kp": recommended.kp,
            "ki": recommended.ki,
            "kd": recommended.kd,
            "integral_limit": integral_limit,
            "output_limit": output_limit,
        },
        "candidates": {
            result.style: {
                "kp": result.kp,
                "ki": result.ki,
                "kd": result.kd,
                "cost": result.cost,
                "metrics": result.metrics,
            }
            for result in results
        },
    }
    (output_dir / "pid_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def plot_tuning_result(output_dir: Path, results: list[TuneResult]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ax_y, ax_u = axes

    base = results[0]
    ax_y.plot(base.t, base.ref, label="reference", color="black", linewidth=1.2)
    for result in results:
        ax_y.plot(result.t, result.y, label=result.style, linewidth=1.2)
    ax_y.set_ylabel("speed (rad/s)")
    ax_y.grid(True, alpha=0.3)
    ax_y.legend(loc="best")

    for result in results:
        ax_u.plot(result.t, result.u, label=result.style, linewidth=1.0)
    ax_u.set_xlabel("time (s)")
    ax_u.set_ylabel("u_raw")
    ax_u.grid(True, alpha=0.3)
    ax_u.legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_dir / "pid_step_response.png", dpi=150)
    plt.close(fig)


def print_result(model_name: str, results: list[TuneResult], recommended_style: str) -> None:
    print(f"model: {model_name}")
    print(f"recommended style: {recommended_style}")
    print("candidate PID sets:")
    for result in results:
        marker = "*" if result.style == recommended_style else "-"
        print(f"{marker} {result.style}:")
        print(f"    kp = {result.kp:.6f}")
        print(f"    ki = {result.ki:.6f}")
        print(f"    kd = {result.kd:.6f}")
        print(f"    cost = {result.cost:.6f}")
        for key, value in result.metrics.items():
            print(f"    {key} = {value:.6f}")


def main() -> int:
    args = parse_args()

    try:
        summary = load_summary(args.summary)
        model_name, model = select_model(summary, args.model)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    output_dir = args.output_dir if args.output_dir is not None else default_output_dir(args.summary)
    output_dir.mkdir(parents=True, exist_ok=True)

    dt = float(summary["dt_s"])
    profile = build_step_profile(
        dt=dt,
        step_value=float(args.step_radps),
        pre_s=float(args.pre_s),
        hold_s=float(args.hold_s),
        post_s=float(args.post_s),
    )

    results: list[TuneResult] = []
    for index, style in enumerate(args.styles):
        try:
            results.append(
                tune_pid(
                    style=style,
                    model_name=model_name,
                    params=model["params"],
                    profile=profile,
                    dt=dt,
                    integral_limit=float(args.integral_limit),
                    output_limit=float(args.output_limit),
                    population=int(args.population),
                    maxiter=int(args.maxiter),
                    seed=int(args.seed) + index,
                )
            )
        except Exception as exc:
            print(f"error: PID tuning failed for style {style}: {exc}", file=sys.stderr)
            return 1

    recommended_style = "conservative" if any(r.style == "conservative" for r in results) else min(results, key=lambda item: item.cost).style

    save_summary(
        output_dir=output_dir,
        source_summary=args.summary,
        model_name=model_name,
        model_params=model["params"],
        results=results,
        recommended_style=recommended_style,
        integral_limit=float(args.integral_limit),
        output_limit=float(args.output_limit),
    )
    plot_tuning_result(output_dir, results)
    print_result(model_name, results, recommended_style)
    print()
    print(f"wrote {output_dir / 'pid_summary.json'}")
    print(f"wrote {output_dir / 'pid_step_response.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
