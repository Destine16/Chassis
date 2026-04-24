#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from scipy import signal

from nav_cdc_monitor import (
    IMU_FAULT_MASK,
    IMU_VALID_MASK,
    WHEEL_FL_VALID_MASK,
    WHEEL_FR_VALID_MASK,
    WHEEL_RL_VALID_MASK,
    WHEEL_RR_VALID_MASK,
    autodetect_port,
    read_frames,
)


WHEEL_RADIUS_M = 0.0815
OMNI_45DEG_COEF = 0.70710678118
LEVEL_JSON_GLOB = "imu_static_level_*.json"


@dataclass
class YawSample:
    host_t_s: float
    seq: int
    t_ms: int
    w_fl: float
    w_fr: float
    w_rl: float
    w_rr: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    flags: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect USB CDC wheel+IMU data and estimate yaw extrinsic from wheel odometry acceleration."
    )
    parser.add_argument("-p", "--port", help="Serial port path, e.g. /dev/ttyACM0")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Collection duration after warmup, seconds (0 = manual Ctrl+C stop)")
    parser.add_argument("--warmup", type=float, default=1.0, help="Initial samples to ignore, seconds")
    parser.add_argument("--no-wait-start", action="store_true", help="Start immediately without waiting for Enter")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout, seconds")
    parser.add_argument("--output-dir", type=Path, default=Path("data/imu_calibration"), help="Output directory")
    parser.add_argument(
        "--level-json",
        default="auto",
        help="Static level calibration JSON. Use 'auto' to choose the strongest static level result, or 'none'.",
    )
    parser.add_argument("--lowpass-hz", type=float, default=3.0, help="Low-pass cutoff for velocity/acceleration")
    parser.add_argument("--min-imu-acc", type=float, default=0.12, help="Minimum IMU horizontal acceleration norm")
    parser.add_argument("--min-odom-acc", type=float, default=0.12, help="Minimum odometry acceleration norm")
    parser.add_argument("--max-gyro-z", type=float, default=0.7, help="Reject samples with large yaw rate, rad/s")
    parser.add_argument("--max-speed", type=float, default=2.5, help="Reject samples with unrealistic odom speed, m/s")
    parser.add_argument("--bias-speed-threshold", type=float, default=0.04,
                        help="Use samples below this odom speed to estimate IMU horizontal accel bias")
    parser.add_argument("--residual-threshold-deg", type=float, default=35.0,
                        help="Robust second-pass residual threshold")
    parser.add_argument("--min-fit-samples", type=int, default=120, help="Minimum samples for yaw fit")
    parser.add_argument("--print-every", type=int, default=300, help="Progress print interval in accepted samples")
    parser.add_argument("--no-plot", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def load_level_matrix(level_json: str, output_dir: Path) -> tuple[np.ndarray, str]:
    if level_json == "none":
        return np.eye(3), "none"
    if level_json != "auto":
        path = Path(level_json)
        data = json.loads(path.read_text(encoding="utf-8"))
        return np.asarray(data["rotation_matrix_chassis_level"], dtype=float), str(path)

    candidates = sorted(output_dir.glob(LEVEL_JSON_GLOB))
    if not candidates:
        print("[warn] no static level calibration JSON found, using identity", file=sys.stderr)
        return np.eye(3), "identity-no-json-found"

    scored: list[tuple[float, Path]] = []
    for path in candidates:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            score = float(data.get("tilt_before_deg", 0.0))
            if "rotation_matrix_chassis_level" in data:
                scored.append((score, path))
        except (OSError, ValueError, TypeError, KeyError):
            continue

    if not scored:
        print("[warn] no usable static level calibration JSON found, using identity", file=sys.stderr)
        return np.eye(3), "identity-no-usable-json"

    # The latest file can be a post-application validation run. For current firmware, which sends the
    # uncorrected base chassis vector, prefer the strongest raw static tilt calibration.
    _, path = max(scored, key=lambda item: item[0])
    data = json.loads(path.read_text(encoding="utf-8"))
    return np.asarray(data["rotation_matrix_chassis_level"], dtype=float), str(path)


def collect_samples(args: argparse.Namespace) -> tuple[str, list[YawSample], dict[str, int]]:
    port = args.port or autodetect_port()
    counters = {
        "parsed_frames": 0,
        "accepted": 0,
        "rejected_imu": 0,
        "rejected_wheel": 0,
    }
    rows: list[YawSample] = []

    print(f"[info] opening {port}", file=sys.stderr)
    with serial.Serial(port=port, baudrate=115200, timeout=args.timeout) as ser:
        if not args.no_wait_start:
            input("Press Enter to start yaw calibration collection, then move the chassis. Press Ctrl+C to stop...")

        start = time.monotonic()
        collect_start = start + max(args.warmup, 0.0)
        manual_stop = args.duration <= 0.0
        end = float("inf") if manual_stop else collect_start + args.duration

        if manual_stop:
            print("[info] collecting until Ctrl+C", file=sys.stderr)
        else:
            print(f"[info] collecting for {args.duration:.1f}s after warmup", file=sys.stderr)

        try:
            for obs in read_frames(ser):
                now = time.monotonic()
                if now < collect_start:
                    continue
                if now >= end:
                    break

                counters["parsed_frames"] += 1
                if ((obs.flags & IMU_VALID_MASK) == 0) or ((obs.flags & IMU_FAULT_MASK) != 0):
                    counters["rejected_imu"] += 1
                    continue
                wheel_mask = WHEEL_FL_VALID_MASK | WHEEL_FR_VALID_MASK | WHEEL_RL_VALID_MASK | WHEEL_RR_VALID_MASK
                if (obs.flags & wheel_mask) != wheel_mask:
                    counters["rejected_wheel"] += 1
                    continue

                rows.append(YawSample(
                    host_t_s=now - collect_start,
                    seq=obs.seq,
                    t_ms=obs.t_ms,
                    w_fl=obs.w_fl,
                    w_fr=obs.w_fr,
                    w_rl=obs.w_rl,
                    w_rr=obs.w_rr,
                    gyro_x=obs.gyro_x,
                    gyro_y=obs.gyro_y,
                    gyro_z=obs.gyro_z,
                    acc_x=obs.acc_x,
                    acc_y=obs.acc_y,
                    acc_z=obs.acc_z,
                    flags=obs.flags,
                ))
                counters["accepted"] += 1
                if args.print_every > 0 and counters["accepted"] % args.print_every == 0:
                    print(f"[info] accepted={counters['accepted']}", file=sys.stderr)
        except KeyboardInterrupt:
            print(f"\n[info] stopped by user, accepted={counters['accepted']}", file=sys.stderr)

    return port, rows, counters


def wheel_speeds_to_chassis_velocity(w: np.ndarray) -> np.ndarray:
    coeff = WHEEL_RADIUS_M / (2.0 * OMNI_45DEG_COEF)
    vx = coeff * (w[:, 0] + w[:, 1] + w[:, 2] + w[:, 3]) * 0.5
    vy = coeff * (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3]) * 0.5
    return np.column_stack((vx, vy))


def lowpass(data: np.ndarray, fs_hz: float, cutoff_hz: float) -> np.ndarray:
    if cutoff_hz <= 0.0 or cutoff_hz >= 0.45 * fs_hz or len(data) < 12:
        return data.copy()
    sos = signal.butter(2, cutoff_hz, btype="lowpass", fs=fs_hz, output="sos")
    return signal.sosfiltfilt(sos, data, axis=0)


def fit_yaw(a_imu_xy: np.ndarray, a_odom_xy: np.ndarray, weights: np.ndarray) -> float:
    x1 = a_imu_xy[:, 0]
    y1 = a_imu_xy[:, 1]
    x2 = a_odom_xy[:, 0]
    y2 = a_odom_xy[:, 1]
    cross_sum = np.sum(weights * (x1 * y2 - y1 * x2))
    dot_sum = np.sum(weights * (x1 * x2 + y1 * y2))
    return float(math.atan2(cross_sum, dot_sum))


def rotate_xy(v: np.ndarray, theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.column_stack((c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1]))


def angle_residual_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    dot = a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]
    return np.degrees(np.arctan2(cross, dot))


def cosine_similarity(a: np.ndarray, b: np.ndarray, weights: np.ndarray) -> float:
    denom = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    valid = denom > 1e-9
    if not np.any(valid):
        return float("nan")
    cos = np.clip(np.sum(a[valid] * b[valid], axis=1) / denom[valid], -1.0, 1.0)
    w = weights[valid]
    return float(np.sum(w * cos) / max(np.sum(w), 1e-12))


def analyze_samples(rows: list[YawSample], level_R: np.ndarray, args: argparse.Namespace) -> tuple[dict[str, object], list[dict[str, object]]]:
    if len(rows) < args.min_fit_samples:
        raise ValueError(f"not enough accepted frames: {len(rows)}")

    t_ms = np.asarray([row.t_ms for row in rows], dtype=float)
    t = (t_ms - t_ms[0]) / 1000.0
    dt_samples = np.diff(t)
    dt_good = dt_samples[(dt_samples > 0.0) & np.isfinite(dt_samples)]
    if len(dt_good) == 0:
        raise ValueError("invalid timestamps")
    dt = float(np.median(dt_good))
    fs = 1.0 / dt

    w = np.asarray([[row.w_fl, row.w_fr, row.w_rl, row.w_rr] for row in rows], dtype=float)
    acc = np.asarray([[row.acc_x, row.acc_y, row.acc_z] for row in rows], dtype=float)
    gyro = np.asarray([[row.gyro_x, row.gyro_y, row.gyro_z] for row in rows], dtype=float)

    acc_level = (level_R @ acc.T).T
    gyro_level = (level_R @ gyro.T).T
    v_odom = wheel_speeds_to_chassis_velocity(w)
    speed_norm = np.linalg.norm(v_odom, axis=1)

    static_mask = speed_norm < args.bias_speed_threshold
    if np.count_nonzero(static_mask) >= 30:
        acc_bias_xy = np.median(acc_level[static_mask, :2], axis=0)
    else:
        acc_bias_xy = np.median(acc_level[:, :2], axis=0)

    v_f = lowpass(v_odom, fs, args.lowpass_hz)
    a_odom = np.gradient(v_f, dt, axis=0)
    a_odom = lowpass(a_odom, fs, args.lowpass_hz)
    a_imu = acc_level[:, :2] - acc_bias_xy
    a_imu = lowpass(a_imu, fs, args.lowpass_hz)

    imu_norm = np.linalg.norm(a_imu, axis=1)
    odom_norm = np.linalg.norm(a_odom, axis=1)
    valid = (
        (imu_norm >= args.min_imu_acc) &
        (odom_norm >= args.min_odom_acc) &
        (speed_norm <= args.max_speed) &
        (np.abs(gyro_level[:, 2]) <= args.max_gyro_z)
    )

    if np.count_nonzero(valid) < args.min_fit_samples:
        raise ValueError(
            f"not enough excitation samples: {np.count_nonzero(valid)} < {args.min_fit_samples}; "
            "move with clearer acceleration/deceleration segments"
        )

    fit_idx = np.flatnonzero(valid)
    weights = np.sqrt(imu_norm[fit_idx] * odom_norm[fit_idx])
    weights = weights / max(float(np.max(weights)), 1e-12)
    yaw_first = fit_yaw(a_imu[fit_idx], a_odom[fit_idx], weights)
    residual_first = angle_residual_deg(rotate_xy(a_imu[fit_idx], yaw_first), a_odom[fit_idx])
    robust = np.abs(residual_first) <= args.residual_threshold_deg
    fit_idx2 = fit_idx[robust]

    if len(fit_idx2) < args.min_fit_samples:
        fit_idx2 = fit_idx
        robust = np.ones_like(fit_idx, dtype=bool)

    weights2 = np.sqrt(imu_norm[fit_idx2] * odom_norm[fit_idx2])
    weights2 = weights2 / max(float(np.max(weights2)), 1e-12)
    yaw = fit_yaw(a_imu[fit_idx2], a_odom[fit_idx2], weights2)
    a_imu_rot = rotate_xy(a_imu, yaw)
    residual = angle_residual_deg(a_imu_rot[fit_idx2], a_odom[fit_idx2])
    residual_before = angle_residual_deg(a_imu[fit_idx2], a_odom[fit_idx2])

    c = math.cos(yaw)
    s = math.sin(yaw)
    R_yaw = np.asarray([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])
    R_total = R_yaw @ level_R

    result = {
        "sample_count_total": int(len(rows)),
        "fit_sample_count_initial": int(len(fit_idx)),
        "fit_sample_count_used": int(len(fit_idx2)),
        "outlier_count": int(len(fit_idx) - len(fit_idx2)),
        "sample_period_s": dt,
        "sample_rate_hz": fs,
        "lowpass_hz": args.lowpass_hz,
        "acc_bias_xy_mps2": [float(v) for v in acc_bias_xy],
        "yaw_correction_rad": yaw,
        "yaw_correction_deg": math.degrees(yaw),
        "yaw_first_pass_deg": math.degrees(yaw_first),
        "residual_mean_deg": float(np.mean(residual)),
        "residual_mae_deg": float(np.mean(np.abs(residual))),
        "residual_rmse_deg": float(math.sqrt(np.mean(residual * residual))),
        "residual_before_mae_deg": float(np.mean(np.abs(residual_before))),
        "residual_before_rmse_deg": float(math.sqrt(np.mean(residual_before * residual_before))),
        "cosine_before": cosine_similarity(a_imu[fit_idx2], a_odom[fit_idx2], weights2),
        "cosine_after": cosine_similarity(a_imu_rot[fit_idx2], a_odom[fit_idx2], weights2),
        "mean_imu_acc_norm_used_mps2": float(np.mean(imu_norm[fit_idx2])),
        "mean_odom_acc_norm_used_mps2": float(np.mean(odom_norm[fit_idx2])),
        "rotation_matrix_yaw": R_yaw.tolist(),
        "rotation_matrix_total_yaw_level": R_total.tolist(),
        "apply_note": "imu_final = rotation_matrix_yaw * rotation_matrix_chassis_level * imu_from_elec",
    }

    export_rows: list[dict[str, object]] = []
    used_set = set(int(i) for i in fit_idx2)
    initial_set = set(int(i) for i in fit_idx)
    for i, row in enumerate(rows):
        export_rows.append({
            **asdict(row),
            "acc_level_x": float(acc_level[i, 0]),
            "acc_level_y": float(acc_level[i, 1]),
            "acc_level_z": float(acc_level[i, 2]),
            "gyro_level_x": float(gyro_level[i, 0]),
            "gyro_level_y": float(gyro_level[i, 1]),
            "gyro_level_z": float(gyro_level[i, 2]),
            "vx_odom_mps": float(v_odom[i, 0]),
            "vy_odom_mps": float(v_odom[i, 1]),
            "ax_odom_mps2": float(a_odom[i, 0]),
            "ay_odom_mps2": float(a_odom[i, 1]),
            "ax_imu_mps2": float(a_imu[i, 0]),
            "ay_imu_mps2": float(a_imu[i, 1]),
            "ax_imu_yaw_mps2": float(a_imu_rot[i, 0]),
            "ay_imu_yaw_mps2": float(a_imu_rot[i, 1]),
            "fit_candidate": int(i in initial_set),
            "used_for_fit": int(i in used_set),
        })

    return result, export_rows


def save_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_plot(path: Path, rows: list[dict[str, object]], result: dict[str, object]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.asarray([float(row["host_t_s"]) for row in rows])
    imu = np.asarray([[float(row["ax_imu_mps2"]), float(row["ay_imu_mps2"])] for row in rows])
    imu_yaw = np.asarray([[float(row["ax_imu_yaw_mps2"]), float(row["ay_imu_yaw_mps2"])] for row in rows])
    odom = np.asarray([[float(row["ax_odom_mps2"]), float(row["ay_odom_mps2"])] for row in rows])
    used = np.asarray([bool(row["used_for_fit"]) for row in rows])

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes[0, 0].plot(t, odom[:, 0], label="odom ax")
    axes[0, 0].plot(t, imu_yaw[:, 0], label="imu ax after yaw")
    axes[0, 0].set_ylabel("x accel (m/s^2)")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[1, 0].plot(t, odom[:, 1], label="odom ay")
    axes[1, 0].plot(t, imu_yaw[:, 1], label="imu ay after yaw")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("y accel (m/s^2)")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[0, 1].scatter(imu[used, 0], imu[used, 1], s=6, alpha=0.45, label="imu before yaw")
    axes[0, 1].scatter(odom[used, 0], odom[used, 1], s=6, alpha=0.45, label="odom")
    axes[0, 1].axis("equal")
    axes[0, 1].set_title("Before yaw correction")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 1].scatter(imu_yaw[used, 0], imu_yaw[used, 1], s=6, alpha=0.45, label="imu after yaw")
    axes[1, 1].scatter(odom[used, 0], odom[used, 1], s=6, alpha=0.45, label="odom")
    axes[1, 1].axis("equal")
    axes[1, 1].set_title("After yaw correction")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle(
        "Yaw calibration: "
        f"yaw={result['yaw_correction_deg']:.3f} deg, "
        f"residual RMSE={result['residual_rmse_deg']:.2f} deg"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def print_result(result: dict[str, object], csv_path: Path, json_path: Path, plot_path: Path | None) -> None:
    print("")
    print("IMU yaw calibration")
    print(f"  samples: total={result['sample_count_total']} initial_fit={result['fit_sample_count_initial']} "
          f"used={result['fit_sample_count_used']} outliers={result['outlier_count']}")
    print(f"  yaw_correction: {result['yaw_correction_rad']:.9f} rad ({result['yaw_correction_deg']:.6f} deg)")
    print(f"  residual RMSE: {result['residual_before_rmse_deg']:.3f} -> {result['residual_rmse_deg']:.3f} deg")
    print(f"  residual MAE: {result['residual_before_mae_deg']:.3f} -> {result['residual_mae_deg']:.3f} deg")
    print(f"  cosine: {result['cosine_before']:.6f} -> {result['cosine_after']:.6f}")
    print(f"  acc_bias_xy: [{result['acc_bias_xy_mps2'][0]:.6f}, {result['acc_bias_xy_mps2'][1]:.6f}] m/s^2")
    print("")
    print("Yaw matrix:")
    print_matrix(result["rotation_matrix_yaw"])
    print("")
    print("Total matrix for navigation, if applying both level and yaw:")
    print_matrix(result["rotation_matrix_total_yaw_level"])
    print("")
    print("Eigen initializer:")
    print("Eigen::Matrix3d R_total;")
    flat = result["rotation_matrix_total_yaw_level"]
    for i, row in enumerate(flat):
        prefix = "R_total << " if i == 0 else "           "
        suffix = "," if i < 2 else ";"
        print(prefix + ", ".join(f"{float(v): .9f}" for v in row) + suffix)
    print("")
    print(f"csv={csv_path}")
    print(f"json={json_path}")
    if plot_path is not None:
        print(f"plot={plot_path}")


def print_matrix(matrix: object) -> None:
    for row in matrix:
        print("    " + " ".join(f"{float(value): .9f}" for value in row))


def main() -> int:
    args = parse_args()
    level_R, level_source = load_level_matrix(args.level_json, args.output_dir)
    print(f"[info] level_matrix={level_source}", file=sys.stderr)
    port, samples, counters = collect_samples(args)
    if len(samples) < args.min_fit_samples:
        print(f"[error] only {len(samples)} accepted samples, counters={counters}", file=sys.stderr)
        return 2

    try:
        result, export_rows = analyze_samples(samples, level_R, args)
    except ValueError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 2

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / f"imu_yaw_{stamp}.csv"
    json_path = args.output_dir / f"imu_yaw_{stamp}.json"
    plot_path = None if args.no_plot else args.output_dir / f"imu_yaw_{stamp}.png"

    metadata = {
        "port": port,
        "duration_s": args.duration,
        "warmup_s": args.warmup,
        "level_matrix_source": level_source,
        "counters": counters,
    }
    output = dict(result)
    output["metadata"] = metadata

    save_csv(csv_path, export_rows)
    json_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    if plot_path is not None:
        save_plot(plot_path, export_rows, result)
    print_result(result, csv_path, json_path, plot_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
