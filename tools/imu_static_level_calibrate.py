#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

import serial

from nav_cdc_monitor import (
    FRAME_LEN,
    IMU_FAULT_MASK,
    IMU_VALID_MASK,
    PAYLOAD_LEN,
    SOF0,
    SOF1,
    autodetect_port,
    crc16_ccitt,
    parse_observation,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect static USB CDC IMU samples and compute roll/pitch level calibration."
    )
    parser.add_argument("-p", "--port", help="Serial port path, e.g. /dev/ttyACM0")
    parser.add_argument("--duration", type=float, default=8.0, help="Static collection duration after warmup, seconds")
    parser.add_argument("--warmup", type=float, default=1.0, help="Initial samples to ignore, seconds")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout, seconds")
    parser.add_argument("--output-dir", type=Path, default=Path("data/imu_calibration"), help="Output directory")
    parser.add_argument("--target-z", choices=("auto", "positive", "negative"), default="auto",
                        help="Expected gravity sign on chassis z after calibration")
    parser.add_argument("--gyro-static-threshold", type=float, default=0.08,
                        help="Reject samples with gyro norm above this value, rad/s")
    parser.add_argument("--acc-norm-min", type=float, default=7.0, help="Reject samples below this accel norm, m/s^2")
    parser.add_argument("--acc-norm-max", type=float, default=12.5, help="Reject samples above this accel norm, m/s^2")
    parser.add_argument("--mad-threshold", type=float, default=4.0,
                        help="Robust outlier rejection threshold in MAD units")
    parser.add_argument("--min-samples", type=int, default=200, help="Minimum accepted static samples")
    parser.add_argument("--print-every", type=int, default=200, help="Progress print interval in accepted samples")
    return parser.parse_args()


def vec_add(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [x + y for x, y in zip(a, b)]


def vec_sub(a: Iterable[float], b: Iterable[float]) -> list[float]:
    return [x - y for x, y in zip(a, b)]


def vec_scale(a: Iterable[float], scale: float) -> list[float]:
    return [x * scale for x in a]


def dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(a: Iterable[float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Iterable[float]) -> list[float]:
    value = list(a)
    n = norm(value)
    if n <= 1e-12:
        raise ValueError("cannot normalize near-zero vector")
    return vec_scale(value, 1.0 / n)


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def mat_vec_mul(a: list[list[float]], v: list[float]) -> list[float]:
    return [sum(a[row][col] * v[col] for col in range(3)) for row in range(3)]


def identity3() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def skew(v: list[float]) -> list[list[float]]:
    return [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]


def axis_angle_rotation(axis: list[float], angle: float) -> list[list[float]]:
    x, y, z = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def rotation_align_vectors(source: list[float], target: list[float]) -> list[list[float]]:
    source = normalize(source)
    target = normalize(target)
    v = cross(source, target)
    c = max(-1.0, min(1.0, dot(source, target)))
    s = norm(v)

    if s < 1e-9:
        if c > 0.0:
            return identity3()
        basis = [1.0, 0.0, 0.0] if abs(source[0]) < 0.9 else [0.0, 1.0, 0.0]
        axis = cross(source, basis)
        return axis_angle_rotation(axis, math.pi)

    k = skew(v)
    k2 = mat_mul(k, k)
    gain = (1.0 - c) / (s * s)
    r = identity3()
    for row in range(3):
        for col in range(3):
            r[row][col] += k[row][col] + k2[row][col] * gain
    return r


def euler_zyx_from_matrix(r: list[list[float]]) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    return roll, pitch, yaw


def mean_vec(values: list[list[float]]) -> list[float]:
    return [statistics.fmean(row[i] for row in values) for i in range(3)]


def std_vec(values: list[list[float]], mean: list[float]) -> list[float]:
    if len(values) < 2:
        return [0.0, 0.0, 0.0]
    return [
        math.sqrt(statistics.fmean((row[i] - mean[i]) ** 2 for row in values))
        for i in range(3)
    ]


def median_vec(values: list[list[float]]) -> list[float]:
    return [statistics.median(row[i] for row in values) for i in range(3)]


def read_observations_once(ser: serial.Serial, buf: bytearray):
    chunk = ser.read(ser.in_waiting or 1)
    if not chunk:
        return []

    observations = []
    buf.extend(chunk)
    while True:
        if len(buf) < FRAME_LEN:
            break

        sof = buf.find(bytes((SOF0, SOF1)))
        if sof < 0:
            del buf[:-1]
            break

        if sof > 0:
            del buf[:sof]

        if len(buf) < FRAME_LEN:
            break

        frame = bytes(buf[:FRAME_LEN])
        if frame[3] != PAYLOAD_LEN:
            del buf[0]
            continue

        expected_crc = int.from_bytes(frame[-2:], "little")
        actual_crc = crc16_ccitt(frame[:-2])
        if expected_crc != actual_crc:
            del buf[0]
            continue

        del buf[:FRAME_LEN]
        observations.append(parse_observation(frame))

    return observations


def collect_static_samples(args: argparse.Namespace) -> tuple[str, list[dict[str, float | int]], dict[str, int]]:
    port = args.port or autodetect_port()
    counters = {
        "parsed_frames": 0,
        "accepted": 0,
        "rejected_imu_invalid": 0,
        "rejected_imu_fault": 0,
        "rejected_motion": 0,
        "rejected_acc_norm": 0,
    }
    rows: list[dict[str, float | int]] = []
    buf = bytearray()

    print(f"[info] opening {port}", file=sys.stderr)
    with serial.Serial(port=port, baudrate=115200, timeout=args.timeout) as ser:
        start = time.monotonic()
        collect_start = start + max(args.warmup, 0.0)
        end = collect_start + max(args.duration, 0.0)

        while time.monotonic() < end:
            now = time.monotonic()
            observations = read_observations_once(ser, buf)
            if now < collect_start:
                continue

            for obs in observations:
                counters["parsed_frames"] += 1

                if (obs.flags & IMU_VALID_MASK) == 0:
                    counters["rejected_imu_invalid"] += 1
                    continue
                if (obs.flags & IMU_FAULT_MASK) != 0:
                    counters["rejected_imu_fault"] += 1
                    continue

                gyro = [obs.gyro_x, obs.gyro_y, obs.gyro_z]
                acc = [obs.acc_x, obs.acc_y, obs.acc_z]
                gyro_norm = norm(gyro)
                acc_norm = norm(acc)

                if gyro_norm > args.gyro_static_threshold:
                    counters["rejected_motion"] += 1
                    continue
                if not (args.acc_norm_min <= acc_norm <= args.acc_norm_max):
                    counters["rejected_acc_norm"] += 1
                    continue

                rows.append({
                    "host_t_s": now - collect_start,
                    "seq": obs.seq,
                    "t_ms": obs.t_ms,
                    "gyro_x": obs.gyro_x,
                    "gyro_y": obs.gyro_y,
                    "gyro_z": obs.gyro_z,
                    "acc_x": obs.acc_x,
                    "acc_y": obs.acc_y,
                    "acc_z": obs.acc_z,
                    "gyro_norm": gyro_norm,
                    "acc_norm": acc_norm,
                    "flags": obs.flags,
                })
                counters["accepted"] += 1
                if args.print_every > 0 and counters["accepted"] % args.print_every == 0:
                    print(f"[info] accepted={counters['accepted']}", file=sys.stderr)

    return port, rows, counters


def compute_level_calibration(rows: list[dict[str, float | int]], target_z: str,
                              mad_threshold: float) -> dict[str, object]:
    acc_all = [[float(row["acc_x"]), float(row["acc_y"]), float(row["acc_z"])] for row in rows]
    gyro_all = [[float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])] for row in rows]
    acc_median = median_vec(acc_all)
    distances = [norm(vec_sub(acc, acc_median)) for acc in acc_all]
    dist_median = statistics.median(distances)
    mad = statistics.median(abs(d - dist_median) for d in distances)
    sigma = 1.4826 * mad

    if sigma <= 1e-9:
        keep = [True] * len(acc_all)
    else:
        keep = [d <= dist_median + mad_threshold * sigma for d in distances]

    acc_used = [acc for acc, used in zip(acc_all, keep) if used]
    gyro_used = [gyro for gyro, used in zip(gyro_all, keep) if used]
    mean_acc = mean_vec(acc_used)
    mean_gyro = mean_vec(gyro_used)
    std_acc = std_vec(acc_used, mean_acc)
    std_gyro = std_vec(gyro_used, mean_gyro)
    measured_g = norm(mean_acc)

    if target_z == "positive":
        z_sign = 1.0
    elif target_z == "negative":
        z_sign = -1.0
    else:
        z_sign = 1.0 if mean_acc[2] >= 0.0 else -1.0

    target = [0.0, 0.0, z_sign]
    r_level = rotation_align_vectors(mean_acc, target)
    corrected_mean_acc = mat_vec_mul(r_level, mean_acc)
    roll, pitch, yaw = euler_zyx_from_matrix(r_level)
    horizontal_before = math.sqrt(mean_acc[0] ** 2 + mean_acc[1] ** 2)
    horizontal_after = math.sqrt(corrected_mean_acc[0] ** 2 + corrected_mean_acc[1] ** 2)
    tilt_before = math.atan2(horizontal_before, abs(mean_acc[2]))
    tilt_after = math.atan2(horizontal_after, abs(corrected_mean_acc[2]))

    return {
        "sample_count_total": len(rows),
        "sample_count_used": len(acc_used),
        "outlier_count": len(rows) - len(acc_used),
        "target_acc_unit": target,
        "mean_acc_before_mps2": mean_acc,
        "mean_gyro_before_radps": mean_gyro,
        "std_acc_used_mps2": std_acc,
        "std_gyro_used_radps": std_gyro,
        "measured_gravity_mps2": measured_g,
        "horizontal_acc_before_mps2": horizontal_before,
        "horizontal_acc_after_mps2": horizontal_after,
        "tilt_before_rad": tilt_before,
        "tilt_before_deg": math.degrees(tilt_before),
        "tilt_after_rad": tilt_after,
        "tilt_after_deg": math.degrees(tilt_after),
        "roll_correction_rad": roll,
        "pitch_correction_rad": pitch,
        "yaw_correction_rad": yaw,
        "roll_correction_deg": math.degrees(roll),
        "pitch_correction_deg": math.degrees(pitch),
        "yaw_correction_deg": math.degrees(yaw),
        "rotation_matrix_chassis_level": r_level,
        "corrected_mean_acc_mps2": corrected_mean_acc,
        "apply_note": "corrected_vec = rotation_matrix_chassis_level * current_chassis_vec",
        "used_mask": keep,
    }


def save_outputs(output_dir: Path, rows: list[dict[str, float | int]], result: dict[str, object],
                 metadata: dict[str, object]) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"imu_static_level_{stamp}.csv"
    json_path = output_dir / f"imu_static_level_{stamp}.json"
    used_mask = result["used_mask"]

    fieldnames = [
        "host_t_s",
        "seq",
        "t_ms",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_norm",
        "acc_norm",
        "flags",
        "used_for_fit",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row, used in zip(rows, used_mask):
            out = dict(row)
            out["used_for_fit"] = int(bool(used))
            writer.writerow(out)

    serializable = dict(result)
    serializable.pop("used_mask", None)
    serializable["metadata"] = metadata
    json_path.write_text(json.dumps(serializable, indent=2), encoding="utf-8")
    return csv_path, json_path


def print_result(result: dict[str, object], csv_path: Path, json_path: Path) -> None:
    matrix = result["rotation_matrix_chassis_level"]
    print("")
    print("Static IMU level calibration")
    print(f"  samples: total={result['sample_count_total']} used={result['sample_count_used']} "
          f"outliers={result['outlier_count']}")
    print(f"  mean_acc_before: {format_vec(result['mean_acc_before_mps2'])} m/s^2")
    print(f"  corrected_mean_acc: {format_vec(result['corrected_mean_acc_mps2'])} m/s^2")
    print(f"  measured_gravity: {result['measured_gravity_mps2']:.6f} m/s^2")
    print(f"  horizontal_acc: {result['horizontal_acc_before_mps2']:.6f} -> "
          f"{result['horizontal_acc_after_mps2']:.6f} m/s^2")
    print(f"  tilt: {result['tilt_before_deg']:.6f} -> {result['tilt_after_deg']:.6f} deg")
    print(f"  roll_correction: {result['roll_correction_rad']:.9f} rad "
          f"({result['roll_correction_deg']:.6f} deg)")
    print(f"  pitch_correction: {result['pitch_correction_rad']:.9f} rad "
          f"({result['pitch_correction_deg']:.6f} deg)")
    print(f"  yaw_correction: {result['yaw_correction_rad']:.9f} rad "
          f"({result['yaw_correction_deg']:.6f} deg)")
    print("  rotation_matrix_chassis_level:")
    for row in matrix:
        print("    " + " ".join(f"{value: .9f}" for value in row))
    print("")
    print("C initializer:")
    print("static const float k_imu_level_R[3][3] = {")
    for row in matrix:
        print("    {" + ", ".join(f"{value:.9f}f" for value in row) + "},")
    print("};")
    print("")
    print(f"csv={csv_path}")
    print(f"json={json_path}")


def format_vec(value: object) -> str:
    return "[" + ", ".join(f"{float(v):.6f}" for v in value) + "]"


def main() -> int:
    args = parse_args()
    port, rows, counters = collect_static_samples(args)

    if len(rows) < args.min_samples:
        print(
            f"[error] only {len(rows)} accepted static samples, need at least {args.min_samples}; "
            f"counters={counters}",
            file=sys.stderr,
        )
        return 2

    result = compute_level_calibration(rows, args.target_z, args.mad_threshold)
    metadata = {
        "port": port,
        "duration_s": args.duration,
        "warmup_s": args.warmup,
        "gyro_static_threshold_radps": args.gyro_static_threshold,
        "acc_norm_min_mps2": args.acc_norm_min,
        "acc_norm_max_mps2": args.acc_norm_max,
        "mad_threshold": args.mad_threshold,
        "counters": counters,
    }
    csv_path, json_path = save_outputs(args.output_dir, rows, result, metadata)
    print_result(result, csv_path, json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
