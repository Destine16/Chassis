#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import serial

from nav_cdc_monitor import autodetect_port, decode_flags, judge_observation, read_frames

WHEEL_RADIUS_M = 0.0815
OMNI_45DEG_COEF = 0.70710678118
CENTER_TO_WHEEL_M = 0.425 * 0.5


def forward_kinematics(w_fl: float, w_fr: float, w_rl: float, w_rr: float) -> tuple[float, float, float]:
    vx = (WHEEL_RADIUS_M / (4.0 * OMNI_45DEG_COEF)) * (w_fl + w_fr + w_rl + w_rr)
    vy = (WHEEL_RADIUS_M / (4.0 * OMNI_45DEG_COEF)) * (-w_fl + w_fr + w_rl - w_rr)
    wz = (WHEEL_RADIUS_M / (4.0 * CENTER_TO_WHEEL_M)) * (-w_fl + w_fr - w_rl + w_rr)
    return vx, vy, wz


def default_output_path(k_brake: float, direction: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe_direction = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in direction)
    return Path("data/brake") / f"brake_k{k_brake:g}_{safe_direction}_{stamp}.csv"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture USB CDC observation frames for chassis active-brake tuning."
    )
    parser.add_argument("-p", "--port", help="Serial port path, e.g. /dev/ttyACM0")
    parser.add_argument("-o", "--output", type=Path, help="CSV output path")
    parser.add_argument("--k-brake", type=float, required=True, help="K_brake value used in firmware")
    parser.add_argument("--direction", default="unknown", help="Trial direction label: forward/back/left/right")
    parser.add_argument("--duration", type=float, default=0.0, help="Capture duration in seconds, 0 = until Ctrl+C")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout in seconds")
    parser.add_argument("--no-wait", action="store_true", help="Start immediately instead of waiting for Enter")
    parser.add_argument("--print-every", type=int, default=40, help="Print one status line every N frames")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    try:
        port = args.port or autodetect_port()
    except FileNotFoundError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1

    output = args.output or default_output_path(args.k_brake, args.direction)
    output.parent.mkdir(parents=True, exist_ok=True)

    if not args.no_wait:
        input(
            "Set g_ctrl_chassis_brake_param.k_brake, prepare the chassis, "
            "then press Enter to start capture..."
        )

    fieldnames = [
        "host_t_s",
        "rel_t_s",
        "seq",
        "t_ms",
        "k_brake",
        "direction_label",
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
        "gyro_x_radps",
        "gyro_y_radps",
        "gyro_z_radps",
        "acc_x_mps2",
        "acc_y_mps2",
        "acc_z_mps2",
        "flags",
        "flags_text",
        "judge",
    ]

    print(f"[info] opening {port}")
    print(f"[info] writing {output}")

    ser = serial.Serial(port=port, baudrate=115200, timeout=args.timeout)
    start = time.monotonic()
    count = 0

    try:
        with output.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            writer.writeheader()

            for obs in read_frames(ser):
                now = time.monotonic()
                rel_t_s = now - start
                vx, vy, wz = forward_kinematics(obs.w_fl, obs.w_fr, obs.w_rl, obs.w_rr)
                speed_planar = math.hypot(vx, vy)
                yaw_equiv = abs(wz) * CENTER_TO_WHEEL_M
                max_abs_wheel = max(abs(obs.w_fl), abs(obs.w_fr), abs(obs.w_rl), abs(obs.w_rr))
                judge = judge_observation(obs)

                writer.writerow(
                    {
                        "host_t_s": f"{now:.6f}",
                        "rel_t_s": f"{rel_t_s:.6f}",
                        "seq": obs.seq,
                        "t_ms": obs.t_ms,
                        "k_brake": f"{args.k_brake:.6g}",
                        "direction_label": args.direction,
                        "w_fl_radps": f"{obs.w_fl:.6f}",
                        "w_fr_radps": f"{obs.w_fr:.6f}",
                        "w_rl_radps": f"{obs.w_rl:.6f}",
                        "w_rr_radps": f"{obs.w_rr:.6f}",
                        "vx_mps": f"{vx:.6f}",
                        "vy_mps": f"{vy:.6f}",
                        "wz_radps": f"{wz:.6f}",
                        "speed_planar_mps": f"{speed_planar:.6f}",
                        "yaw_equiv_mps": f"{yaw_equiv:.6f}",
                        "max_abs_wheel_radps": f"{max_abs_wheel:.6f}",
                        "gyro_x_radps": f"{obs.gyro_x:.6f}",
                        "gyro_y_radps": f"{obs.gyro_y:.6f}",
                        "gyro_z_radps": f"{obs.gyro_z:.6f}",
                        "acc_x_mps2": f"{obs.acc_x:.6f}",
                        "acc_y_mps2": f"{obs.acc_y:.6f}",
                        "acc_z_mps2": f"{obs.acc_z:.6f}",
                        "flags": f"0x{obs.flags:04x}",
                        "flags_text": decode_flags(obs.flags),
                        "judge": judge,
                    }
                )
                count += 1

                if args.print_every > 0 and count % args.print_every == 0:
                    print(
                        f"t={rel_t_s:7.2f}s k={args.k_brake:g} "
                        f"v=({vx:+.3f},{vy:+.3f}) wz={wz:+.3f} "
                        f"speed={speed_planar:.3f} max_w={max_abs_wheel:.2f} {judge}"
                    )

                if args.duration > 0.0 and rel_t_s >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = max(time.monotonic() - start, 1e-6)
    print(f"[done] frames={count} elapsed={elapsed:.2f}s rate={count / elapsed:.1f}Hz output={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
