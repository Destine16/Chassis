#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path


META_STRUCT = struct.Struct("<IIIIfIII")
SAMPLE_STRUCT = struct.Struct("<hhf")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert ffid gdb dump binaries into a CSV file."
    )
    parser.add_argument("meta", type=Path, help="Path to ffid_meta.bin")
    parser.add_argument("buffer", type=Path, help="Path to ffid_buf.bin")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output CSV path. Default: alongside meta file with .csv suffix.",
    )
    return parser.parse_args()


def load_meta(meta_path: Path) -> dict[str, int]:
    data = meta_path.read_bytes()
    if len(data) < META_STRUCT.size:
        raise ValueError(
            f"meta file too small: expected at least {META_STRUCT.size} bytes, got {len(data)}"
        )

    (
        sample_hz,
        sample_count,
        capacity,
        wheel_id,
        speed_max_radps,
        speed_level_count,
        hold_duration_ms,
        settle_skip_ms,
    ) = META_STRUCT.unpack_from(data, 0)

    if sample_hz == 0:
        raise ValueError("meta.sample_hz is zero")
    if sample_count > capacity:
        raise ValueError(
            f"meta.sample_count ({sample_count}) exceeds capacity ({capacity})"
        )

    return {
        "sample_hz": sample_hz,
        "sample_count": sample_count,
        "capacity": capacity,
        "wheel_id": wheel_id,
        "speed_max_radps": speed_max_radps,
        "speed_level_count": speed_level_count,
        "hold_duration_ms": hold_duration_ms,
        "settle_skip_ms": settle_skip_ms,
    }


def iter_samples(buffer_path: Path, sample_count: int):
    data = buffer_path.read_bytes()
    required = sample_count * SAMPLE_STRUCT.size
    if len(data) < required:
        raise ValueError(
            f"buffer file too small: need {required} bytes for {sample_count} samples, got {len(data)}"
        )

    for index in range(sample_count):
        offset = index * SAMPLE_STRUCT.size
        u_raw, phase_id, speed_radps = SAMPLE_STRUCT.unpack_from(data, offset)
        yield index, u_raw, phase_id, speed_radps


def default_output_path(meta_path: Path) -> Path:
    return meta_path.with_suffix(".csv")


def step_name(step_id: int, speed_level_count: int) -> str:
    if step_id == 0:
        return "pre_hold"
    if step_id == 30000:
        return "mid_hold"
    if step_id == 30001:
        return "post_hold"
    if step_id == 30002:
        return "done"
    if 1 <= step_id <= speed_level_count:
        return f"pos_{step_id:02d}"
    if -int(speed_level_count) <= step_id <= -1:
        level = abs(step_id)
        return f"neg_{level:02d}"
    return "unknown"


def speed_ref_from_step(step_id: int, speed_max_radps: float, speed_level_count: int) -> float:
    if step_id <= 0:
        return 0.0
    return (float(step_id) * speed_max_radps) / float(speed_level_count)


def signed_speed_ref_from_step(step_id: int, speed_max_radps: float, speed_level_count: int) -> float:
    if step_id in (0, 30000, 30001, 30002):
        return 0.0
    if step_id > 0:
        return speed_ref_from_step(step_id, speed_max_radps, speed_level_count)
    return -(float(abs(step_id)) * speed_max_radps) / float(speed_level_count)


def write_csv(output_path: Path, meta: dict[str, int], buffer_path: Path) -> None:
    sample_hz = meta["sample_hz"]

    with output_path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)

        writer.writerow(["meta_key", "meta_value"])
        writer.writerow(["sample_hz", sample_hz])
        writer.writerow(["sample_count", meta["sample_count"]])
        writer.writerow(["capacity", meta["capacity"]])
        writer.writerow(["wheel_id", meta["wheel_id"]])
        writer.writerow(["speed_max_radps", f"{meta['speed_max_radps']:.6f}"])
        writer.writerow(["speed_level_count", meta["speed_level_count"]])
        writer.writerow(["hold_duration_ms", meta["hold_duration_ms"]])
        writer.writerow(["settle_skip_ms", meta["settle_skip_ms"]])
        writer.writerow([])
        writer.writerow(
            [
                "sample_idx",
                "t_s",
                "u_raw",
                "step_id",
                "step_name",
                "speed_ref_radps",
                "speed_radps",
            ]
        )

        for sample_idx, u_raw, step_id, speed_radps in iter_samples(
            buffer_path, meta["sample_count"]
        ):
            t_s = sample_idx / float(sample_hz)
            speed_ref_radps = signed_speed_ref_from_step(
                step_id,
                float(meta["speed_max_radps"]),
                int(meta["speed_level_count"]),
            )
            writer.writerow(
                [
                    sample_idx,
                    f"{t_s:.6f}",
                    u_raw,
                    step_id,
                    step_name(step_id, int(meta["speed_level_count"])),
                    f"{speed_ref_radps:.9f}",
                    f"{speed_radps:.9f}",
                ]
            )


def main() -> int:
    args = parse_args()

    try:
        meta = load_meta(args.meta)
        output_path = args.output if args.output is not None else default_output_path(args.meta)
        write_csv(output_path, meta, args.buffer)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"wrote {output_path}")
    print(
        "summary:"
        f" sample_hz={meta['sample_hz']}"
        f" sample_count={meta['sample_count']}"
        f" wheel_id={meta['wheel_id']}"
        f" speed_max_radps={meta['speed_max_radps']:.3f}"
        f" speed_level_count={meta['speed_level_count']}"
        f" hold_duration_ms={meta['hold_duration_ms']}"
        f" settle_skip_ms={meta['settle_skip_ms']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
