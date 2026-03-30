#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path


META_STRUCT = struct.Struct("<IIIIiII")
SAMPLE_STRUCT = struct.Struct("<hhf")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert sysid gdb dump binaries into a CSV file."
    )
    parser.add_argument("meta", type=Path, help="Path to sysid_meta.bin")
    parser.add_argument("buffer", type=Path, help="Path to sysid_buf.bin")
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
        amplitude_raw,
        bit_period_ms,
        total_bits,
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
        "amplitude_raw": amplitude_raw,
        "bit_period_ms": bit_period_ms,
        "total_bits": total_bits,
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
        u_raw, _reserved, speed_radps = SAMPLE_STRUCT.unpack_from(data, offset)
        yield index, u_raw, speed_radps


def default_output_path(meta_path: Path) -> Path:
    return meta_path.with_suffix(".csv")


def write_csv(output_path: Path, meta: dict[str, int], buffer_path: Path) -> None:
    sample_hz = meta["sample_hz"]

    with output_path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)

        writer.writerow(["meta_key", "meta_value"])
        writer.writerow(["sample_hz", sample_hz])
        writer.writerow(["sample_count", meta["sample_count"]])
        writer.writerow(["capacity", meta["capacity"]])
        writer.writerow(["wheel_id", meta["wheel_id"]])
        writer.writerow(["amplitude_raw", meta["amplitude_raw"]])
        writer.writerow(["bit_period_ms", meta["bit_period_ms"]])
        writer.writerow(["total_bits", meta["total_bits"]])
        writer.writerow([])
        writer.writerow(["sample_idx", "t_s", "u_raw", "speed_radps"])

        for sample_idx, u_raw, speed_radps in iter_samples(
            buffer_path, meta["sample_count"]
        ):
            t_s = sample_idx / float(sample_hz)
            writer.writerow([sample_idx, f"{t_s:.6f}", u_raw, f"{speed_radps:.9f}"])


def main() -> int:
    args = parse_args()

    try:
        meta = load_meta(args.meta)
        output_path = args.output if args.output is not None else default_output_path(args.meta)
        write_csv(output_path, meta, args.buffer)
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"wrote {output_path}")
    print(
        "summary:"
        f" sample_hz={meta['sample_hz']}"
        f" sample_count={meta['sample_count']}"
        f" wheel_id={meta['wheel_id']}"
        f" amplitude_raw={meta['amplitude_raw']}"
        f" bit_period_ms={meta['bit_period_ms']}"
        f" total_bits={meta['total_bits']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
