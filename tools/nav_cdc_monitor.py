#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import struct
import sys
import time
from dataclasses import dataclass

import serial

SOF0 = 0xA5
SOF1 = 0x5A
PAYLOAD_LEN = 46
FRAME_LEN = 4 + PAYLOAD_LEN + 2

FLAG_NAMES = {
    0: "imu_valid",
    1: "wheel_fl_valid",
    2: "wheel_fr_valid",
    3: "wheel_rl_valid",
    4: "wheel_rr_valid",
    5: "imu_fault",
    6: "motor_fault",
    7: "chassis_safe_mode",
    8: "rc_offline",
}


@dataclass
class Observation:
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


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def autodetect_port() -> str:
    candidates = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise FileNotFoundError("No /dev/ttyACM* or /dev/ttyUSB* device found")
    return candidates[0]


def decode_flags(flags: int) -> str:
    names = [name for bit, name in FLAG_NAMES.items() if flags & (1 << bit)]
    return ",".join(names) if names else "-"


def parse_observation(frame: bytes) -> Observation:
    seq = frame[2]
    payload = frame[4:4 + PAYLOAD_LEN]
    (
        t_ms,
        w_fl,
        w_fr,
        w_rl,
        w_rr,
        gyro_x,
        gyro_y,
        gyro_z,
        acc_x,
        acc_y,
        acc_z,
        flags,
    ) = struct.unpack("<I10fH", payload)
    return Observation(
        seq=seq,
        t_ms=t_ms,
        w_fl=w_fl,
        w_fr=w_fr,
        w_rl=w_rl,
        w_rr=w_rr,
        gyro_x=gyro_x,
        gyro_y=gyro_y,
        gyro_z=gyro_z,
        acc_x=acc_x,
        acc_y=acc_y,
        acc_z=acc_z,
        flags=flags,
    )


def read_frames(ser: serial.Serial):
    buf = bytearray()
    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
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

            expected_crc = struct.unpack("<H", frame[-2:])[0]
            actual_crc = crc16_ccitt(frame[:-2])
            if expected_crc != actual_crc:
                del buf[0]
                continue

            del buf[:FRAME_LEN]
            yield parse_observation(frame)


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor current USB CDC navigation observation frames.")
    parser.add_argument("-p", "--port", help="Serial port path, e.g. /dev/ttyACM0")
    parser.add_argument("-n", "--count", type=int, default=0, help="Stop after N valid frames (0 = run forever)")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout in seconds")
    parser.add_argument("--summary-every", type=int, default=50, help="Print rate/loss summary every N valid frames")
    args = parser.parse_args()

    try:
        port = args.port or autodetect_port()
    except FileNotFoundError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1

    print(f"[info] opening {port}")
    ser = serial.Serial(port=port, baudrate=115200, timeout=args.timeout)

    last_seq = None
    dropped = 0
    start = time.monotonic()
    count = 0

    try:
        for obs in read_frames(ser):
            count += 1
            if last_seq is not None:
                delta = (obs.seq - last_seq) & 0xFF
                if delta > 1:
                    dropped += delta - 1
            last_seq = obs.seq

            print(
                f"seq={obs.seq:3d} t={obs.t_ms:8d}ms "
                f"w=[{obs.w_fl:7.2f},{obs.w_fr:7.2f},{obs.w_rl:7.2f},{obs.w_rr:7.2f}] "
                f"g=[{obs.gyro_x:7.3f},{obs.gyro_y:7.3f},{obs.gyro_z:7.3f}] "
                f"a=[{obs.acc_x:7.3f},{obs.acc_y:7.3f},{obs.acc_z:7.3f}] "
                f"flags=0x{obs.flags:04x}({decode_flags(obs.flags)})"
            )

            if args.summary_every > 0 and count % args.summary_every == 0:
                elapsed = max(time.monotonic() - start, 1e-6)
                print(
                    f"[summary] frames={count} rate={count / elapsed:.1f} Hz dropped={dropped}",
                    file=sys.stderr,
                )

            if args.count > 0 and count >= args.count:
                break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = max(time.monotonic() - start, 1e-6)
    print(f"[done] frames={count} rate={count / elapsed:.1f} Hz dropped={dropped}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
