# Chassis Project

English | [简体中文](README_zh.md)

A four-wheel omni chassis control project based on `STM32F405 + M3508 + DR16/DBUS + CAN1 + USB CDC + BMI088`.

Current system boundaries:

- Chassis motion is controlled only by the remote controller.
- The embedded controller sends wheel-speed and inertial observations to navigation through `USB CDC`.

## Quick Overview

- Body frame: `+vx = forward`, `+vy = left`, `+wz = counter-clockwise`
- Chassis: `45°` four-wheel omni chassis
- Wheel radius: `81.5 mm`
- Wheel diagonal distance: `425 mm`
- ESC IDs:
  - `front_left = 2`
  - `front_right = 1`
  - `rear_left = 3`
  - `rear_right = 4`
- Remote controller: `USART3_RX(PB11)`, `100000 / 9-bit / Even / 1 stop bit`
- Navigation observation link: `USB CDC @ 200 Hz`
- Center-return active braking: default `K_brake = 1030`

## Main Pipeline

```text
DR16/DBUS
-> RC service layer
-> SAFE / MANUAL arbitration
-> chassis control
-> inverse kinematics / wheel-speed proportional scaling / velocity feedforward + velocity PID / center-return active braking
-> CAN1 -> C620 -> M3508
```

## Documentation

- Architecture: [docs/architecture.md](docs/architecture.md)
- Control and remote mapping: [docs/control.md](docs/control.md)
- Center-return active braking tuning: [docs/brake_tuning.md](docs/brake_tuning.md)
- IMU and body frame: [docs/imu.md](docs/imu.md)
- USB CDC protocol: [docs/telemetry.md](docs/telemetry.md)
- Identification and tuning: [docs/identification.md](docs/identification.md)

Specialized documents:

- [docs/sysid_gdb.md](docs/sysid_gdb.md)
- [docs/sysid_identify.md](docs/sysid_identify.md)
- [docs/pid_tune.md](docs/pid_tune.md)

## Key Scripts

- USB CDC local monitor:
  - [tools/nav_cdc_monitor.py](tools/nav_cdc_monitor.py)
- PRBS dump to CSV:
  - [tools/sysid_dump_to_csv.py](tools/sysid_dump_to_csv.py)
- PRBS model identification:
  - [tools/sysid_identify.py](tools/sysid_identify.py)
- FFID dump to CSV:
  - [tools/ffid_dump_to_csv.py](tools/ffid_dump_to_csv.py)
- FFID feedforward fitting:
  - [tools/ffid_identify.py](tools/ffid_identify.py)
- PID tuning:
  - [tools/pid_tune.py](tools/pid_tune.py)
- Center-return active braking capture:
  - [tools/brake_capture.py](tools/brake_capture.py)
- Center-return active braking discrete analysis:
  - [tools/brake_analyze.py](tools/brake_analyze.py)
- Center-return active braking continuous optimization:
  - [tools/brake_optimize.py](tools/brake_optimize.py)
- IMU static level calibration:
  - [tools/imu_static_level_calibrate.py](tools/imu_static_level_calibrate.py)
- IMU yaw calibration:
  - [tools/imu_yaw_calibrate.py](tools/imu_yaw_calibrate.py)

Prefer the project virtual environment for analysis scripts:

```bash
.venv/bin/python tools/...
```

## Build

```bash
cmake -S . -B build/Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug -j
```

Release builds disable experimental code by default (`PRBS / FFID / wheeltest`). To enable it again:

```bash
cmake -S . -B build/Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCHASSIS_ENABLE_EXPERIMENTS=ON
```

Build artifacts:

```text
build/Debug/Chassis.elf
build/Debug/Chassis.bin
```
