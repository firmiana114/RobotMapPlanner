# Unitree Odometer Tools

This project holds odometer readers for Unitree robots. The current implemented
tool is `g1_odometer_probe`, which subscribes to the Unitree G1 odometer DDS
topics documented at:

https://support.unitree.com/home/zh/G1_developer/odometer_service_interface

It listens for `unitree_go::msg::dds_::SportModeState_` messages on:

- `rt/lf/odommodestate` for low frequency odometer data, documented as 20 Hz.
- `rt/odommodestate` for high frequency odometer data, documented as 500 Hz.

The repository has been renamed to `Odom` so a Go2 odometer reader can be added
alongside the G1 program later. The Go2 implementation is not added yet.

## Prerequisites

- Unitree SDK2 installed and discoverable by CMake through `find_package(unitree_sdk2 REQUIRED)`.
- Unitree SDK2 can also be placed at `./unitree_sdk2`; the top-level CMake file will use that local copy first.
- A Linux-compatible Unitree SDK2 build environment. The official SDK ships Linux prebuilt libraries for `aarch64` and `x86_64`, so it cannot be linked directly with the current macOS toolchain.
- A network interface connected to the robot. On this machine, `en11` was observed as an active wired interface during this work.

## Build

```sh
cmake -S . -B build
cmake --build build
```

## Run

Low frequency topic, 5 samples, 10 second timeout:

```sh
./build/g1_odometer_probe en11 --topic lf --samples 5 --timeout 10
```

Both low and high frequency topics:

```sh
./build/g1_odometer_probe en11 --topic both --samples 10 --timeout 10
```

Continuous low frequency output until Ctrl+C:

```sh
./build/g1_odometer_probe en11 --topic lf --samples 0 --timeout 0
```

Continuous high frequency subscription with one line printed every 100 samples:

```sh
./build/g1_odometer_probe en11 --topic hf --samples 0 --timeout 0 --print-every 100
```

Options:

- `--topic lf|hf|both`: subscribe to low frequency, high frequency, or both topics. Default: `lf`.
- `--samples N`: exit after receiving `N` total samples; `0` means run until interrupted. Default: `5`.
- `--timeout SEC`: exit with failure if no enough samples arrive before timeout; `0` disables timeout. Default: `10`.
- `--domain ID`: DDS domain id passed to `ChannelFactory::Init`. Default: `0`.
- `--print-every N`: print one sample every `N` received samples. Default: `1`.

The program logs startup configuration, DDS initialization, subscription setup, each printed sample, timeout, and exceptions at INFO/WARN/ERROR level.
