# orecchiette-sdr-hackrf-rs

[![CI](https://github.com/isaacbentley/orecchiette-sdr-hackrf-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/isaacbentley/orecchiette-sdr-hackrf-rs/actions/workflows/ci.yml)
[![License: GPL-3.0-or-later](https://img.shields.io/github/license/isaacbentley/orecchiette-sdr-hackrf-rs.svg)](https://choosealicense.com/licenses/gpl-3.0/)

A pure-Rust implementation of the [`SdrSource`](https://github.com/isaacbentley/orecchiette-sdr-source-rs) trait for the Great Scott Gadgets **HackRF One**. This crate enables seamless integration of HackRF hardware into an SDR detection orchestrator, supporting high-speed IQ capture, dynamic channel hopping, and adaptive dwell strategies.

## Overview

Traditional SDR integrations often require complex C libraries (like `libhackrf` and `libusb`) which complicate cross-platform deployment and CI/CD pipelines. `hackrfone` v0.4 talks to the device over USB via `nusb` (pure Rust), so neither `libhackrf` nor `libusb` is required to build or run — the crate builds cleanly on macOS, Linux, and Windows with no system dependencies.

## Key Features

- **Pure-Rust Architecture:** Zero C dependencies, memory-safe throughout, no build-time system library requirements.
- **Complete Gain Management:** Fine-grained control over LNA (IF) gain, VGA (baseband) gain, and the +14 dB front-end RF amplifier.
- **Bias-Tee Support:** Configurable antenna-port DC power for active antennas or external LNAs.
- **Adaptive Channel Hopping:** Automatically retunes across frequency lists with integrated per-hop pacing using `DwellController`.
- **Automatic IQ Scaling:** Transparently scales raw 8-bit signed IQ data to `Complex32` in `[-1, 1)` to match the orchestrator's common `IqPacket` format.

## Caveats vs. the USRP backend

- **8-bit dynamic range**: ~4 fewer bits than the B210's 12-bit path — expect a noisier picture on weak signals.
- **~20 MSPS ceiling**: HackRF One is USB 2.0; above ~20 MSPS the bulk transport drops samples. Analog FPV FM occupies ~20 MHz, so the device is right at its useful limit for full-quality video. `start` clamps any larger requested rate to `HACKRF_MAX_SAMPLE_RATE_HZ` (20 MSPS).
- **No hardware overrun flag**: The bulk RX read surfaces no dropped-sample metadata, so `IqPacket::overrun` is always `false`; the orchestrator's overrun-driven rate step-down won't fire (drops show up as visible glitches instead).

## Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
orecchiette-sdr-hackrf-rs = "0.1.0"
orecchiette-sdr-source-rs = "0.1.0"
```

## Usage

### Basic Single-Channel Capture

```rust,no_run
use orecchiette_sdr_hackrf_rs::HackRfSource;
use orecchiette_sdr_source_rs::{DwellAdvice, SdrSource, SourceConfig};
use std::sync::Arc;
use std::time::{Duration, Instant};

struct NoSignalLog;
impl DwellAdvice for NoSignalLog {
    fn latest_signal_at(&self, _freq_key_khz: u64) -> Option<Instant> { None }
}

let advice: Arc<dyn DwellAdvice> = Arc::new(NoSignalLog);

// Configure device gains
let source = Box::new(HackRfSource {
    lna_gain:   16,    // 0–40, 8 dB steps
    vga_gain:   20,    // 0–62, 2 dB steps
    amp_enable: false, // front-end +14 dB amp
    bias_tee:   false, // antenna-port DC power
});

// Configure capture parameters
let config = SourceConfig {
    sample_rate_hz:  20_000_000.0,
    channels_hz:     vec![5_845e6],   // single channel
    dwell_min:       Duration::from_secs(3600),
    dwell_max:       Duration::from_secs(3600),
    dwell_extension: Duration::ZERO,
};

// Start streaming
let handle = source.start(config, advice).unwrap();

// Process incoming IQ packets
for packet in handle.receiver.iter() {
    // packet.samples: PooledIqBuffer (use like &[Complex32], 8-bit → [-1, 1))
    // packet.center_frequency_hz, packet.sample_rate_hz
}
```

## Builder Fields

| Field | Default | Notes |
|---|---|---|
| `lna_gain` | `16` | LNA (IF) gain in dB, 0–40 in 8 dB steps (device rounds down). |
| `vga_gain` | `20` | VGA (baseband) gain in dB, 0–62 in 2 dB steps. |
| `amp_enable` | `false` | Front-end +14 dB RF amplifier. Overloads easily on strong ambient traffic. |
| `bias_tee` | `false` | Antenna-port DC power for active antennas / external LNAs. |

## MSRV & Semver Policy

- **MSRV:** This crate does not maintain an explicit Minimum Supported Rust Version (MSRV) policy and tracks the latest `stable` compiler.
- **Semver:** This crate follows semantic versioning. While in `0.x.y`, breaking API changes will result in a minor version bump (e.g. `0.1.x` to `0.2.0`).

## Testing & Contributing

Tests cover hardware instantiation, trait contract fulfillment, integration with the adaptive dwell controller, and clean shutdown behavior.

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for detailed instructions on running the test suite and formatting your code before submitting a Pull Request.

## Related Projects

- **[fpv-viewer-rs](https://github.com/isaacbentley/fpv-viewer-rs)** — the parent SDR orchestrator.

## Documentation

- [Architecture & Design](DESIGN.md) — internal architecture and execution flow.

## License

This project is licensed under the GNU General Public License v3.0 or later (GPL-3.0-or-later) - see the [LICENSE](LICENSE) file for details.
