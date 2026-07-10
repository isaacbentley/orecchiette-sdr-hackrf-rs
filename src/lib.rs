#![doc = include_str!("../README.md")]
//! HackRF One SDR source for SDR applications.
//!
//! Implements [`orecchiette_sdr_source_rs::SdrSource`] for the Great Scott Gadgets
//! HackRF One, using the pure-Rust [`hackrfone`] driver (USB via `nusb`
//! — no `libhackrf` *or* libusb C library needed, so it builds and runs
//! with zero system dependencies). Owns the device handle, the
//! channel-hop loop, and the IQ conversion. The orchestrator consumes
//! [`IqPacket`]s through the receiver returned in [`SdrHandle`].
//!
//! ## Caveats vs. the USRP backend
//!
//! - **8-bit samples.** The HackRF's ADC delivers interleaved signed
//!   8-bit I/Q; we scale to `[-1, 1)` `Complex32`. That's ~4 fewer bits
//!   of dynamic range than the B210's 12-bit path, so expect a noisier
//!   picture on weak signals.
//! - **~20 MSPS ceiling (USB 2.0).** Analog FPV FM occupies ~20 MHz, so
//!   the HackRF is right at its limit for full-quality video; 16–20 MSPS
//!   is the usable range.
//! - **No hardware overrun flag.** `hackrfone`'s bulk RX read doesn't
//!   surface dropped-sample metadata, so [`IqPacket::overrun`] is always
//!   `false` here (the viewer's overrun-driven rate step-down won't fire
//!   for HackRF — drops show up as visible glitches instead).
//! - **Retune requires an RX-mode round-trip.** `set_freq` lives on the
//!   `UnknownMode` typestate, so a channel hop stops RX, retunes, and
//!   re-enters RX — mirroring the USRP backend's per-hop streamer
//!   recreation.

use crossbeam::channel;
use hackrfone::HackRfOne;
use num_complex::Complex32;
use orecchiette_sdr_source_rs::{
    DwellAdvice, DwellController, IqPacket, SdrError, SdrHandle, SdrSource, SourceConfig,
    freq_key_khz,
};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use tracing::info;

/// Highest RX sample rate we let a caller request. The HackRF One is a
/// USB 2.0 device; above ~20 MSPS the bulk transport can't keep up and
/// drops samples wholesale.
pub const HACKRF_MAX_SAMPLE_RATE_HZ: f64 = 20_000_000.0;

/// Builder for a HackRF One source. Wrap in `Box::new(...)` and call
/// [`SdrSource::start`] from the orchestrator.
pub struct HackRfSource {
    /// LNA (IF) gain in dB, 0–40 in 8 dB steps. Rounded down to the
    /// nearest step by the device. Default 16.
    pub lna_gain: u16,
    /// VGA (baseband) gain in dB, 0–62 in 2 dB steps. Default 20.
    pub vga_gain: u16,
    /// Front-end +14 dB RF amplifier. Off by default — it overloads
    /// easily on strong ambient ISM traffic.
    pub amp_enable: bool,
    /// Bias-tee (antenna port DC power) for active antennas / LNAs.
    /// Off by default.
    pub bias_tee: bool,
}

impl Default for HackRfSource {
    fn default() -> Self {
        Self {
            lna_gain: 16,
            vga_gain: 20,
            amp_enable: false,
            bias_tee: false,
        }
    }
}

impl SdrSource for HackRfSource {
    fn start(
        self: Box<Self>,
        config: SourceConfig,
        advice: Arc<dyn DwellAdvice>,
    ) -> Result<SdrHandle, SdrError> {
        if config.channels_hz.is_empty() {
            return Err(SdrError::BadConfig(
                "SourceConfig.channels_hz must not be empty".into(),
            ));
        }

        // Clamp the requested rate to the HackRF's USB-2.0 ceiling.
        let sample_rate = config.sample_rate_hz.min(HACKRF_MAX_SAMPLE_RATE_HZ);
        if sample_rate <= 0.0 {
            return Err(SdrError::BadConfig(format!(
                "invalid sample rate {} Hz",
                config.sample_rate_hz
            )));
        }
        if config.sample_rate_hz > HACKRF_MAX_SAMPLE_RATE_HZ {
            info!(
                "[hackrf] Requested {:.2} MSPS exceeds the {:.0} MSPS USB-2.0 ceiling; clamping.",
                config.sample_rate_hz / 1e6,
                HACKRF_MAX_SAMPLE_RATE_HZ / 1e6
            );
        }

        let mut radio = HackRfOne::new().ok_or_else(|| {
            SdrError::NotFound(
                "No HackRF One found. Ensure it is connected and not claimed by another process."
                    .into(),
            )
        })?;

        info!(
            "[hackrf] Configuring: Rate={:.2} MSPS | LNA={} dB | VGA={} dB | Amp={} | BiasTee={}",
            sample_rate / 1e6,
            self.lna_gain,
            self.vga_gain,
            self.amp_enable,
            self.bias_tee
        );

        radio
            .set_sample_rate(sample_rate as u32, 1)
            .map_err(|e| SdrError::BadConfig(format!("set_sample_rate({sample_rate}): {e:?}")))?;
        radio
            .set_lna_gain(self.lna_gain)
            .map_err(|e| SdrError::BadConfig(format!("set_lna_gain({}): {e:?}", self.lna_gain)))?;
        radio
            .set_vga_gain(self.vga_gain)
            .map_err(|e| SdrError::BadConfig(format!("set_vga_gain({}): {e:?}", self.vga_gain)))?;
        radio
            .set_amp_enable(self.amp_enable)
            .map_err(|e| SdrError::BadConfig(format!("set_amp_enable: {e:?}")))?;
        radio
            .set_antenna_enable(self.bias_tee as u8)
            .map_err(|e| SdrError::BadConfig(format!("set_antenna_enable: {e:?}")))?;

        let dwell_controller = DwellController {
            min: config.dwell_min,
            max: config.dwell_max,
            extension: config.dwell_extension,
        };
        let channels_hz = config.channels_hz.clone();
        let num_channels = channels_hz.len();
        if dwell_controller.is_adaptive() {
            info!(
                "[hackrf] Starting scan: {} channels, adaptive dwell {}–{}ms (+{}ms per detection)",
                num_channels,
                config.dwell_min.as_millis(),
                config.dwell_max.as_millis(),
                config.dwell_extension.as_millis()
            );
        } else {
            info!(
                "[hackrf] Starting scan: {} channels, fixed {}ms dwell per channel",
                num_channels,
                config.dwell_min.as_millis()
            );
        }

        let (tx, receiver) = channel::bounded::<IqPacket>(1024);
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_flag_thread = stop_flag.clone();
        let advice_thread = advice;
        let sample_rate_f32 = sample_rate as f32;

        let (pool_tx, pool_rx) = channel::bounded::<Vec<Complex32>>(1024);
        for _ in 0..1024 {
            let _ = pool_tx.send(Vec::with_capacity(131072));
        }

        let lna_gain = self.lna_gain;
        let vga_gain = self.vga_gain;
        let amp_enable = self.amp_enable;
        let bias_tee = self.bias_tee;

        let capture_thread = thread::spawn(move || {
            let panic_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                if let Err(e) = (move || -> Result<(), anyhow::Error> {
                    // The HackRF typestate puts `set_freq` on `UnknownMode` and
                    // `rx` on `RxMode`, so we thread the single device handle
                    // through `into_rx_mode` / `stop_rx` around each hop. For
                    // the common single-channel viewer case (`channels_hz.len()
                    // == 1`, dwell ~forever) the outer loop runs once and we
                    // just stream.
                    let mut device = radio; // HackRfOne<UnknownMode>
                    let mut channel_idx = 0usize;
                    let mut last_report = Instant::now();
                    let mut channel_switches = 0u64;
                    let mut consecutive_failures = 0;

                    'outer: loop {
                        if stop_flag_thread.load(Ordering::SeqCst) {
                            break;
                        }

                        if consecutive_failures >= num_channels {
                            tracing::warn!("[hackrf] All channels failed consecutively. Sleeping for 500ms before retrying.");
                            thread::sleep(Duration::from_millis(500));
                            consecutive_failures = 0;
                        }

                        let current_freq_hz = channels_hz[channel_idx];
                        let freq_key = freq_key_khz(current_freq_hz);
                        if let Err(e) = device.set_freq(current_freq_hz as u64) {
                            tracing::warn!("[hackrf] Failed to set frequency to {} Hz: {:?}. Skipping channel.", current_freq_hz, e);
                            consecutive_failures += 1;
                            channel_idx = (channel_idx + 1) % num_channels;
                            continue;
                        }

                        let mut rx = match device.into_rx_mode() {
                            Ok(r) => r,
                            Err(e) => {
                                tracing::warn!("[hackrf] into_rx_mode failed for {} Hz: {:?}. Attempting to re-open/recreate device.", current_freq_hz, e);
                                consecutive_failures += 1;
                                thread::sleep(Duration::from_millis(100));
                                if let Some(new_radio) = hackrfone::HackRfOne::new() {
                                    if let Err(e2) = new_radio.set_sample_rate(sample_rate as u32, 1) {
                                        tracing::error!("[hackrf] Failed to re-set sample rate: {:?}", e2);
                                    }
                                    if let Err(e2) = new_radio.set_lna_gain(lna_gain) {
                                        tracing::error!("[hackrf] Failed to re-set LNA gain: {:?}", e2);
                                    }
                                    if let Err(e2) = new_radio.set_vga_gain(vga_gain) {
                                        tracing::error!("[hackrf] Failed to re-set VGA gain: {:?}", e2);
                                    }
                                    if let Err(e2) = new_radio.set_amp_enable(amp_enable) {
                                        tracing::error!("[hackrf] Failed to re-set amp enable: {:?}", e2);
                                    }
                                    if let Err(e2) = new_radio.set_antenna_enable(bias_tee as u8) {
                                        tracing::error!("[hackrf] Failed to re-set antenna enable: {:?}", e2);
                                    }
                                    device = new_radio;
                                } else {
                                    tracing::error!("[hackrf] Failed to re-open HackRF device.");
                                }
                                channel_idx = (channel_idx + 1) % num_channels;
                                continue;
                            }
                        };

                        // Reset consecutive failures on successful tune/start
                        consecutive_failures = 0;

                        let dwell_start = Instant::now();
                        // The loop yields the device back in `UnknownMode` (via
                        // `stop_rx`) so the next hop can retune it.
                        device = loop {
                            if stop_flag_thread.load(Ordering::SeqCst) {
                                break rx
                                    .stop_rx()
                                    .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                            }
                            let latest_signal = advice_thread.latest_signal_at(freq_key);
                            let deadline = dwell_controller.deadline(dwell_start, latest_signal);
                            if Instant::now() >= deadline {
                                break rx
                                    .stop_rx()
                                    .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                            }

                            match rx.rx() {
                                Ok(bytes) => {
                                    // Interleaved signed 8-bit I, Q → Complex32 in
                                    // [-1, 1). `chunks_exact(2)` drops a trailing
                                    // odd byte (never expected from the device).
                                    let mut samples = pool_rx
                                        .try_recv()
                                        .unwrap_or_else(|_| Vec::with_capacity(131072));
                                    samples.clear();
                                    samples.extend(bytes.chunks_exact(2).map(|c| {
                                        Complex32::new(
                                            (c[0] as i8) as f32 / 127.0,
                                            (c[1] as i8) as f32 / 127.0,
                                        )
                                    }));
                                    if !samples.is_empty() {
                                        let pkt = IqPacket {
                                            samples:
                                                orecchiette_sdr_source_rs::PooledIqBuffer::new_pooled(
                                                    samples,
                                                    pool_tx.clone(),
                                                ),
                                            center_frequency_hz: current_freq_hz,
                                            sample_rate_hz: sample_rate_f32,
                                            overrun: false,
                                        };
                                        if tx.send(pkt).is_err() {
                                            // Receiver dropped — wind down.
                                            let _ = rx.stop_rx();
                                            break 'outer;
                                        }
                                    }
                                }
                                Err(e) => {
                                    // A transient USB read error ends this dwell;
                                    // the outer loop retunes and re-enters RX.
                                    tracing::warn!("[hackrf] rx error: {e:?}");
                                    break rx
                                        .stop_rx()
                                        .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                                }
                            }

                            if last_report.elapsed() >= Duration::from_secs(60) {
                                let rate =
                                    channel_switches as f32 / last_report.elapsed().as_secs_f32();
                                info!(
                                    "[hackrf] Scanning speed: {:.1} ch/s | Pool size: {} channels",
                                    rate, num_channels
                                );
                                channel_switches = 0;
                                last_report = Instant::now();
                            }
                        };

                        channel_idx = (channel_idx + 1) % num_channels;
                        channel_switches += 1;
                    }
                    Ok(())
                })() {
                    tracing::error!("[hackrf] Capture thread failed: {:?}", e);
                }
            }));
            if let Err(e) = panic_res {
                tracing::error!("[hackrf] Capture thread panicked: {:?}", e);
            }
        });

        let stop_flag_for_stop = stop_flag.clone();
        let stop = Box::new(move || {
            stop_flag_for_stop.store(true, Ordering::SeqCst);
        });
        let wait = Box::new(move || {
            if let Err(e) = capture_thread.join() {
                tracing::error!("[hackrf] capture thread join failed: {:?}", e);
            }
        });

        Ok(SdrHandle {
            receiver,
            stop,
            wait,
        })
    }
}
