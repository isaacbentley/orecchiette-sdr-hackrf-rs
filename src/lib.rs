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

/// Scale one signed 8-bit ADC sample to `[-1, 1)`.
///
/// Divides by 128, not 127. `i8` spans -128..=127, so dividing by 127 maps
/// full-scale negative to -1.008 — outside the range this module documents,
/// and reached by any strong signal that drives the ADC to its rail. 128 is
/// the half-range, giving exactly [-1, 1): -128 -> -1.0 and +127 -> +0.992.
#[inline]
fn i8_to_unit(byte: u8) -> f32 {
    f32::from(byte as i8) / 128.0
}

/// Highest RX sample rate we let a caller request. The HackRF One is a
/// USB 2.0 device; above ~20 MSPS the bulk transport can't keep up and
/// drops samples wholesale.
pub const HACKRF_MAX_SAMPLE_RATE_HZ: f64 = 20_000_000.0;

/// After this many consecutive full sweeps where every channel fails
/// to tune/stream (each sweep followed by a 500ms backoff sleep), give
/// up on the device rather than retrying forever — a HackRF that's
/// been unplugged or wedged should surface as a terminal error instead
/// of spinning silently.
const MAX_CONSECUTIVE_SWEEP_FAILURES: u32 = 10;

/// Consecutive RX-streaming failures tolerated before giving up on the device.
///
/// Distinct from `MAX_CONSECUTIVE_SWEEP_FAILURES`, which counts *tuning*
/// failures, and it has to be: `consecutive_failures` is reset immediately
/// after a successful tune and `into_rx_mode`, before any samples are read. A
/// device that tunes perfectly well but cannot stream — a half-dead USB link,
/// a HackRF wedged after a bus reset — therefore reset the tuning counter on
/// every pass and retuned forever without ever reaching the "Giving up" path.
/// This counter is reset only when a packet is actually delivered, so it
/// measures the thing that matters: are samples reaching the caller?
const MAX_CONSECUTIVE_STREAM_FAILURES: u32 = 20;

/// Pause after a streaming failure before retuning.
///
/// The retune path has no natural delay, so without this a device that
/// consistently fails at `start_rx` spins the outer loop at full speed.
const STREAM_FAILURE_BACKOFF: Duration = Duration::from_millis(50);

/// Should the device be abandoned after this many consecutive streaming
/// failures? Extracted so the decision is testable without hardware.
fn should_abandon_device(consecutive_stream_failures: u32) -> bool {
    consecutive_stream_failures >= MAX_CONSECUTIVE_STREAM_FAILURES
}

// Invalid or driver-rejected requests must not kill a working scan.
fn accepted_override(requested: Option<f64>, rejected: Option<u64>) -> Option<f64> {
    requested.filter(|f| f.is_finite() && *f > 0.0 && rejected != Some(f.to_bits()))
}

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

/// Validate `SourceConfig` and clamp the requested sample rate to the
/// HackRF's USB-2.0 ceiling. Returns the clamped rate and whether
/// clamping occurred, so the caller can log it. Kept standalone (no
/// hardware access) so it's unit-testable ahead of `HackRfOne::new()`.
fn resolve_sample_rate(num_channels: usize, requested_hz: f64) -> Result<(f64, bool), SdrError> {
    if num_channels == 0 {
        return Err(SdrError::BadConfig(
            "SourceConfig.channels_hz must not be empty".into(),
        ));
    }
    if !requested_hz.is_finite() || requested_hz < 1.0 {
        return Err(SdrError::BadConfig(format!(
            "invalid sample rate {requested_hz} Hz"
        )));
    }
    // The driver programs an integer-Hz rate; report exactly that rate.
    let clamped = requested_hz.min(HACKRF_MAX_SAMPLE_RATE_HZ).floor();
    Ok((clamped, requested_hz > HACKRF_MAX_SAMPLE_RATE_HZ))
}

fn configure_radio(
    radio: &mut HackRfOne<hackrfone::UnknownMode>,
    rate: f64,
    lna: u16,
    vga: u16,
    amp: bool,
    bias: bool,
) -> Result<(), hackrfone::Error> {
    radio.set_sample_rate(rate as u32, 1)?;
    radio.set_lna_gain(lna)?;
    radio.set_vga_gain(vga)?;
    radio.set_amp_enable(amp)?;
    radio.set_antenna_enable(bias as u8)
}

impl SdrSource for HackRfSource {
    fn start(
        self: Box<Self>,
        config: SourceConfig,
        advice: Arc<dyn DwellAdvice>,
    ) -> Result<SdrHandle, SdrError> {
        let (sample_rate, was_clamped) =
            resolve_sample_rate(config.channels_hz.len(), config.sample_rate_hz)?;
        if was_clamped {
            info!(
                "[hackrf] Requested {:.2} MSPS exceeds the {:.0} MSPS USB-2.0 ceiling; clamping.",
                config.sample_rate_hz / 1e6,
                HACKRF_MAX_SAMPLE_RATE_HZ / 1e6
            );
        }

        if config
            .channels_hz
            .iter()
            .any(|f| !f.is_finite() || *f < 1.0 || *f >= u64::MAX as f64)
        {
            return Err(SdrError::BadConfig(
                "channel frequencies must be finite, positive integer-Hz representable values"
                    .into(),
            ));
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

        configure_radio(
            &mut radio,
            sample_rate,
            self.lna_gain,
            self.vga_gain,
            self.amp_enable,
            self.bias_tee,
        )
        .map_err(|e| SdrError::BadConfig(format!("HackRF configuration failed: {e:?}")))?;

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

        let (tx, receiver) = channel::bounded::<IqPacket>(64);
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_flag_thread = stop_flag.clone();
        let advice_thread = advice;
        let sample_rate_f32 = sample_rate as f32;

        // The driver reads at most 128 KiB of bytes (65,536 complex samples).
        // Allocate on demand, retaining at most 32 MiB in each queue.
        let (pool_tx, pool_rx) = channel::bounded::<Vec<Complex32>>(64);

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
                    // the single-channel case (`channels_hz.len() == 1`) the
                    // dwell deadline is ignored (see the inner loop), so the
                    // outer loop runs once and we stream continuously regardless
                    // of the configured dwell.
                    let mut device_opt = Some(radio); // Option<HackRfOne<UnknownMode>>
                    let mut channel_idx = 0usize;
                    let mut last_rejected_override = None;
                    let mut last_report = Instant::now();
                    let mut channel_switches = 0u64;
                    let mut consecutive_failures = 0;
                    let mut consecutive_sweep_failures = 0;
                    // Reset only when a packet is delivered — see
                    // MAX_CONSECUTIVE_STREAM_FAILURES.
                    let mut consecutive_stream_failures = 0u32;

                    'outer: loop {
                        if stop_flag_thread.load(Ordering::SeqCst) {
                            break;
                        }

                        // A device that tunes fine but never streams would
                        // otherwise retune forever, because the tuning counter
                        // below is reset on every successful tune.
                        if should_abandon_device(consecutive_stream_failures) {
                            tracing::error!(
                                "[hackrf] {} consecutive RX streaming failures with no samples \
                                 delivered. Giving up — is the HackRF still connected?",
                                consecutive_stream_failures
                            );
                            break 'outer;
                        }
                        if consecutive_stream_failures > 0 {
                            // Pace the retune; this path has no natural delay.
                            thread::sleep(STREAM_FAILURE_BACKOFF);
                        }

                        if consecutive_failures >= num_channels {
                            consecutive_sweep_failures += 1;
                            if consecutive_sweep_failures >= MAX_CONSECUTIVE_SWEEP_FAILURES {
                                tracing::error!(
                                    "[hackrf] All channels failed to tune for {} consecutive sweeps. Giving up — is the HackRF still connected?",
                                    consecutive_sweep_failures
                                );
                                break 'outer;
                            }
                            tracing::warn!(
                                "[hackrf] All channels failed consecutively. Sleeping for 500ms before retrying."
                            );
                            thread::sleep(Duration::from_millis(500));
                            consecutive_failures = 0;
                        }

                        if device_opt.is_none() {
                            let reopened = HackRfOne::new().and_then(|mut device| {
                                match configure_radio(
                                    &mut device,
                                    sample_rate,
                                    lna_gain,
                                    vga_gain,
                                    amp_enable,
                                    bias_tee,
                                ) {
                                    Ok(()) => Some(device),
                                    Err(e) => {
                                        tracing::warn!(
                                            "[hackrf] recovery configuration failed: {e:?}"
                                        );
                                        None
                                    }
                                }
                            });
                            if reopened.is_none() {
                                consecutive_failures += 1;
                                thread::sleep(Duration::from_millis(100));
                                continue;
                            }
                            device_opt = reopened;
                        }

                        let mut device = device_opt.take().unwrap();
                        // A live `/video` viewer wants a specific channel: park there and
                        // hold, bypassing the hop list entirely, until the override changes
                        // or clears. `channel_idx` isn't touched while parked, so hopping
                        // resumes exactly where it left off once the viewer disconnects.
                        let override_freq = accepted_override(
                            advice_thread.channel_override(),
                            last_rejected_override,
                        );
                        let current_freq_hz = override_freq.unwrap_or(channels_hz[channel_idx]);
                        let freq_key = freq_key_khz(current_freq_hz);
                        if let Err(e) = device.set_freq(current_freq_hz as u64) {
                            tracing::warn!(
                                "[hackrf] Failed to set frequency to {} Hz: {:?}. Skipping channel.",
                                current_freq_hz,
                                e
                            );
                            device_opt = Some(device);
                            if override_freq.is_some() {
                                last_rejected_override = Some(current_freq_hz.to_bits());
                                continue;
                            }
                            consecutive_failures += 1;
                            channel_idx = (channel_idx + 1) % num_channels;
                            continue;
                        }

                        let mut rx = match device.into_rx_mode() {
                            Ok(r) => r,
                            Err(e) => {
                                tracing::warn!(
                                    "[hackrf] into_rx_mode failed for {} Hz: {:?}. Attempting to re-open/recreate device.",
                                    current_freq_hz,
                                    e
                                );
                                consecutive_failures += 1;
                                thread::sleep(Duration::from_millis(100));
                                // Re-open and fully configure at the next outer iteration.
                                channel_idx = (channel_idx + 1) % num_channels;
                                continue;
                            }
                        };

                        // Reset consecutive failures on successful tune/start
                        consecutive_failures = 0;
                        consecutive_sweep_failures = 0;

                        let dwell_start = Instant::now();
                        // The loop yields the device back in `UnknownMode` (via
                        // `stop_rx`) so the next hop can retune it.
                        device_opt = Some(loop {
                            if stop_flag_thread.load(Ordering::SeqCst) {
                                break rx
                                    .stop_rx()
                                    .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                            }
                            let next_override = accepted_override(
                                advice_thread.channel_override(),
                                last_rejected_override,
                            );
                            if next_override != override_freq {
                                break rx
                                    .stop_rx()
                                    .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                            }
                            if override_freq.is_none() && num_channels > 1 {
                                // With a single channel there is nowhere to hop, so
                                // never tear the RX down on the dwell deadline —
                                // stream continuously instead. Otherwise a
                                // single-channel caller with a short `dwell_min`
                                // (e.g. a wideband channelizer) would stop + retune
                                // the radio every dwell period, punching periodic
                                // gaps into an otherwise continuous stream. Matches
                                // the Pluto backend. The dwell deadline only gates
                                // hopping, which needs `num_channels > 1`.
                                let latest_signal = advice_thread.latest_signal_at(freq_key);
                                let deadline =
                                    dwell_controller.deadline(dwell_start, latest_signal);
                                if Instant::now() >= deadline {
                                    break rx
                                        .stop_rx()
                                        .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                                }
                            }

                            match rx.rx() {
                                Ok(bytes) => {
                                    if bytes.len() < 2 {
                                        consecutive_stream_failures += 1;
                                        break rx
                                            .stop_rx()
                                            .map_err(|e| anyhow::anyhow!("stop_rx: {e:?}"))?;
                                    }
                                    // Interleaved signed 8-bit I, Q → Complex32 in
                                    // [-1, 1). `as_chunks::<2>()` drops a trailing
                                    // odd byte (never expected from the device).
                                    let mut samples = pool_rx
                                        .try_recv()
                                        .unwrap_or_else(|_| Vec::with_capacity(65536));
                                    samples.clear();
                                    samples.extend(bytes.as_chunks::<2>().0.iter().map(|c| {
                                        Complex32::new(i8_to_unit(c[0]), i8_to_unit(c[1]))
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
                                        // Samples are reaching the caller, which
                                        // is the only proof the device works.
                                        consecutive_stream_failures = 0;
                                    }
                                }
                                Err(e) => {
                                    // A transient USB read error ends this dwell;
                                    // the outer loop retunes and re-enters RX.
                                    consecutive_stream_failures += 1;
                                    tracing::warn!(
                                        "[hackrf] rx error ({} consecutive): {e:?}",
                                        consecutive_stream_failures
                                    );
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
                        });

                        // Parking on a live-view override isn't a hop step — don't advance
                        // past it, so hopping resumes at the same channel it would have
                        // once the override clears.
                        if override_freq.is_none() {
                            channel_idx = (channel_idx + 1) % num_channels;
                            channel_switches += 1;
                        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_sample_rate_rejects_empty_channels() {
        let err = resolve_sample_rate(0, 10_000_000.0).unwrap_err();
        assert!(matches!(err, SdrError::BadConfig(_)));
    }

    #[test]
    fn resolve_sample_rate_rejects_non_positive_rate() {
        assert!(resolve_sample_rate(1, 0.0).is_err());
        assert!(resolve_sample_rate(1, -1.0).is_err());
    }

    #[test]
    fn resolve_sample_rate_passes_through_under_ceiling() {
        let (rate, clamped) = resolve_sample_rate(1, 10_000_000.0).unwrap();
        assert_eq!(rate, 10_000_000.0);
        assert!(!clamped);
    }

    #[test]
    fn resolve_sample_rate_clamps_above_ceiling() {
        let (rate, clamped) = resolve_sample_rate(1, 40_000_000.0).unwrap();
        assert_eq!(rate, HACKRF_MAX_SAMPLE_RATE_HZ);
        assert!(clamped);
    }
}

#[cfg(test)]
mod sample_scaling_tests {
    use super::i8_to_unit;

    /// The ADC's extremes must land inside the documented `[-1, 1)` range.
    /// Dividing by 127 put full-scale negative at -1.008, which any signal
    /// strong enough to hit the rail would produce.
    #[test]
    fn full_scale_samples_stay_in_the_documented_range() {
        assert_eq!(i8_to_unit(0x80), -1.0, "-128 is exactly -1.0");
        assert!(i8_to_unit(0x7F) < 1.0, "+127 must stay below +1.0");
        for raw in 0u16..=255 {
            let v = i8_to_unit(raw as u8);
            assert!(
                (-1.0..1.0).contains(&v),
                "raw {raw} scaled to {v}, outside [-1, 1)"
            );
        }
    }

    #[test]
    fn zero_and_sign_are_preserved() {
        assert_eq!(i8_to_unit(0), 0.0);
        assert!(i8_to_unit(0x01) > 0.0, "+1 is positive");
        assert!(i8_to_unit(0xFF) < 0.0, "-1 (0xFF) is negative");
        // Symmetric magnitudes either side of zero.
        assert_eq!(i8_to_unit(0x40), 0.5, "+64 is half scale");
        assert_eq!(i8_to_unit(0xC0), -0.5, "-64 is minus half scale");
    }
}

#[cfg(test)]
mod stream_failure_bounding_tests {
    use super::*;

    /// Transient streaming errors must not abandon a working device.
    #[test]
    fn transient_stream_errors_are_tolerated() {
        for n in 0..MAX_CONSECUTIVE_STREAM_FAILURES {
            assert!(!should_abandon_device(n));
        }
    }

    /// A device that never delivers samples must be given up on.
    ///
    /// The specific failure this guards: `consecutive_failures` is reset right
    /// after a successful tune and `into_rx_mode`, before any samples are read,
    /// so a HackRF that tunes but cannot stream reset that counter on every
    /// pass and retuned forever without ever reaching the "Giving up" path.
    /// This counter is reset only when a packet is actually delivered.
    #[test]
    fn a_device_that_never_streams_is_abandoned() {
        assert!(should_abandon_device(MAX_CONSECUTIVE_STREAM_FAILURES));
        assert!(should_abandon_device(MAX_CONSECUTIVE_STREAM_FAILURES + 50));
    }

    /// Giving up must happen in a bounded, short time.
    #[test]
    fn giving_up_is_bounded_in_wall_clock_time() {
        let worst = STREAM_FAILURE_BACKOFF * MAX_CONSECUTIVE_STREAM_FAILURES;
        assert!(
            worst <= Duration::from_secs(2),
            "should abandon a dead device in seconds, not {worst:?}"
        );
    }
}

#[cfg(test)]
mod review_regressions {
    use super::*;
    #[test]
    fn rate_matches_the_integer_programmed_into_the_driver() {
        assert_eq!(
            resolve_sample_rate(1, 10_000_000.75).unwrap(),
            (10_000_000.0, false)
        );
        for rate in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.5] {
            assert!(resolve_sample_rate(1, rate).is_err());
        }
    }
    #[test]
    fn override_changes_are_detected_and_rejected_requests_are_ignored() {
        assert_ne!(None, accepted_override(Some(915e6), None));
        assert_ne!(Some(915e6), accepted_override(Some(433e6), None));
        assert_ne!(Some(915e6), accepted_override(None, None));
        for f in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
            assert_eq!(None, accepted_override(Some(f), None));
        }
        assert_eq!(
            None,
            accepted_override(Some(915e6), Some(915e6_f64.to_bits()))
        );
    }
}
