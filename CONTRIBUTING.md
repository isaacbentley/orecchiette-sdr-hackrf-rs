# Contributing to orecchiette-sdr-hackrf-rs

First off, thank you for considering contributing! This crate
implements the `SdrSource` trait (from `orecchiette-sdr-source-rs`)
for the Great Scott Gadgets HackRF One via the pure-Rust `hackrfone`
crate.

## Quick Start

```bash
git clone https://github.com/isaacbentley/orecchiette-sdr-hackrf-rs.git
cd orecchiette-sdr-hackrf-rs

# No system C libraries needed — hackrfone talks USB via nusb.
cargo test
cargo clippy --all-features --all-targets -- -D warnings
cargo fmt --all --check
cargo deny check
```

## Testing Hardware Changes

Most of this crate's logic (gain/bias-tee configuration, dwell/hop
pacing, IQ scaling) can be unit-tested without a device attached. If
your change affects the actual capture loop, please test against
real HackRF One hardware before opening a PR.

## Code Style

We use standard `rustfmt` defaults. Please run `cargo fmt --all` before pushing.

Clippy is run with `-D warnings` in CI. If a lint is genuinely wrong for the situation, allow it with a `// ALLOW:` justification comment explaining why.

## Pull Requests

- **Commit messages:** Describe *why* the change is needed and *what* it changes.
- **Templates:** Please fill out the Pull Request template when opening a PR.

## License

By contributing, you agree your contributions will be licensed under GPL-3.0-or-later, the same as the rest of the project.
