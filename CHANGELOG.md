# Changelog

All notable changes to this driver are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-07

First public release. A port of the
[Protocentral easyEMG Arduino library](https://github.com/Protocentral/protocentral_easyemg_arduino)
1.1.0 for MicroPython and Raspberry Pi.

### Added
- The Arduino API in snake_case: raw sEMG in µV at the electrodes, sliding-window RMS
  envelope, baseline zeroing, activation threshold and %MVC.
- `set_mains_notch()`, a notch on every mains harmonic up to 0.45× the sample rate
  (6 Hz wide at the fundamental, 2 Hz on the harmonics), the single `set_notch()`, and
  a 2nd-order digital high-pass. All off by default.
- Paced sampling: `update()` on a fixed clock (500 SPS by default), with
  `missed_samples()` and `reset_clock()`.
- `MvcRecorder`, the MVC recording step from the Arduino `06.OpenView` sketch.
- Low-level ADC passthroughs: `set_range()`, `set_data_rate()`, `set_mode()`,
  `set_mux()` and `read_single_shot()`.
- Python-only extras: `set_gain()`, `microvolts_per_count()`, `last_envelope_raw()`
  and `.adc`.
- An example for every Arduino example: `easyemg_simpletest.py` and
  `easyemg_raspberrypi.py` (`01`), `easyemg_plotter.py` (`02`),
  `easyemg_calibration.py` (`03`), `easyemg_envelope.py` (`04`),
  `easyemg_activation_led.py` (`05`), `easyemg_openview.py` (`06`, the same packets for
  OpenView's easyEMG Breakout profile) and `easyemg_diagnostics.py` (`07`).
- Mocked-bus tests, including each example run end to end, and an `mpy-cross` compile
  in CI, on every push and before every PyPI publish.

### Differences from the Arduino library
- The driver paces sampling itself, at 500 SPS by default, where the Arduino sketches
  pace at 1000 SPS. `easyemg_openview.py` therefore sends one sample per packet rather
  than averaging two, and its harmonic notch covers 50–200 Hz.

Validated with the mocked-bus tests. On an ESP32-S3 (MicroPython 1.29) the driver holds
500 SPS through the full filter chain with no missed samples. Further bench results
will be added to the README as they're measured.
