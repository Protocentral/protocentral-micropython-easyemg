# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_calibration.py - Arduino counterpart: 03.Calibration
#
# Zeroes the resting offset and measures your noise floor. Runs on MicroPython and on a
# Raspberry Pi.
#
# Do this every session, per subject. The board reads differentially against its own
# buffered mid-rail, so the resting count is already near zero, but amplifier offset leaves
# a few counts behind. calibrate_baseline() averages the resting signal and subtracts it.
#
# The front-end gain (x51.7) is set by the onboard resistors, so there is nothing to
# calibrate there. The script prints it for reference.
#
# With the muscle relaxed, the resting envelope should sit close to the board's resolution
# (a few uV). If it is much higher, suspect mains pickup, long electrode leads or poor skin
# prep, and run easyemg_diagnostics.py.
#
# For research and education only - not a medical device. With electrodes on a person,
# power the host from a battery or connect through an easyISO isolated Qwiic adapter.

try:
    from machine import Pin, I2C
    # Defaults for the Adafruit QT Py ESP32-C3 (SCL=6, SDA=5). Change for your board.
    i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)
except ImportError:                     # Raspberry Pi
    from protocentral_tla20xx_linux import I2C
    i2c = I2C(bus=1)

from protocentral_easyemg import EasyEMG, sleep_ms

BASELINE_SAMPLES = 512
ENVELOPE_WINDOW = 64                    # samples: 128 ms at 500 SPS
NOISE_SECONDS = 2

emg = EasyEMG(i2c)                      # ships at 0x48
if not emg.begin():
    raise OSError("easyEMG not found - check wiring and I2C address (0x48)")

print("=== easyEMG calibration ===")
print("Front-end gain        : x{:.2f}".format(emg.get_gain()))
print("ADC full scale        : +/-{:.3f} V".format(emg.full_scale_volts()))
print("Resolution at the skin: {:.2f} uV per count".format(emg.microvolts_per_count()))
print()

print("Relax the muscle completely. Measuring baseline in 3 seconds...")
sleep_ms(3000)
emg.calibrate_baseline(BASELINE_SAMPLES)
print("Baseline offset: {} counts".format(emg.get_baseline()))
print()

# Paced on the driver's clock, so the envelope covers a known time span.
print("Measuring resting noise floor ({} s)...".format(NOISE_SECONDS))
emg.set_envelope_window(ENVELOPE_WINDOW)
n = 0
target = NOISE_SECONDS * emg.sample_rate_hz()
while n < target:
    if emg.update():
        n += 1
print("Resting envelope: {:.2f} uV RMS".format(emg.last_envelope()))
if emg.missed_samples():
    print("({} samples missed - the host couldn't hold {} SPS)".format(
        emg.missed_samples(), emg.sample_rate_hz()))
print()

print("Copy this line into your own script, after emg.begin():")
print("  emg.set_baseline({})".format(emg.get_baseline()))
