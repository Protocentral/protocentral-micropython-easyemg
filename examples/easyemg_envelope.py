# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_envelope.py - Arduino counterpart: 04.EnvelopeRMS
#
# Prints the RMS envelope and %MVC. Runs on MicroPython and on a Raspberry Pi.
#
# Surface EMG is a zero-mean AC signal, so averaging it gives you roughly nothing. The
# number that means "how hard is this muscle working" is the RMS over a sliding window:
# the envelope.
#
# This prints the envelope in microvolts and, once you have recorded a maximum voluntary
# contraction (MVC), as a percentage of it. %MVC is how sports science and rehab compare
# muscle effort across subjects and sessions, because raw microvolts depend on electrode
# placement, skin and body composition.
#
# The window is in SAMPLES. At the driver's default 500 SPS, 64 samples is 128 ms;
# envelope_window_ms() converts it. 100 ms is the usual starting point for sEMG.
#
# This takes the MVC as a simple peak, as the sketch does. MvcRecorder in the driver
# (used by easyemg_openview.py) also measures your rest level and refuses a weak MVC.
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

from protocentral_easyemg import (
    EasyEMG, EASYEMG_MVC_MIN_UV, ticks_us, ticks_diff, sleep_ms,
)

ENVELOPE_WINDOW = 64                    # samples
MVC_SECONDS = 5
PRINT_HZ = 20                           # fast enough to feel live, slow enough to read

emg = EasyEMG(i2c)                      # ships at 0x48
if not emg.begin():
    raise OSError("easyEMG not found - check wiring and I2C address (0x48)")

fs = emg.sample_rate_hz()

print("Relax the muscle. Zeroing...")
sleep_ms(1000)
emg.calibrate_baseline()
emg.set_envelope_window(ENVELOPE_WINDOW)
# emg.set_mains_notch(50.0, fs)         # if room hum swamps the signal
print("Envelope window: {} samples = {:.0f} ms at {} SPS".format(
    ENVELOPE_WINDOW, emg.envelope_window_ms(fs), fs))

# Record a maximum voluntary contraction so readings can be normalised.
print()
print("Contract the muscle AS HARD AS YOU CAN for the next {} seconds.".format(MVC_SECONDS))
sleep_ms(2000)
print("GO!")

emg.reset_clock()
peak = 0.0
n = 0
while n < MVC_SECONDS * fs:
    if emg.update():
        n += 1
        e = emg.last_envelope()
        if e > peak:
            peak = e

emg.set_mvc(peak)
print("MVC recorded: {:.1f} uV RMS".format(peak))
if peak < EASYEMG_MVC_MIN_UV:
    print("That's weak for an MVC (under {:.0f} uV), so %MVC will read high.".format(
        EASYEMG_MVC_MIN_UV))
    print("Check the electrodes and REF, then reset and try again.")
print("Relax. Streaming...")
print()
print("Envelope(uV)\t%MVC\tRate(SPS)\tMissed")

every = max(1, fs // PRINT_HZ)
n = 0
count = 0
rate = 0.0
t0 = ticks_us()
emg.reset_clock()
while True:
    if emg.update():
        count += 1
        n += 1
        if n >= every:
            n = 0
            elapsed = ticks_diff(ticks_us(), t0)
            if elapsed >= 1000000:
                rate = count * 1000000.0 / elapsed
                count = 0
                t0 = ticks_us()
            print("{:.1f}\t{:.1f}\t{:.0f}\t{}".format(
                emg.last_envelope(), emg.read_percent_mvc(), rate, emg.missed_samples()))
