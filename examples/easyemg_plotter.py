# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_plotter.py - Arduino counterpart: 02.SerialPlotter
#
# Raw sEMG and its RMS envelope for a plotter, such as Thonny's View -> Plotter. Runs on
# MicroPython and on a Raspberry Pi.
#
# Both traces are in uV at the electrodes. Prints every PLOT_EVERY-th sample, because a
# plotter can't keep up with 500 lines a second; the envelope still uses every sample.
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

from protocentral_easyemg import EasyEMG

PLOT_EVERY = 5                          # 100 points a second at 500 SPS
MAINS_HZ = 50.0                         # 50 or 60

emg = EasyEMG(i2c)                      # ships at 0x48
if not emg.begin():
    raise OSError("easyEMG not found - check wiring and I2C address (0x48)")

emg.calibrate_baseline()                # muscle relaxed, electrodes on
emg.set_mains_notch(MAINS_HZ, emg.sample_rate_hz())
emg.set_high_pass(20.0, emg.sample_rate_hz())

n = 0
while True:
    if emg.update():
        n += 1
        if n >= PLOT_EVERY:
            n = 0
            print("sEMG: {:.0f} Envelope: {:.0f}".format(
                emg.last_microvolts(), emg.last_envelope()))
