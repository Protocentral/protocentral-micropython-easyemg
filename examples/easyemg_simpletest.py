# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_simpletest.py - Arduino counterpart: 01.BasicReading
#
# The "hello world": prints the raw sample and RMS envelope on a paced 500 SPS clock.
# Runs on MicroPython; easyemg_raspberrypi.py is the Raspberry Pi version. Requires
# protocentral_tla20xx on the board (installed automatically as a mip dependency).
#
# For research and education only - not a medical device. With electrodes on a person,
# power the host from a battery or connect through an easyISO isolated Qwiic adapter.

from machine import Pin, I2C
from protocentral_easyemg import EasyEMG

PRINT_EVERY = 50                # samples: 10 lines a second at 500 SPS

# Defaults for the Adafruit QT Py ESP32-C3 (SCL=6, SDA=5). Change for your board.
i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)

emg = EasyEMG(i2c)              # ships at 0x48
if not emg.begin():
    raise OSError("easyEMG not found - check wiring and I2C address (0x48)")

emg.calibrate_baseline()        # muscle relaxed, electrodes on
# emg.set_mains_notch(50.0, emg.sample_rate_hz())   # if room hum swamps the signal

n = 0
while True:
    if emg.update():
        n += 1
        if n >= PRINT_EVERY:
            n = 0
            print("raw {:8.1f} uV   envelope {:7.1f} uV RMS   missed {}".format(
                emg.last_microvolts(), emg.last_envelope(), emg.missed_samples()))
