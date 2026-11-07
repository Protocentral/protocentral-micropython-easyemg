# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_raspberrypi.py - Arduino counterpart: 01.BasicReading
#
# The basic reading on a Raspberry Pi. Reuses the TLA20xx smbus2 shim.
#
# Setup:  sudo raspi-config (enable I2C); pip install protocentral-easyemg
#         i2cdetect -y 1   ->  should show 0x48
#
# A mains-powered Pi gives no EMG at all.
# For research and education only - not a medical device. With electrodes on a person,
# power the host from a battery or connect through an easyISO isolated Qwiic adapter.

from protocentral_tla20xx_linux import I2C   # shim ships with protocentral-tla20xx
from protocentral_easyemg import EasyEMG

PRINT_EVERY = 50                # samples: 10 lines a second at 500 SPS

with I2C(bus=1) as i2c:
    emg = EasyEMG(i2c)
    if not emg.begin():
        raise OSError("easyEMG not found - check wiring and I2C address (0x48)")
    emg.calibrate_baseline()    # muscle relaxed, electrodes on
    n = 0
    while True:
        if emg.update():
            n += 1
            if n >= PRINT_EVERY:
                n = 0
                print("raw {:8.1f} uV   envelope {:7.1f} uV RMS   missed {}".format(
                    emg.last_microvolts(), emg.last_envelope(), emg.missed_samples()))
