# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_openview.py - Arduino counterpart: 06.OpenView
#
# Streams sEMG, envelope and %MVC into the Protocentral OpenView desktop app over USB
# serial, with the same packets as the sketch. MicroPython only (it needs
# sys.stdout.buffer).
#
# In OpenView 3.4 or later: Connect -> USB -> select "easyEMG Breakout". Close Thonny or
# mpremote first: only one program can hold the serial port. Start this file as the
# board's main.py, so it runs at power-up without a REPL attached.
#
# ---- Packet format (identical to 06.OpenView) ----------------------------------
#
#   0x0A 0xFA <len=0x08> 0x00 <type=0x02> <payload, 8 bytes> 0x00 0x0B
#
#   [0-1]  sEMG          int16 LE, signed counts (+/-2048)
#   [2-3]  RMS envelope  int16 LE, counts, smoothed
#   [4-5]  %MVC          int16 LE, 0 until an MVC has been recorded
#   [6]    MVC step      0 no MVC yet, 1 relax, 2 contract now, 3 too weak, 4 MVC set
#   [7]    reserved, 0
#
# ---- What differs from the Arduino sketch -----------------------------------------
#
# Packets go out at 500 Hz, as OpenView's easyEMG profile expects. The Arduino sketch
# samples at 1000 SPS and averages 2 samples per packet. Interpreted Python can't be
# relied on to hold that, so this samples at ADC_RATE_HZ = 500, one sample per packet.
# That puts Nyquist at 250 Hz, below the front end's 310 Hz corner, so the top of the
# band folds down, and set_mains_notch() builds 4 notches (50-200 Hz) rather than 9. If
# your board holds 1000 SPS with missed samples at 0 (run easyemg_diagnostics.py), set
# ADC_RATE_HZ = 1000 and this matches the Arduino sketch.
#
# The envelope window is 128 ms, as in the sketch: 64 samples at 500 SPS.
#
# ---- Recording the MVC ------------------------------------------------------------
#
# Runs after reset (RECORD_MVC) and again whenever the host sends 'm' (OpenView's Record
# MVC button). Relax for 2 s, then contract as hard as you can for 5 s. An MVC under
# 100 uV or under 5x the resting level is refused; with MVC_AUTO_RAISE a stronger later
# contraction becomes the new MVC. See MvcRecorder in the driver. The step goes out in
# byte 6 of every packet; LED_PIN, if set, lights while you should contract.
#
# The 'm' command needs select.poll on sys.stdin, which the ESP32 and RP2040 ports have.
# Where it's missing, the MVC step only runs at reset.
#
# Bytes go out through sys.stdout.buffer, which doesn't turn 0x0A into CR LF. Nothing
# else may print while this runs, or the stream breaks.
#
# Download OpenView: https://github.com/Protocentral/protocentral_openview
#
# A mains-powered host gives no EMG at all.
# For research and education only - not a medical device. With electrodes on a person,
# power the host from a battery or connect through an easyISO isolated Qwiic adapter.

import sys

try:
    from machine import Pin, I2C
    # Defaults for the Adafruit QT Py ESP32-C3 (SCL=6, SDA=5). Change for your board.
    i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)
except ImportError:                     # Raspberry Pi: writes the same bytes to stdout
    Pin = None
    from protocentral_tla20xx_linux import I2C
    i2c = I2C(bus=1)

from protocentral_easyemg import (
    EasyEMG, MvcRecorder, EASYEMG_MAX_ENVELOPE_WINDOW, sleep_ms,
    EASYEMG_MVC_SETTLE_MS, EASYEMG_MVC_RECORD_MS, EASYEMG_MVC_MIN_UV,
    EASYEMG_MVC_MIN_REST_RATIO,
)

# OpenView packet constants
CES_CMDIF_PKT_START_1 = 0x0A
CES_CMDIF_PKT_START_2 = 0xFA
CES_CMDIF_TYPE_DATA = 0x02
CES_CMDIF_PKT_STOP = 0x0B
DATA_LEN = 0x08

# MVC, as in 06.OpenView
RECORD_MVC = True                       # run the MVC step after reset
MVC_AUTO_RAISE = True                   # a stronger contraction becomes the new MVC
MVC_COMMAND = "m"                       # host command that runs the step again

# Mains notch on every harmonic below 0.45x the ADC rate. MAINS_HZ is 50 or 60.
USE_NOTCH = True
MAINS_HZ = 50.0
NOTCH_FUND_BW_HZ = 6.0
NOTCH_BW_HZ = 2.0

HIGH_PASS_HZ = 20.0                     # 0 to disable

PACKET_RATE_HZ = 500                    # OpenView's easyEMG profile
ADC_RATE_HZ = 500                       # a multiple of PACKET_RATE_HZ; 1000 = Arduino
SAMPLES_PER_PACKET = ADC_RATE_HZ // PACKET_RATE_HZ

ENVELOPE_SMOOTH_MS = 120.0              # display smoothing, as in 06.OpenView
ENV_ALPHA = (1000.0 / PACKET_RATE_HZ) / ENVELOPE_SMOOTH_MS

LED_PIN = None                          # e.g. "LED" on a Pico, 2 on many ESP32 boards


def _int16(v):
    return -32768 if v < -32768 else (32767 if v > 32767 else v)


def _put16(buf, i, v):
    buf[i] = v & 0xFF
    buf[i + 1] = (v >> 8) & 0xFF


emg = EasyEMG(i2c)
if not emg.begin():
    # No text: the host expects binary packets.
    while True:
        sleep_ms(500)

emg.calibrate_baseline()                # relax through this, and through any reset
emg.set_envelope_window(min(EASYEMG_MAX_ENVELOPE_WINDOW, (128 * ADC_RATE_HZ) // 1000))
emg.set_sample_rate(ADC_RATE_HZ)
if USE_NOTCH:
    emg.set_mains_notch(MAINS_HZ, ADC_RATE_HZ, NOTCH_BW_HZ, NOTCH_FUND_BW_HZ)
emg.set_high_pass(HIGH_PASS_HZ, ADC_RATE_HZ)    # leaves it off when 0
uv_per_count = emg.microvolts_per_count()

mvc = MvcRecorder(emg, PACKET_RATE_HZ, EASYEMG_MVC_SETTLE_MS, EASYEMG_MVC_RECORD_MS,
                  EASYEMG_MVC_MIN_UV, EASYEMG_MVC_MIN_REST_RATIO, MVC_AUTO_RAISE)
if RECORD_MVC:
    mvc.start()

led = Pin(LED_PIN, Pin.OUT) if (Pin is not None and LED_PIN is not None) else None

# Host commands. select.poll on stdin exists on the ESP32 and RP2040 ports and on Linux.
poller = None
try:
    import select
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
except (ImportError, AttributeError, OSError, TypeError, ValueError):
    poller = None

out = sys.stdout.buffer
flush = out.flush if sys.implementation.name != "micropython" else None

packet = bytearray(5 + DATA_LEN + 2)
packet[0] = CES_CMDIF_PKT_START_1
packet[1] = CES_CMDIF_PKT_START_2
packet[2] = DATA_LEN
packet[3] = 0
packet[4] = CES_CMDIF_TYPE_DATA
packet[5 + DATA_LEN] = 0
packet[6 + DATA_LEN] = CES_CMDIF_PKT_STOP

env_smooth = 0.0
raw_accum = 0
raw_count = 0

while True:
    if not emg.update():                # one sample on the paced clock
        continue
    raw_accum += emg.last_raw()
    raw_count += 1
    if raw_count < SAMPLES_PER_PACKET:
        continue

    raw = int(raw_accum / raw_count)    # truncate toward zero, as C does
    raw_accum = 0
    raw_count = 0

    env_smooth += (emg.last_envelope_raw() - env_smooth) * ENV_ALPHA

    if poller is not None and poller.poll(0):
        c = sys.stdin.read(1)
        if c == MVC_COMMAND or c == MVC_COMMAND.upper():
            mvc.start()

    env_uv = env_smooth * uv_per_count
    step = mvc.update(env_uv)
    if led is not None:
        led.value(1 if mvc.led() else 0)

    _put16(packet, 5, _int16(raw))
    _put16(packet, 7, _int16(int(env_smooth + 0.5)))
    _put16(packet, 9, _int16(mvc.percent(env_uv)))
    packet[11] = step
    packet[12] = 0
    out.write(packet)
    if flush:
        flush()
