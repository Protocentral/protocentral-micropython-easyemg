# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_diagnostics.py - Arduino counterpart: 07.Diagnostics
#
# Run this when something is wrong and you want numbers. Runs on MicroPython and on a
# Raspberry Pi.
#
#   1. Is the ADC on the bus, and at which address?
#   2. Did the configuration take? (CONFIG read back, expect 0x0AC3)
#   3. Does the PGA range field really change? (fixed range = wrong scaling)
#   4. Is the conversion register changing at all, or stuck?
#   5. Sample rate, then a live table: noise floor, hum or muscle.
#
# A stuck ADC makes everything read exactly zero, because calibrate_baseline() averages
# the constant and subtracts it. Step 4 tells that apart from a quiet board.
#
# Every measurement is PACED on the driver's update() clock (500 SPS by default), and the
# notches are built from that exact rate. Live table columns, uV at the electrodes:
#
#   peak      largest |count| in the window      DC       window mean, counts
#   ac_raw    AC RMS, no notch                   ac_notch AC RMS, every mains harmonic notched
#   hum%      share of ac_raw the notches remove missed   samples the clock missed (should be 0)
#
# Commands (type the letter, then Enter):
#   s   shorted-input test: E1 + E2 + REF bridged through 10 kOhm, no body in the loop
#   h   harmonic breakdown: how much each mains harmonic contributes
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

import math
import sys

from protocentral_easyemg import (
    EasyEMG, EASYEMG_I2C_ADDR_DEFAULT, EASYEMG_I2C_ADDR_ALT, ticks_us, ticks_diff, sleep_ms,
)

MAINS_HZ = 50.0                         # 50 or 60
NOTCH_FUND_BW_HZ = 6.0                  # live table: as easyemg_openview.py
NOTCH_BW_HZ = 2.0
BREAKDOWN_BW_HZ = 4.0                   # 'h': one notch per harmonic, a little wider
WINDOW_N = 300                          # samples per reported figure
SETTLE_S = 0.8                          # notch settling before measuring (800 at 1 kSPS)
EXPECTED_CONFIG = 0x0AC3

try:
    import select
    _poller = select.poll()
    _poller.register(sys.stdin, select.POLLIN)
except (ImportError, AttributeError, OSError, TypeError, ValueError):
    _poller = None


def command():
    """One character from the host, or None. Needs select.poll on stdin."""
    if _poller is not None and _poller.poll(0):
        return sys.stdin.read(1)
    return None


print()
print("========== easyEMG diagnostics ==========")
print()

# ---- 1. I2C scan
print("--- 1. I2C scan")
found = None
for addr in i2c.scan():
    label = ""
    if addr == EASYEMG_I2C_ADDR_DEFAULT:
        label = "  <- easyEMG, JP1 default"
        found = addr
    elif addr == EASYEMG_I2C_ADDR_ALT:
        label = "  <- easyEMG, JP1 alternate (or an easyECG)"
        if found is None:
            found = addr
    print("   device at 0x{:02X}{}".format(addr, label))
if found is None:
    print("   NOTHING at 0x48/0x49. Check 3V3 and GND, then SDA/SCL.")
    raise SystemExit
print()

emg = EasyEMG(i2c, found)

# ---- 2. CONFIG read-back
print("--- 2. CONFIG register read-back")
emg.adc._write16(0x01, EXPECTED_CONFIG)
sleep_ms(10)
got = emg.read_config()
print("   wrote     0x{:04X}".format(EXPECTED_CONFIG))
print("   read back 0x{:04X}".format(got))
if (got & 0x7FFF) == EXPECTED_CONFIG:   # bit 15 (OS) reads 1 when idle
    print("   MATCH - configuration took.")
else:
    diff = (got ^ EXPECTED_CONFIG) & 0x7FFF
    print("   MISMATCH - the part did not accept every field:")
    if diff & 0x7000:
        print("     MUX  differs (expected on a part with no input mux)")
    if diff & 0x0E00:
        print("     PGA  differs  <- SIGNIFICANT, see step 3")
    if diff & 0x0100:
        print("     MODE differs  <- SIGNIFICANT, may be stuck single-shot")
    if diff & 0x00E0:
        print("     DR   differs")
    if diff & 0x001F:
        print("     COMP differs (harmless, no comparator on this part)")
print()

# ---- 3. PGA field
print("--- 3. Does the PGA field exist?")
names = ("6.144", "4.096", "2.048", "1.024", "0.512", "0.256")
seen = []
for pga in range(6):
    emg.adc._write16(0x01, (pga << 9) | 0x00C3)
    sleep_ms(10)
    field = (emg.read_config() >> 9) & 0x07
    print("   +/-{} V -> field reads {}  {}".format(
        names[pga], field, "ok" if field == pga else "NOT ACCEPTED"))
    if field not in seen:
        seen.append(field)
print("   VERDICT: " + ("PGA field is fixed. This is not a TLA20xx."
                        if len(seen) <= 1 else "PGA is programmable, as expected."))
emg.begin()                             # back to 0x0AC3
print()

# ---- 4. Conversion register activity
print("--- 4. Is the conversion register changing?")
lo = 32767
hi = -32768
total = 0
for _ in range(200):
    v = emg.read_raw_uncorrected()
    lo = min(lo, v)
    hi = max(hi, v)
    total += v
    sleep_ms(2)
print("   min {}   max {}   mean {:.1f}   spread {}".format(lo, hi, total / 200, hi - lo))
if lo == hi:
    print("   STUCK - every sample identical. The ADC is not converting, and every")
    print("   reading will be exactly zero after calibrate_baseline().")
else:
    print("   ALIVE - the conversion register is moving.")
print()

emg.calibrate_baseline()
rate = emg.sample_rate_hz()
settle_n = int(SETTLE_S * rate)


def timed_read():
    while not emg.update():
        pass
    return emg.last_raw()


def window_ac_uv():
    """AC RMS about the window's own mean, uV; also peak |count| and the mean."""
    mn = 32767
    mx = -32768
    s = 0.0
    s2 = 0.0
    for _ in range(WINDOW_N):
        v = timed_read()
        if v < mn:
            mn = v
        if v > mx:
            mx = v
        s += v
        s2 += v * v
    mean = s / WINDOW_N
    var = max(0.0, s2 / WINDOW_N - mean * mean)
    return emg.microvolts_per_count() * math.sqrt(var), max(mx, -mn), mean


def measure_raw():
    emg.disable_notch()
    return window_ac_uv()


def settle():
    for _ in range(settle_n):
        timed_read()


def measure_with_notch(hz):
    # Q chosen so the -3 dB width is BREAKDOWN_BW_HZ at every frequency.
    w0 = 2.0 * math.pi * hz / rate
    q = math.sin(w0) / (2.0 * math.tan(math.pi * BREAKDOWN_BW_HZ / rate))
    emg.set_notch(hz, rate, q)
    settle()
    uv = window_ac_uv()[0]
    emg.disable_notch()
    return uv


def measure_with_mains_notch():
    n = emg.set_mains_notch(MAINS_HZ, rate, NOTCH_BW_HZ, NOTCH_FUND_BW_HZ)
    settle()
    uv = window_ac_uv()[0]
    emg.disable_notch()
    return uv, n


def table_header():
    print("   peak(cts)  DC(cts)  ac_raw(uV)  ac_notch(uV)  hum%  missed  verdict")


# ---- 5. Sample rate and live signal
print("--- 5. Live signal")
t0 = ticks_us()
n = 0
while ticks_diff(ticks_us(), t0) < 1000000:
    emg.read_raw_uncorrected()
    n += 1
print("   bare ADC reads:     {} per second".format(n))

# The whole per-sample chain the examples run: envelope, notches, high-pass.
sections = emg.set_mains_notch(MAINS_HZ, rate, NOTCH_BW_HZ, NOTCH_FUND_BW_HZ)
emg.set_high_pass(20.0, rate)
t0 = ticks_us()
n = 0
while ticks_diff(ticks_us(), t0) < 1000000:
    emg.read_envelope_raw()
    n += 1
emg.disable_high_pass()
emg.disable_notch()
print("   full filter chain:  {} per second    paced at: {} SPS".format(n, rate))
if n < rate:
    print("   WARNING: this host can't sustain the paced rate. Lower it with")
    print("   emg.set_sample_rate(), or the notches will be off-tune.")
if sections:
    print("   ac_notch: {} notches on {:.0f}-{:.0f} Hz ({:.0f} Hz wide at {:.0f} Hz, "
          "{:.0f} Hz on the rest)".format(sections, MAINS_HZ, MAINS_HZ * sections,
                                          NOTCH_FUND_BW_HZ, MAINS_HZ, NOTCH_BW_HZ))
else:
    print("   SAMPLE RATE TOO LOW to build a notch.")
print()
print("   Full scale is +/-2048 counts. 1 count = 2.42 uV at the electrodes.")
print("   Reference: relaxed muscle ~5 uV, light contraction ~100 uV,")
print("   strong contraction ~1000 uV RMS.")
print()
print("   >>> 's' + Enter = shorted-input test <<<")
print("   >>> 'h' + Enter = harmonic breakdown  <<<")
print()
table_header()


def harmonic_breakdown():
    print()
    print("--- HARMONIC BREAKDOWN")
    print("   Stay relaxed: this takes a while.")
    emg.reset_clock()
    base, pk, dc = measure_raw()
    print("   unfiltered     : {:.1f} uV RMS   (peak {} cts, DC {:.0f} cts)".format(base, pk, dc))
    hz = MAINS_HZ
    while hz <= 0.45 * rate:
        uv = measure_with_notch(hz)
        pct = max(0.0, 100.0 * (base - uv) / base) if base > 0.01 else 0.0
        print("   notch {:3.0f} Hz   : {:.1f} uV RMS   removes {:.0f}%".format(hz, uv, pct))
        hz += MAINS_HZ
    uv, n = measure_with_mains_notch()
    pct = max(0.0, 100.0 * (base - uv) / base) if base > 0.01 else 0.0
    print("   all harmonics  : {:.1f} uV RMS   removes {:.0f}%   ({} notches)".format(uv, pct, n))
    print()
    print("   If 50 Hz removes most of it, a single notch is enough.")
    print("   If 100 or 150 Hz removes the most, the hum is in the harmonics: room")
    print("   lighting or high electrode impedance, even on a battery-powered host.")
    print("   If none of them remove much, it is broadband, not mains.")
    print()
    table_header()


def shorted_input_test():
    print()
    print("--- SHORTED-INPUT REFERENCE")
    print("   Unplug the electrodes from your skin. Bridge E1 + E2 + REF together")
    print("   through a 10 kOhm resistor and plug that into the 3.5 mm jack.")
    while not input("   Then type 'g' + Enter: ").strip().lower().startswith("g"):
        pass
    print("   letting the front-end settle, then re-zeroing...")
    for _ in range(4000):
        emg.read_raw_uncorrected()
    emg.calibrate_baseline(512)
    raw_uv, peak, dc = measure_raw()
    notched_uv = measure_with_mains_notch()[0]
    print("   peak {} counts   DC {:.0f} counts   ac {:.1f} uV   ac notched {:.1f} uV".format(
        peak, dc, raw_uv, notched_uv))
    print()
    if raw_uv < 25.0:
        print("   VERDICT: board + cable are CLEAN. What you saw on skin is pickup at the")
        print("   electrode/body interface: REF firmly on bone, fresh gel pads, short leads.")
    elif raw_uv < 200.0:
        print("   VERDICT: some board/cable pickup, but the electrodes still dominate.")
    else:
        print("   VERDICT: the BOARD OR CABLE picks this up on its own. Suspect the input")
        print("   network, the cable or the jack. Re-run with the jack EMPTY.")
    print()
    while not input("   Reconnect the electrodes, then type 'r' + Enter: ").strip().lower().startswith("r"):
        pass
    emg.calibrate_baseline(512)
    print()
    table_header()


def report_window():
    emg.reset_clock()
    missed0 = emg.missed_samples()
    raw_uv, peak, dc = measure_raw()
    notched_uv = measure_with_mains_notch()[0]
    missed = emg.missed_samples() - missed0
    hum = max(0.0, 100.0 * (raw_uv - notched_uv) / raw_uv) if raw_uv > 0.01 else 0.0

    if peak >= 2040:
        verdict = "RAILED - clipped at full scale"
    elif abs(dc) > 100.0:
        verdict = "DC OFFSET - baseline is stale, re-zero (relax, then reset)"
    elif peak > 1000:
        verdict = "HUGE - ~100x physiological. Not muscle."
    elif hum > 60.0 and raw_uv > 100.0:
        verdict = "hum dominated - electrodes or room lighting, not the board"
    elif raw_uv < 20.0:
        verdict = "quiet - at or near the board noise floor"
    elif notched_uv < 150.0:
        verdict = "light contraction"
    else:
        verdict = "strong contraction"
    print("   {:9d} {:8.0f} {:11.1f} {:13.1f} {:5.0f}% {:7d}  {}".format(
        peak, dc, raw_uv, notched_uv, hum, missed, verdict))


while True:
    c = command()
    if c == "s":
        shorted_input_test()
    elif c == "h":
        harmonic_breakdown()
    report_window()
