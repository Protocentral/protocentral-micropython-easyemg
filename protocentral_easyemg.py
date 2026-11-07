# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# MicroPython driver for the Protocentral easyEMG single-channel surface EMG breakout.
# An INA333 and two OPA2333 stages (total gain x51.7, roughly 20-310 Hz) feed an on-board
# TI TLA2024 12-bit ADC, read differentially against the buffered 1.65 V mid-rail. This
# library DEPENDS ON the TLA20xx driver and adds the easyEMG configuration, microvolt
# conversion, RMS envelope, mains notches, digital high-pass, activation and %MVC.
#
# Ported from the Protocentral easyEMG Arduino library 1.1.0 (single source of truth):
#   https://github.com/Protocentral/protocentral_easyemg_arduino
#
# Unlike an analog "muscle sensor", easyEMG hands you the RAW band-limited waveform. The
# envelope is computed here, so you can always get back to the underlying signal.
#
# Two additions the Arduino library doesn't have, because Python needs them more:
#
#   update()        a paced sample clock (default 500 SPS) with missed-sample counting.
#                   The Arduino sketches pace their own loops; here the driver does it,
#                   as the easyECG driver does. The notch and high-pass are built for an
#                   exact rate, so a clock the host can hold beats a faster one it can't.
#   MvcRecorder     the MVC recording step from the Arduino 06.OpenView sketch: relax,
#                   contract, refuse a weak reference, auto-raise.
#
# Requires: protocentral_tla20xx  (mip/pip dependency)

import math

try:
    from time import ticks_us, ticks_diff, ticks_add, sleep_ms, sleep_us
except ImportError:                     # CPython (Raspberry Pi)
    import time as _time

    def ticks_us():
        return int(_time.monotonic() * 1000000)

    def ticks_diff(a, b):
        return a - b

    def ticks_add(a, b):
        return a + b

    def sleep_ms(ms):
        _time.sleep(ms / 1000.0)

    def sleep_us(us):
        _time.sleep(us / 1000000.0)

from protocentral_tla20xx import (
    TLA20XX, FSR_6_144V, FSR_4_096V, FSR_2_048V, FSR_1_024V, FSR_0_512V, FSR_0_256V,
    DR_3300SPS, MUX_AIN0_AIN1, OP_CONTINUOUS, OP_SINGLE, _REG_CONFIG,
)

# JP1 bridged 1-2 straps ADDR to GND -> 0x48. Cut and bridge 2-3 for 0x49. An easyECG
# ships at 0x49, so the two share a bus without touching the jumper.
EASYEMG_I2C_ADDR_DEFAULT = 0x48
EASYEMG_I2C_ADDR_ALT = 0x49

# INA333 x11 (R10 = 10k) times 1 + R14/R19 = 1 + 100k/27k = x4.7. Design value; pass your
# measured gain to set_gain(). Envelope shape, %MVC and activation ratios don't depend on it.
EASYEMG_DEFAULT_GAIN = 51.7

EASYEMG_HPF_HZ = 20.0                   # C6/R13 (9.1k), 1-pole
EASYEMG_LPF_HZ = 310.0                  # Sallen-Key, Q = 0.5

EASYEMG_MAX_ENVELOPE_WINDOW = 128

# Most biquads the notch chain holds, as on the Arduino library's 32-bit targets. At
# 1000 SPS and 50 Hz mains, set_mains_notch() wants 9 (50-450 Hz); at the default
# 500 SPS it builds 4 (50-200 Hz).
EASYEMG_MAX_NOTCH_SECTIONS = 9

# Paced sample clock. The Arduino 06.OpenView sketch paces at 1000 SPS. Interpreted
# Python can't be relied on to hold that through the envelope, the notches and a serial
# write, so the default here is half of it. Still unmeasured on hardware: watch
# missed_samples().
EASYEMG_DEFAULT_SAMPLE_RATE = 500
EASYEMG_MAX_SAMPLE_RATE = 3300          # the ADC's own conversion rate

# MVC recording, as in the Arduino 06.OpenView sketch.
EASYEMG_MVC_SETTLE_MS = 2000            # relax: filters settle, last second = rest level
EASYEMG_MVC_RECORD_MS = 5000            # contract as hard as you can
EASYEMG_MVC_MIN_UV = 100.0              # refuse an MVC below this ...
EASYEMG_MVC_MIN_REST_RATIO = 5.0        # ... or below this many times the rest level
EASYEMG_MVC_AUTO_RAISE = True           # a stronger contraction becomes the new MVC

# MVC step codes: byte 6 of the OpenView packet.
MVC_NONE = 0                            # no MVC yet
MVC_RELAX = 1                           # settling; the last second is the rest level
MVC_CONTRACT = 2                        # LED lit: contract now
MVC_TOO_WEAK = 3                        # refused; shown for 2 s while the LED blinks
MVC_SET = 4                             # an MVC is in use

_FSR_VOLTS = {
    FSR_6_144V: 6.144, FSR_4_096V: 4.096, FSR_2_048V: 2.048,
    FSR_1_024V: 1.024, FSR_0_512V: 0.512, FSR_0_256V: 0.256,
}

# One conversion period per DR code (128 ... 3300 SPS), microseconds.
_CONV_US = (7813, 4000, 2041, 1087, 625, 417, 303)


def _round(v):
    return int(v + 0.5) if v >= 0.0 else int(v - 0.5)


class EasyEMG:
    """Driver for the Protocentral easyEMG breakout.

    Example:
        from machine import Pin, I2C
        from protocentral_easyemg import EasyEMG
        i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)   # QT Py C3: SCL=6, SDA=5
        emg = EasyEMG(i2c)
        emg.begin()
        emg.calibrate_baseline()
        while True:
            if emg.update():                    # one sample at 500 SPS
                print(emg.last_envelope())      # uV RMS at the electrodes
    """

    def __init__(self, i2c, address=EASYEMG_I2C_ADDR_DEFAULT):
        self.i2c = i2c
        self.address = address
        self.adc = TLA20XX(i2c, address)
        self._fsr = FSR_0_256V
        self._gain = EASYEMG_DEFAULT_GAIN
        self._baseline = 0

        self._env_window = 64
        self._env_buf = [0] * EASYEMG_MAX_ENVELOPE_WINDOW
        self._env_count = 0
        self._env_index = 0
        self._env_sum_sq = 0
        self._last_envelope = 0.0
        self._last_env_counts = 0.0
        self._last_raw = 0

        self._threshold = 0.0
        self._mvc = 0.0

        # Notch chain, one biquad per section (direct form II transposed). A notch has
        # b2 = b0 and a1 = b1, so each section keeps three coefficients:
        #   y = b0*x + z1;  z1 = b1*x - b1*y + z2;  z2 = b0*x - a2*y
        n = EASYEMG_MAX_NOTCH_SECTIONS
        self._nb0 = [1.0] * n
        self._nb1 = [0.0] * n
        self._na2 = [0.0] * n
        self._nz1 = [0.0] * n
        self._nz2 = [0.0] * n
        self._notch_count = 0

        # Digital high-pass biquad.
        self._hp_on = False
        self._hb0 = 1.0
        self._hb1 = 0.0
        self._hb2 = 0.0
        self._ha1 = 0.0
        self._ha2 = 0.0
        self._hz1 = 0.0
        self._hz2 = 0.0

        # Paced clock.
        self._rate = EASYEMG_DEFAULT_SAMPLE_RATE
        self._period_us = 1000000 // EASYEMG_DEFAULT_SAMPLE_RATE
        self._next_us = 0
        self._clock_running = False
        self._missed = 0

    # ------------------------------------------------------------- connection
    def begin(self):
        """AIN0/AIN1, +/-0.256 V, continuous, 3300 SPS -> CONF 0x0AC3.

        The +/-0.256 V range is not optional: at the +/-2.048 V power-on default one LSB
        is 19 uV at the electrodes. Returns False without touching the bus further if the
        ADC doesn't answer: a write to an absent device raises OSError on real I2C.
        """
        if not self.is_connected():
            return False
        self.adc.begin()
        self.adc._mux = MUX_AIN0_AIN1
        self.adc._mode = OP_CONTINUOUS
        self.adc._dr = DR_3300SPS
        self._fsr = FSR_0_256V
        self.adc.set_fsr(FSR_0_256V)
        self.reset_envelope()
        self._clock_running = False
        return True

    def is_connected(self):
        return self.address in self.i2c.scan()

    def read_config(self):
        """CONFIG register as read back. Bit 15 reads 0 while a conversion is in progress."""
        return self.adc._read16(_REG_CONFIG)

    # ------------------------------------------------------------- reading
    def read_raw_uncorrected(self):
        """Signed 12-bit counts straight from the ADC."""
        return self.adc.read_adc()

    def read_raw(self):
        """Signed 12-bit counts: baseline-corrected, then high-pass and notch if on."""
        v = float(self.read_raw_uncorrected() - self._baseline)
        if self._hp_on:
            v = self._apply_high_pass(v)
        if self._notch_count:
            v = self._apply_notch(v)
        # The filters' transients can briefly overshoot the ADC's span.
        if v > 2047.0:
            v = 2047.0
        elif v < -2048.0:
            v = -2048.0
        self._last_raw = _round(v)
        return self._last_raw

    def read_voltage(self):
        """Differential ADC voltage, volts (output-referred)."""
        return self.read_raw() * (self.full_scale_volts() / 2048.0)

    def read_microvolts(self):
        """sEMG amplitude in uV at the electrodes: the primary unit."""
        return self._counts_to_uv(self.read_raw())

    def full_scale_volts(self):
        return _FSR_VOLTS[self._fsr]

    def microvolts_per_count(self):
        """uV at the electrodes per ADC count, at the current range and gain."""
        return self._counts_to_uv(1.0)

    def _counts_to_uv(self, counts):
        if self._gain <= 0.0:
            return 0.0
        return counts * (self.full_scale_volts() / 2048.0) * 1000000.0 / self._gain

    # ------------------------------------------------------------- paced sampling
    def set_sample_rate(self, hz):
        """Rate of the update() clock, samples per second. Default 500.

        Returns False and keeps the old rate outside 1..3300. Build the notch and
        high-pass from this same figure: set_mains_notch(50, emg.sample_rate_hz()).
        """
        if hz <= 0 or hz > EASYEMG_MAX_SAMPLE_RATE:
            return False
        self._rate = hz
        self._period_us = int(1000000 // hz)
        self._clock_running = False
        return True

    def sample_rate_hz(self):
        return self._rate

    def update(self):
        """Non-blocking. Call as often as you can.

        When the sample clock is due, takes one sample through read_envelope_raw() and
        returns True; read last_raw(), last_microvolts() and last_envelope() after it.
        Otherwise returns False straight away.
        """
        now = ticks_us()
        if not self._clock_running:
            self._clock_running = True
            self._next_us = now

        late = ticks_diff(now, self._next_us)
        if late < 0:
            return False

        # Advance by whole periods so the rate stays exact. Resync only if badly behind.
        if late > 4 * self._period_us:
            self._missed += late // self._period_us
            self._next_us = now
        self._next_us = ticks_add(self._next_us, self._period_us)

        self.read_envelope_raw()
        return True

    def reset_clock(self):
        """Restart the clock on the next update() without counting missed samples.

        Use it after deliberately stopping (printing a report, waiting for input).
        """
        self._clock_running = False

    def missed_samples(self):
        """Samples the clock couldn't take on time. Should stay 0."""
        return self._missed

    # ------------------------------------------------------------- mains notch
    def _set_notch_section(self, i, hz, sample_rate_hz, alpha):
        # b = [1, -2cos(w0), 1] / (1 + alpha); a2 = (1 - alpha) / (1 + alpha)
        w0 = 2.0 * math.pi * hz / sample_rate_hz
        a0 = 1.0 + alpha
        self._nb0[i] = 1.0 / a0
        self._nb1[i] = (-2.0 * math.cos(w0)) / a0
        self._na2[i] = (1.0 - alpha) / a0
        self._nz1[i] = 0.0
        self._nz2[i] = 0.0

    def set_notch(self, mains_hz, sample_rate_hz, q=4.0):
        """ONE notch at mains_hz, about mains_hz/q wide. OFF by default.

        Pass your ACTUAL sample rate. Leaves the harmonics alone: see set_mains_notch().
        Returns False and leaves the notch off if the rate is below about 2.5x the notch
        frequency.
        """
        if mains_hz <= 0.0 or sample_rate_hz < mains_hz * 2.5 or q <= 0.0:
            self.disable_notch()
            return False
        w0 = 2.0 * math.pi * mains_hz / sample_rate_hz
        self._set_notch_section(0, mains_hz, sample_rate_hz, math.sin(w0) / (2.0 * q))
        self._notch_count = 1
        self.reset_envelope()
        return True

    def set_mains_notch(self, mains_hz, sample_rate_hz, bandwidth_hz=2.0,
                        fundamental_bandwidth_hz=6.0, max_harmonics=0):
        """A notch on EVERY mains harmonic up to 0.45x the sample rate.

        Harmonics are bandwidth_hz wide at -3 dB, the fundamental is
        fundamental_bandwidth_hz wide, so a jump in 50/60 Hz pickup settles in about
        0.1 s. max_harmonics limits the chain (0 = as many as the rate allows, at most 9).
        Returns the number of notches built, or 0 (notch left off) if none can be: no
        harmonic below 0.45x the rate, or a bandwidth outside 0 < bw < mains_hz/2.
        """
        if (mains_hz <= 0.0 or sample_rate_hz <= 0.0 or
                bandwidth_hz <= 0.0 or bandwidth_hz >= mains_hz * 0.5 or
                fundamental_bandwidth_hz <= 0.0 or
                fundamental_bandwidth_hz >= mains_hz * 0.5):
            self.disable_notch()
            return 0

        top = 0.45 * sample_rate_hz
        # alpha = tan(pi * bw / fs): exactly bw wide at -3 dB at any frequency, unlike
        # the RBJ alpha, which narrows towards Nyquist.
        alpha = math.tan(math.pi * bandwidth_hz / sample_rate_hz)
        alpha_fund = math.tan(math.pi * fundamental_bandwidth_hz / sample_rate_hz)

        n = 0
        while n < EASYEMG_MAX_NOTCH_SECTIONS:
            if max_harmonics and n >= max_harmonics:
                break
            hz = mains_hz * (n + 1)
            if hz > top:
                break
            self._set_notch_section(n, hz, sample_rate_hz, alpha_fund if n == 0 else alpha)
            n += 1

        if n == 0:
            self.disable_notch()
            return 0
        self._notch_count = n
        self.reset_envelope()
        return n

    def disable_notch(self):
        self._notch_count = 0
        self._clear_notch_state()
        self.reset_envelope()

    def notch_enabled(self):
        return self._notch_count > 0

    def notch_sections(self):
        """Notches in the chain: 0 off, 1 after set_notch()."""
        return self._notch_count

    def _clear_notch_state(self):
        for i in range(EASYEMG_MAX_NOTCH_SECTIONS):
            self._nz1[i] = 0.0
            self._nz2[i] = 0.0

    def _apply_notch(self, x):
        b0 = self._nb0
        b1 = self._nb1
        a2 = self._na2
        z1 = self._nz1
        z2 = self._nz2
        for i in range(self._notch_count):
            y = (b0[i] * x) + z1[i]
            z1[i] = (b1[i] * x) - (b1[i] * y) + z2[i]
            z2[i] = (b0[i] * x) - (a2[i] * y)
            x = y
        return x

    # ------------------------------------------------------------- digital high-pass
    def set_high_pass(self, corner_hz, sample_rate_hz):
        """2nd-order Butterworth high-pass, OFF by default.

        Takes electrode motion artifact out of the envelope, activation and %MVC. Pass
        your ACTUAL sample rate. Returns False (filter left off) below 2.5x the corner.
        """
        if corner_hz <= 0.0 or sample_rate_hz < corner_hz * 2.5:
            self.disable_high_pass()
            return False
        w0 = 2.0 * math.pi * corner_hz / sample_rate_hz
        cosw0 = math.cos(w0)
        alpha = math.sin(w0) / (2.0 * 0.70710678)
        a0 = 1.0 + alpha
        self._hb0 = ((1.0 + cosw0) / 2.0) / a0
        self._hb1 = -(1.0 + cosw0) / a0
        self._hb2 = self._hb0
        self._ha1 = (-2.0 * cosw0) / a0
        self._ha2 = (1.0 - alpha) / a0
        self._hz1 = 0.0
        self._hz2 = 0.0
        self._hp_on = True
        self.reset_envelope()
        return True

    def disable_high_pass(self):
        self._hp_on = False
        self._hz1 = 0.0
        self._hz2 = 0.0
        self.reset_envelope()

    def high_pass_enabled(self):
        return self._hp_on

    def _apply_high_pass(self, x):
        y = (self._hb0 * x) + self._hz1
        self._hz1 = (self._hb1 * x) - (self._ha1 * y) + self._hz2
        self._hz2 = (self._hb2 * x) - (self._ha2 * y)
        return y

    # ------------------------------------------------------------- RMS envelope
    def set_envelope_window(self, samples):
        """Window in SAMPLES, 1..128, because the rate depends on your host."""
        self._env_window = max(1, min(EASYEMG_MAX_ENVELOPE_WINDOW, int(samples)))
        self.reset_envelope()

    def get_envelope_window(self):
        return self._env_window

    def envelope_window_ms(self, sample_rate_hz):
        if sample_rate_hz <= 0.0:
            return 0.0
        return (1000.0 * self._env_window) / sample_rate_hz

    def reset_envelope(self):
        self._env_count = 0
        self._env_index = 0
        self._env_sum_sq = 0
        self._last_envelope = 0.0
        self._last_env_counts = 0.0
        for i in range(EASYEMG_MAX_ENVELOPE_WINDOW):
            self._env_buf[i] = 0

    def read_envelope_raw(self):
        """RMS of the window, in ADC counts. Takes one new sample."""
        sample = self.read_raw()
        sq = sample * sample
        if self._env_count < self._env_window:
            self._env_sum_sq += sq
            self._env_buf[self._env_index] = sample
            self._env_count += 1
        else:
            old = self._env_buf[self._env_index]
            self._env_sum_sq += sq - (old * old)
            self._env_buf[self._env_index] = sample
        self._env_index += 1
        if self._env_index >= self._env_window:
            self._env_index = 0
        rms = math.sqrt(self._env_sum_sq / self._env_count)
        self._last_env_counts = rms
        self._last_envelope = self._counts_to_uv(rms)
        return int(rms + 0.5)

    def read_envelope(self):
        """RMS of the window, in uV at the electrodes. Takes one new sample."""
        self.read_envelope_raw()
        return self._last_envelope

    def last_envelope(self):
        """Last computed envelope, uV RMS, without a new sample."""
        return self._last_envelope

    def last_envelope_raw(self):
        """Last computed envelope, counts (rounded), without a new sample."""
        return int(self._last_env_counts + 0.5)

    def last_raw(self):
        return self._last_raw

    def last_microvolts(self):
        return self._counts_to_uv(self._last_raw)

    # ------------------------------------------------------------- baseline and gain
    def calibrate_baseline(self, samples=256):
        """Average the resting count. Muscle relaxed, electrodes attached."""
        if samples <= 0:
            return
        total = 0
        for _ in range(samples):
            total += self.read_raw_uncorrected()
        self._baseline = int(total / samples)
        # The filters' state refers to the old baseline; start them clean.
        self._clear_notch_state()
        self._hz1 = 0.0
        self._hz2 = 0.0
        self.reset_envelope()
        self._clock_running = False

    def set_baseline(self, offset_counts):
        self._baseline = offset_counts
        self.reset_envelope()

    def get_baseline(self):
        return self._baseline

    def set_gain(self, gain):
        if gain > 0.0:
            self._gain = gain

    def get_gain(self):
        return self._gain

    # ------------------------------------------------------------- low-level passthroughs
    def set_range(self, fsr):
        """Change the PGA range. The counts-to-uV maths follows automatically."""
        self._fsr = fsr
        self.adc.set_fsr(fsr)
        self.reset_envelope()

    def set_data_rate(self, dr):
        """ADC conversion rate, a DR_* constant. begin() sets DR_3300SPS."""
        self.adc.set_dr(dr)

    def set_mode(self, mode):
        """OP_CONTINUOUS or OP_SINGLE. begin() sets continuous."""
        self.adc.set_mode(mode)

    def set_mux(self, mux):
        """Input mux, a MUX_* constant. begin() sets MUX_AIN0_AIN1."""
        self.adc.set_mux(mux)

    def read_single_shot(self):
        """Start one conversion, wait two conversion periods, return read_raw().

        Leaves the ADC in single-shot mode; call begin() to go back to continuous.
        """
        self.adc._mode = OP_SINGLE
        self.adc._write16(_REG_CONFIG, self.adc._conf_word() | 0x8000)
        dr = self.adc._dr
        wait = _CONV_US[dr] if 0 <= dr < len(_CONV_US) else _CONV_US[0]
        sleep_us(2 * wait)
        return self.read_raw()

    # ------------------------------------------------------------- activation and %MVC
    def set_activation_threshold(self, uv_rms):
        self._threshold = uv_rms

    def get_activation_threshold(self):
        return self._threshold

    def is_active(self):
        """Uses the last computed envelope, so take a sample first."""
        return self._threshold > 0.0 and self._last_envelope >= self._threshold

    def set_mvc(self, uv_rms):
        self._mvc = uv_rms

    def get_mvc(self):
        return self._mvc

    def read_percent_mvc(self):
        if self._mvc <= 0.0:
            return 0.0
        return (self._last_envelope / self._mvc) * 100.0


class MvcRecorder:
    """The MVC recording step from the Arduino 06.OpenView sketch, as a state machine.

    Feed it one envelope value (uV) per call at a steady update_rate_hz. OpenView feeds
    the smoothed envelope it plots, once per packet.

      1. start(): relax for settle_ms. The last second is taken as the resting level.
      2. Contract as hard as you can for record_ms (led() is True).
      3. The MVC is the highest envelope seen. It's refused if below min_uv or below
         min_rest_ratio x rest: step() shows MVC_TOO_WEAK for 2 s, led() blinks at 5 Hz,
         and the previous MVC stays.

    With auto_raise, any later envelope above both the MVC and the refusal threshold
    becomes the new MVC, so %MVC stays at or below 100. Turn it off for measurements.

    If emg is given, its set_mvc() follows, so emg.read_percent_mvc() agrees.
    """

    def __init__(self, emg, update_rate_hz, settle_ms=EASYEMG_MVC_SETTLE_MS,
                 record_ms=EASYEMG_MVC_RECORD_MS, min_uv=EASYEMG_MVC_MIN_UV,
                 min_rest_ratio=EASYEMG_MVC_MIN_REST_RATIO,
                 auto_raise=EASYEMG_MVC_AUTO_RAISE):
        self._emg = emg
        self._per_sec = int(update_rate_hz)
        self._settle_n = (settle_ms * self._per_sec) // 1000
        self._record_n = (record_ms * self._per_sec) // 1000
        self._blink_n = max(1, self._per_sec // 10)
        self._min_uv = min_uv
        self._min_ratio = min_rest_ratio
        self._auto_raise = auto_raise

        self._state = MVC_NONE
        self._left = 0
        self._mvc = 0.0
        self._peak = 0.0
        self._rest = 0.0
        self._rest_sum = 0.0
        self._rest_count = 0
        self._led = False

    def start(self):
        """Begin (or restart) the relax-then-contract step. OpenView's 'm' command."""
        self._state = MVC_RELAX
        self._left = self._settle_n
        self._peak = 0.0
        self._rest_sum = 0.0
        self._rest_count = 0
        self._led = False

    def threshold_uv(self):
        """Smallest envelope accepted as an MVC."""
        t = self._min_ratio * self._rest
        return t if t > self._min_uv else self._min_uv

    def _set(self, uv):
        self._mvc = uv
        if self._emg is not None:
            self._emg.set_mvc(uv)

    def update(self, env_uv):
        """One envelope value, uV. Returns the step code."""
        s = self._state
        if s == MVC_RELAX:
            if self._left <= self._per_sec:         # last second of the settle
                self._rest_sum += env_uv
                self._rest_count += 1
            self._left -= 1
            if self._left <= 0:
                self._rest = (self._rest_sum / self._rest_count) if self._rest_count else 0.0
                self._state = MVC_CONTRACT
                self._left = self._record_n
                self._led = True
            return self._state

        if s == MVC_CONTRACT:
            if env_uv > self._peak:
                self._peak = env_uv
            self._left -= 1
            if self._left <= 0:
                self._led = False
                if self._peak >= self.threshold_uv():
                    self._set(self._peak)
                    self._state = MVC_SET
                else:
                    self._state = MVC_TOO_WEAK
                    self._left = 2 * self._per_sec
            return self._state

        if s == MVC_TOO_WEAK:
            self._led = bool((self._left // self._blink_n) & 1)
            self._left -= 1
            if self._left <= 0:
                self._led = False
                self._state = MVC_SET if self._mvc > 0.0 else MVC_NONE

        if self._auto_raise and env_uv >= self.threshold_uv() and env_uv > self._mvc:
            self._set(env_uv)
            if self._state == MVC_NONE:
                self._state = MVC_SET
        return self._state

    def step(self):
        """0 none, 1 relax, 2 contract, 3 too weak, 4 set."""
        return self._state

    def led(self):
        """True while the user should contract; blinks after a refused MVC."""
        return self._led

    def mvc_uv(self):
        """The MVC in use, uV; 0 until one is recorded."""
        return self._mvc

    def rest_uv(self):
        """Resting level from the last step, uV."""
        return self._rest

    def percent(self, env_uv):
        """env_uv as a rounded percentage of the MVC; 0 with no MVC."""
        if self._mvc <= 0.0:
            return 0
        return int(env_uv / self._mvc * 100.0 + 0.5)
