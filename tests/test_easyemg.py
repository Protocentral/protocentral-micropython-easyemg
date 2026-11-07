# SPDX-License-Identifier: MIT
# Mocked-bus unit tests for the easyEMG driver (which depends on protocentral_tla20xx).
# Runs on desktop CPython: `python tests/test_easyemg.py`.
# Needs protocentral_tla20xx: a sibling protocentral-micropython-tla20xx checkout, or
# `pip install protocentral-tla20xx`.

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "..", "protocentral-micropython-tla20xx"))

from protocentral_tla20xx import _REG_CONFIG, _REG_CONVERSION, DR_128SPS, MUX_AIN2_AIN3
import protocentral_easyemg as drv
from protocentral_easyemg import (
    EasyEMG, MvcRecorder, EASYEMG_I2C_ADDR_DEFAULT,
    MVC_NONE, MVC_RELAX, MVC_CONTRACT, MVC_TOO_WEAK, MVC_SET,
)


class FakeI2C:
    def __init__(self, addresses=(0x48,)):
        self.regs = {}
        self.writes = []
        self.addresses = list(addresses)

    def store(self, reg, value):
        self.regs[reg] = bytes([(value >> 8) & 0xFF, value & 0xFF])

    def _ack(self, addr):
        # machine.I2C and smbus2 both raise OSError when nothing answers.
        if addr not in self.addresses:
            raise OSError(19, "ENODEV")

    def readfrom_mem(self, addr, reg, n):
        self._ack(addr)
        return self.regs.get(reg, b"\x00\x00")[:n]

    def writeto_mem(self, addr, reg, buf):
        self._ack(addr)
        self.regs[reg] = bytes(buf)
        self.writes.append((reg, (buf[0] << 8) | buf[1]))

    def scan(self):
        return self.addresses


def _ready():
    bus = FakeI2C()
    emg = EasyEMG(bus)
    emg.begin()
    return bus, emg


def _feed(bus, count):
    bus.store(_REG_CONVERSION, (count << 4) & 0xFFFF)


def test_default_address_is_stock_board():
    assert EASYEMG_I2C_ADDR_DEFAULT == 0x48


def test_begin_writes_arduino_config_word():
    bus = FakeI2C()
    assert EasyEMG(bus).begin() is True
    assert bus.writes[-1] == (_REG_CONFIG, 0x0AC3), hex(bus.writes[-1][1])


def test_begin_false_when_absent():
    bus = FakeI2C(addresses=())
    assert EasyEMG(bus).begin() is False
    assert bus.writes == []


def test_begin_false_at_wrong_address():
    # A board re-strapped to 0x49 with the driver left at the default.
    assert EasyEMG(FakeI2C(addresses=(0x49,))).begin() is False


def test_begin_true_at_alt_address():
    assert EasyEMG(FakeI2C(addresses=(0x49,)), address=0x49).begin() is True


def test_read_config_returns_written_word():
    bus, emg = _ready()
    assert emg.read_config() == 0x0AC3


def test_one_count_is_2_42_uv():
    bus, emg = _ready()
    _feed(bus, 1)
    assert abs(emg.read_microvolts() - 2.42) < 0.005


def test_set_gain_scales():
    bus, emg = _ready()
    emg.set_gain(50.0)
    _feed(bus, 100)
    assert math.isclose(emg.read_microvolts(), 250.0, rel_tol=1e-9)


def test_negative_count_rounds_away_and_sign_extends():
    bus, emg = _ready()
    _feed(bus, -7)
    assert emg.read_raw() == -7


def test_envelope_of_constant_is_its_magnitude():
    bus, emg = _ready()
    _feed(bus, -100)
    env = 0.0
    for _ in range(emg.get_envelope_window() + 5):
        env = emg.read_envelope()
    assert math.isclose(env, 100 * 125.0 / 51.7, rel_tol=1e-6)


def test_envelope_window_slides():
    bus, emg = _ready()
    emg.set_envelope_window(4)
    _feed(bus, 100)
    for _ in range(4):
        emg.read_envelope_raw()
    _feed(bus, 0)
    assert emg.read_envelope_raw() == int(math.sqrt(3 * 100 * 100 / 4) + 0.5)
    for _ in range(3):
        last = emg.read_envelope_raw()
    assert last == 0


def test_envelope_of_sine_is_peak_over_root2():
    bus, emg = _ready()
    emg.set_envelope_window(100)
    out = 0
    for i in range(300):
        _feed(bus, int(round(400 * math.sin(2 * math.pi * i / 100))))
        out = emg.read_envelope_raw()
    assert abs(out - 400 / math.sqrt(2)) < 2


def test_notch_removes_mains():
    bus, emg = _ready()
    fs = 1000.0
    assert emg.set_notch(50.0, fs) is True
    last = []
    for i in range(3000):
        _feed(bus, int(round(500 * math.sin(2 * math.pi * 50.0 * i / fs))))
        last.append(abs(emg.read_raw()))
    assert max(last[-200:]) < 15, max(last[-200:])


def test_notch_passes_emg_band():
    bus, emg = _ready()
    fs = 1000.0
    emg.set_notch(50.0, fs)
    last = []
    for i in range(3000):
        _feed(bus, int(round(500 * math.sin(2 * math.pi * 150.0 * i / fs))))
        last.append(abs(emg.read_raw()))
    assert max(last[-200:]) > 450


def test_notch_refused_at_low_rate():
    _, emg = _ready()
    assert emg.set_notch(50.0, 100.0) is False
    assert emg.notch_enabled() is False


def test_baseline():
    bus, emg = _ready()
    _feed(bus, 12)
    emg.calibrate_baseline(16)
    assert emg.get_baseline() == 12
    assert emg.read_raw() == 0
    assert emg.read_raw_uncorrected() == 12


def test_activation_and_mvc():
    bus, emg = _ready()
    emg.set_envelope_window(1)
    _feed(bus, 100)
    env = emg.read_envelope()
    emg.set_activation_threshold(env + 1.0)
    assert not emg.is_active()
    emg.set_activation_threshold(env - 1.0)
    assert emg.is_active()
    emg.set_mvc(env * 2.0)
    assert math.isclose(emg.read_percent_mvc(), 50.0, rel_tol=1e-6)



# ----------------------------------------------------------------- helpers for 1.1.0 parity
def _section_gain(emg, i, f, fs):
    """|H| of notch section i at f Hz, from its coefficients."""
    b0, b1, a2 = emg._nb0[i], emg._nb1[i], emg._na2[i]
    z1 = complex(math.cos(-2 * math.pi * f / fs), math.sin(-2 * math.pi * f / fs))
    z2 = z1 * z1
    return abs((b0 + b1 * z1 + b0 * z2) / (1 + b1 * z1 + a2 * z2))


def _run_tone(bus, emg, f, fs, amp=500, n=4000, tail=400):
    out = []
    for i in range(n):
        _feed(bus, int(round(amp * math.sin(2 * math.pi * f * i / fs))))
        out.append(abs(emg.read_raw()))
    return max(out[-tail:])


class FakeClock:
    def __init__(self):
        self.t = 1000

    def __call__(self):
        return self.t


def _with_clock(fn):
    saved = drv.ticks_us
    clock = FakeClock()
    drv.ticks_us = clock
    try:
        fn(clock)
    finally:
        drv.ticks_us = saved


# ----------------------------------------------------------------- harmonic notch
def test_mains_notch_section_counts():
    _, emg = _ready()
    assert emg.set_mains_notch(50.0, 1000.0) == 9         # 50 ... 450 Hz
    assert emg.notch_sections() == 9 and emg.notch_enabled()
    assert emg.set_mains_notch(60.0, 1000.0) == 7         # 60 ... 420 Hz
    assert emg.set_mains_notch(50.0, 500.0) == 4          # 50 ... 200 Hz, the default rate
    assert emg.set_mains_notch(50.0, 1000.0, max_harmonics=3) == 3
    assert emg.set_mains_notch(50.0, 3000.0) == 9         # capped at 9 sections
    assert emg.set_notch(50.0, 1000.0) is True
    assert emg.notch_sections() == 1


def test_mains_notch_refusals_leave_it_off():
    _, emg = _ready()
    emg.set_mains_notch(50.0, 1000.0)
    assert emg.set_mains_notch(50.0, 100.0) == 0          # 50 Hz > 0.45 x 100
    assert not emg.notch_enabled() and emg.notch_sections() == 0
    assert emg.set_mains_notch(50.0, 1000.0, bandwidth_hz=25.0) == 0
    assert emg.set_mains_notch(50.0, 1000.0, fundamental_bandwidth_hz=0.0) == 0
    assert emg.set_mains_notch(0.0, 1000.0) == 0
    assert not emg.notch_enabled()


def test_mains_notch_widths_6hz_fundamental_2hz_harmonics():
    _, emg = _ready()
    fs = 1000.0
    emg.set_mains_notch(50.0, fs)
    # -3 dB edges: 3 Hz either side of the fundamental, 1 Hz either side of a harmonic.
    assert abs(_section_gain(emg, 0, 53.0, fs) - 0.7071) < 0.02
    assert abs(_section_gain(emg, 0, 47.0, fs) - 0.7071) < 0.02
    assert abs(_section_gain(emg, 1, 101.0, fs) - 0.7071) < 0.02
    assert abs(_section_gain(emg, 8, 451.0, fs) - 0.7071) < 0.02   # same width near Nyquist
    assert _section_gain(emg, 2, 150.0, fs) < 1e-3


def test_mains_notch_removes_every_harmonic():
    fs = 1000.0
    for f in (50.0, 100.0, 150.0, 450.0):
        bus, emg = _ready()
        emg.set_mains_notch(50.0, fs)
        assert _run_tone(bus, emg, f, fs, n=6000) < 15, f


def test_mains_notch_passes_between_harmonics():
    bus, emg = _ready()
    emg.set_mains_notch(50.0, 1000.0)
    assert _run_tone(bus, emg, 125.0, 1000.0) > 450


def test_single_notch_leaves_harmonics():
    bus, emg = _ready()
    emg.set_notch(50.0, 1000.0)
    assert _run_tone(bus, emg, 150.0, 1000.0) > 450


# ----------------------------------------------------------------- digital high-pass
def test_high_pass_removes_drift_and_passes_emg():
    fs = 1000.0
    bus, emg = _ready()
    assert emg.set_high_pass(20.0, fs) is True and emg.high_pass_enabled()
    assert _run_tone(bus, emg, 1.0, fs, n=6000, tail=2000) < 5     # 1% of 500
    bus, emg = _ready()
    emg.set_high_pass(20.0, fs)
    assert _run_tone(bus, emg, 150.0, fs) > 480


def test_high_pass_refused_and_disabled():
    _, emg = _ready()
    assert emg.set_high_pass(20.0, 40.0) is False
    assert not emg.high_pass_enabled()
    emg.set_high_pass(20.0, 1000.0)
    emg.disable_high_pass()
    assert not emg.high_pass_enabled()


def test_high_pass_removes_dc_step():
    bus, emg = _ready()
    emg.set_high_pass(20.0, 1000.0)
    _feed(bus, 300)
    for _ in range(1000):
        v = emg.read_raw()
    assert abs(v) <= 1


def test_calibrate_baseline_clears_filter_state():
    bus, emg = _ready()
    emg.set_high_pass(20.0, 1000.0)
    emg.set_mains_notch(50.0, 1000.0)
    _feed(bus, 500)
    emg.read_raw()
    emg.calibrate_baseline(4)
    assert emg._hz1 == 0.0 and emg._hz2 == 0.0
    assert all(z == 0.0 for z in emg._nz1) and all(z == 0.0 for z in emg._nz2)
    assert emg.read_raw() == 0


# ----------------------------------------------------------------- paced sampling
def test_set_sample_rate_validates():
    _, emg = _ready()
    assert emg.sample_rate_hz() == 500
    assert emg.set_sample_rate(1000) is True and emg.sample_rate_hz() == 1000
    assert emg.set_sample_rate(0) is False
    assert emg.set_sample_rate(5000) is False
    assert emg.sample_rate_hz() == 1000


def test_update_paces_on_the_clock():
    def body(clock):
        bus, emg = _ready()
        _feed(bus, 10)
        assert emg.update() is True              # first call starts the clock
        assert emg.update() is False             # not due yet
        clock.t += 1999
        assert emg.update() is False
        clock.t += 1                             # 2000 us = one period at 500 SPS
        assert emg.update() is True
        assert emg.last_raw() == 10
        assert emg.missed_samples() == 0
    _with_clock(body)


def test_update_catches_up_small_lag_without_counting():
    def body(clock):
        _, emg = _ready()
        emg.update()
        clock.t += 3 * 2000                      # 3 periods late: catch up in a burst
        taken = 0
        while emg.update():
            taken += 1
        assert taken == 3
        assert emg.missed_samples() == 0
    _with_clock(body)


def test_update_counts_missed_samples_when_far_behind():
    def body(clock):
        _, emg = _ready()
        emg.update()
        clock.t += 10 * 2000                     # stalled for 10 periods
        assert emg.update() is True
        assert emg.missed_samples() == 9         # 10 periods late from the due time
        assert emg.update() is False             # resynced to now
    _with_clock(body)


def test_reset_clock_does_not_count_missed():
    def body(clock):
        _, emg = _ready()
        emg.update()
        clock.t += 100000
        emg.reset_clock()
        assert emg.update() is True
        assert emg.missed_samples() == 0
    _with_clock(body)


def test_update_feeds_the_envelope():
    def body(clock):
        bus, emg = _ready()
        emg.set_envelope_window(4)
        _feed(bus, -100)
        for _ in range(4):
            clock.t += 2000
            emg.update()
        assert emg.last_envelope_raw() == 100
        assert math.isclose(emg.last_envelope(), 100 * emg.microvolts_per_count())
    _with_clock(body)


def test_update_real_clock_rate():
    bus, emg = _ready()
    import time
    n = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.2:
        if emg.update():
            n += 1
    assert 90 <= n <= 110, n                     # 500 SPS for 200 ms


# ----------------------------------------------------------------- passthroughs
def test_microvolts_per_count():
    _, emg = _ready()
    assert math.isclose(emg.microvolts_per_count(), 0.256 / 2048 * 1e6 / 51.7)


def test_low_level_passthroughs_write_conf():
    bus, emg = _ready()
    emg.set_data_rate(DR_128SPS)
    assert bus.writes[-1] == (_REG_CONFIG, 0x0A03), hex(bus.writes[-1][1])
    emg.set_mux(MUX_AIN2_AIN3)
    assert bus.writes[-1] == (_REG_CONFIG, 0x3A03), hex(bus.writes[-1][1])


def test_read_single_shot_sets_os_bit():
    bus, emg = _ready()
    _feed(bus, 42)
    assert emg.read_single_shot() == 42
    assert bus.writes[-1] == (_REG_CONFIG, 0x8BC3), hex(bus.writes[-1][1])


# ----------------------------------------------------------------- MVC recorder
RATE = 100                                       # updates/s, keeps the tests short


def _feed_mvc(rec, uv, seconds):
    for _ in range(int(seconds * RATE)):
        rec.update(uv)
    return rec.step()


def test_mvc_step_codes_match_openview():
    assert (MVC_NONE, MVC_RELAX, MVC_CONTRACT, MVC_TOO_WEAK, MVC_SET) == (0, 1, 2, 3, 4)
    assert drv.EASYEMG_MVC_SETTLE_MS == 2000 and drv.EASYEMG_MVC_RECORD_MS == 5000
    assert drv.EASYEMG_MVC_MIN_UV == 100.0 and drv.EASYEMG_MVC_MIN_REST_RATIO == 5.0
    assert drv.EASYEMG_MVC_AUTO_RAISE is True


def test_mvc_records_after_relax_and_contract():
    _, emg = _ready()
    rec = MvcRecorder(emg, RATE)
    assert rec.step() == MVC_NONE
    rec.start()
    assert rec.step() == MVC_RELAX and not rec.led()
    # First second of the relax is ignored, the last second is the rest level.
    assert _feed_mvc(rec, 1000.0, 1.0) == MVC_RELAX
    assert _feed_mvc(rec, 10.0, 1.0) == MVC_CONTRACT
    assert math.isclose(rec.rest_uv(), 10.0)
    assert rec.led()
    rec.update(300.0)
    rec.update(800.0)
    assert _feed_mvc(rec, 400.0, 5.0 - 0.02) == MVC_SET
    assert not rec.led()
    assert rec.mvc_uv() == 800.0
    assert emg.get_mvc() == 800.0


def test_mvc_refuses_under_100uv_and_blinks():
    rec = MvcRecorder(None, RATE)
    rec.start()
    _feed_mvc(rec, 0.0, 2.0)
    assert _feed_mvc(rec, 90.0, 5.0) == MVC_TOO_WEAK
    leds = set()
    for _ in range(RATE * 2 - 1):
        rec.update(0.0)
        leds.add(rec.led())
    assert leds == {True, False}                 # blinked
    rec.update(0.0)
    assert rec.step() == MVC_NONE                # no previous MVC to fall back on
    assert rec.mvc_uv() == 0.0


def test_mvc_refuses_under_5x_rest_and_keeps_previous():
    rec = MvcRecorder(None, RATE)
    rec.start()
    _feed_mvc(rec, 10.0, 2.0)
    _feed_mvc(rec, 500.0, 5.0)
    assert rec.mvc_uv() == 500.0
    rec.start()                                  # 'm' from the host
    _feed_mvc(rec, 80.0, 2.0)                    # rest 80 -> needs 400
    assert rec.threshold_uv() == 400.0
    assert _feed_mvc(rec, 350.0, 5.0) == MVC_TOO_WEAK
    _feed_mvc(rec, 0.0, 2.0)
    assert rec.step() == MVC_SET and rec.mvc_uv() == 500.0


def test_mvc_auto_raise():
    rec = MvcRecorder(None, RATE)
    rec.update(150.0)                            # no step run, over 100 uV
    assert rec.step() == MVC_SET and rec.mvc_uv() == 150.0
    rec.update(90.0)
    assert rec.mvc_uv() == 150.0
    rec.update(600.0)
    assert rec.mvc_uv() == 600.0
    off = MvcRecorder(None, RATE, auto_raise=False)
    off.update(150.0)
    assert off.step() == MVC_NONE and off.mvc_uv() == 0.0


def test_mvc_percent_is_rounded():
    rec = MvcRecorder(None, RATE)
    assert rec.percent(50.0) == 0                # no MVC yet
    rec.update(300.0)
    assert rec.percent(150.0) == 50
    assert rec.percent(1.0) == 0                 # 0.33 -> 0
    assert rec.percent(2.0) == 1                 # 0.67 -> 1, not truncated to 0
    assert rec.percent(299.0) == 100             # 99.67 -> 100


# ----------------------------------------------------------------- OpenView framing
def test_openview_example_packet_framing():
    import types
    bus = FakeI2C()
    machine = types.ModuleType("machine")
    machine.Pin = lambda *a, **k: None
    machine.Pin.OUT = 1
    machine.I2C = lambda *a, **k: bus
    sys.modules["machine"] = machine

    class Done(Exception):
        pass

    class Out:
        def __init__(self):
            self.packets = []

        def write(self, b):
            self.packets.append(bytes(b))
            if len(self.packets) >= 20:
                raise Done()

        def flush(self):
            pass

    class Std:
        def __init__(self):
            self.buffer = Out()

    saved = sys.stdout, sys.stdin
    std = Std()
    sys.stdout = std
    sys.stdin = object()                         # no fileno: no host commands
    try:
        path = os.path.join(ROOT, "examples", "easyemg_openview.py")
        try:
            exec(compile(open(path).read(), path, "exec"), {"__name__": "__main__"})
        except Done:
            pass
    finally:
        sys.stdout, sys.stdin = saved
        del sys.modules["machine"]

    pkts = std.buffer.packets
    assert len(pkts) == 20
    for p in pkts:
        assert len(p) == 15
        assert p[:5] == bytes([0x0A, 0xFA, 0x08, 0x00, 0x02])
        assert p[13:] == bytes([0x00, 0x0B])
        assert p[11] == MVC_RELAX                # step runs after reset
        assert p[12] == 0


# ----------------------------------------------------------------- calibration, envelope, LED
class EndOfSignal(Exception):
    """Raised by a test signal when it has nothing more to say; ends the example run."""


class SignalI2C(FakeI2C):
    """Conversion register driven by signal(k), k = how many conversions read so far."""

    def __init__(self, signal):
        FakeI2C.__init__(self)
        self.signal = signal
        self.k = 0

    def readfrom_mem(self, addr, reg, n):
        if reg == _REG_CONVERSION:
            self._ack(addr)
            v = (int(self.signal(self.k)) << 4) & 0xFFFF
            self.k += 1
            return bytes([(v >> 8) & 0xFF, v & 0xFF])
        return FakeI2C.readfrom_mem(self, addr, reg, n)


def _run_example(name, signal, max_lines=None, led_log=None):
    """exec examples/<name> on a fake bus and a fake clock; returns (printed lines, bus)."""
    import types
    bus = SignalI2C(signal)
    machine = types.ModuleType("machine")

    def pin(*a, **k):
        p = types.SimpleNamespace()
        p.value = (lambda v: led_log.append(v)) if led_log is not None else (lambda v: None)
        return p
    pin.OUT = 1
    machine.Pin = pin
    machine.I2C = lambda *a, **k: bus
    sys.modules["machine"] = machine

    class Done(Exception):
        pass

    lines = []

    def fake_print(*args, **kw):
        lines.append(" ".join(str(a) for a in args))
        if max_lines is not None and len(lines) >= max_lines:
            raise Done()

    clock = FakeClock()

    def tick():                                  # every read moves time on 500 us
        clock.t += 500
        return clock.t

    saved = drv.ticks_us, drv.sleep_ms
    drv.ticks_us = tick
    drv.sleep_ms = lambda ms: None
    try:
        path = os.path.join(ROOT, "examples", name)
        g = {"__name__": "__main__", "print": fake_print}
        if led_log is not None:
            src = open(path).read().replace("LED_PIN = None", "LED_PIN = 2", 1)
        else:
            src = open(path).read()
        try:
            exec(compile(src, path, "exec"), g)
        except (Done, EndOfSignal):
            pass
    finally:
        drv.ticks_us, drv.sleep_ms = saved
        del sys.modules["machine"]
    return lines, bus


def test_calibration_example_reports_baseline_and_noise():
    # Rest: offset of 7 counts plus a +/-2 square wave -> envelope 2 counts.
    lines, _ = _run_example("easyemg_calibration.py", lambda k: 7 + (2 if k % 2 else -2))
    text = "\n".join(lines)
    assert "Front-end gain        : x51.70" in text
    assert "Resolution at the skin: 2.42 uV per count" in text
    assert "Baseline offset: 7 counts" in text
    env = [l for l in lines if l.startswith("Resting envelope:")][0]
    uv = float(env.split()[2])
    assert abs(uv - 2 * 2.42) < 0.2, env
    assert lines[-1] == "  emg.set_baseline(7)"
    assert "missed" not in text


def test_envelope_example_records_mvc_and_streams_percent():
    # 1 s zeroing reads (256), then a 5 s contraction at +/-100 counts, then +/-20 at rest.
    def sig(k):
        amp = 2 if k < 256 else (100 if k < 256 + 2500 else 20)
        return amp if k % 2 else -amp
    lines, _ = _run_example("easyemg_envelope.py", sig, max_lines=40)
    text = "\n".join(lines)
    assert "Envelope window: 64 samples = 128 ms at 500 SPS" in text
    mvc = float([l for l in lines if l.startswith("MVC recorded:")][0].split()[2])
    assert abs(mvc - 100 * 2.42) < 2, mvc
    assert "weak" not in text
    head = lines.index("Envelope(uV)\t%MVC\tRate(SPS)\tMissed")
    rows = [l.split("\t") for l in lines[head + 1:]]
    assert len(rows) > 10
    env, pct, rate, missed = rows[-1]
    assert abs(float(env) - 20 * 2.42) < 1
    assert abs(float(pct) - 20.0) < 0.5
    assert abs(int(rate) - 500) <= 10            # paced rate, measured (fake clock ticks per call)
    assert missed == "0"


def test_envelope_example_flags_weak_mvc():
    lines, _ = _run_example("easyemg_envelope.py", lambda k: 3 if k % 2 else -3, max_lines=12)
    assert any("weak for an MVC" in l for l in lines)


def test_activation_example_hysteresis_drives_led():
    # rest +/-4 counts, then flex +/-40, dip to +/-14 (between the 2.5x OFF and 4x ON
    # levels), flex again, rest. With hysteresis that is ONE flex; with a single
    # threshold the dip would read as relax and the second flex as another FLEX.
    phases = ((256 + 1000, 4), (500, 40), (500, 14), (500, 40), (500, 4))

    def sig(k):
        for n, a in phases:
            if k < n:
                return a if k % 2 else -a
            k -= n
        raise EndOfSignal()
    leds = []
    lines, _ = _run_example("easyemg_activation_led.py", sig, led_log=leds)
    rest = float([l for l in lines if l.startswith("Resting level")][0].split()[3])
    on = float([l for l in lines if l.startswith("Trigger at")][0].split()[3])
    off = float([l for l in lines if l.startswith("Release at")][0].split()[3])
    assert abs(rest - 4 * 2.42) < 0.2
    assert abs(on - 4.0 * rest) < 0.01 and abs(off - 2.5 * rest) < 0.01
    events = [l.split()[0] for l in lines if l.startswith("FLEX") or l == "relax"]
    assert events == ["FLEX", "relax"], events   # one of each, no chatter
    assert leds == [0, 1, 0]


def test_activation_example_floors_a_silent_board():
    lines, _ = _run_example("easyemg_activation_led.py", lambda k: 5, max_lines=8)
    assert any("under one ADC count" in l for l in lines)
    on = float([l for l in lines if l.startswith("Trigger at")][0].split()[3])
    assert on > 0.0
    assert not any(l.startswith("FLEX") for l in lines)



if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print("ok", name)
    print("ALL PASSED")
