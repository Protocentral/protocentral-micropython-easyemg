# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
#
# easyemg_activation_led.py - Arduino counterpart: 05.MuscleActivation_LED
#
# Flex the muscle, the LED lights. Runs on MicroPython and on a Raspberry Pi.
#
# The classic muscle-sensor demo, and the starting point for prosthetics and gesture
# control. The threshold is set from YOUR resting noise floor rather than hard-coded,
# because the right number depends on electrode placement, skin prep and the person: the
# script measures the resting envelope, then triggers at a multiple of it.
#
# Hysteresis matters. A single threshold makes the LED chatter whenever the envelope sits
# near it, so this uses separate ON and OFF levels: a Schmitt trigger, in software.
#
# FLEX / relax is printed either way, so it works with no LED. Set LED_PIN for yours.
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
    Pin = None

from protocentral_easyemg import EasyEMG, sleep_ms

# MicroPython: "LED" on a Pico / Pico W, 2 on many ESP32 dev boards. The QT Py ESP32-C3
# has only an RGB NeoPixel, so wire an LED (with a resistor) to a free pin.
# Raspberry Pi: a BCM pin number, driven through gpiozero.
LED_PIN = None

TRIGGER_MULTIPLE = 4.0                  # ON at 4x the resting noise floor ...
RELEASE_MULTIPLE = 2.5                  # ... OFF below 2.5x
ENVELOPE_WINDOW = 32                    # samples: 64 ms at 500 SPS, quicker to react
REST_SECONDS = 2


def make_led(pin):
    """A function that sets the LED on (1) or off (0), or None with no LED."""
    if pin is None:
        return None
    if Pin is not None:
        p = Pin(pin, Pin.OUT)
        return p.value
    from gpiozero import LED            # Raspberry Pi
    led = LED(pin)
    return lambda v: led.on() if v else led.off()


set_led = make_led(LED_PIN)
if set_led:
    set_led(0)

emg = EasyEMG(i2c)                      # ships at 0x48
if not emg.begin():
    raise OSError("easyEMG not found - check wiring and I2C address (0x48)")

print("Relax the muscle. Measuring your resting noise floor...")
sleep_ms(1000)
emg.calibrate_baseline()
emg.set_envelope_window(ENVELOPE_WINDOW)

# Average the resting envelope over a couple of seconds rather than trusting one reading.
total = 0.0
n = 0
target = REST_SECONDS * emg.sample_rate_hz()
while n < target:
    if emg.update():
        total += emg.last_envelope()
        n += 1
resting = total / n

# A dead-quiet (or stuck) ADC gives a resting level of 0, and a 0 threshold would light the
# LED permanently. Hold the floor at one ADC count.
floor = emg.microvolts_per_count()
if resting < floor:
    print("Resting level is under one ADC count. If every reading stays at 0, run")
    print("easyemg_diagnostics.py: step 4 tells a quiet board from a stuck ADC.")
    resting = floor

on_threshold = resting * TRIGGER_MULTIPLE
off_threshold = resting * RELEASE_MULTIPLE
emg.set_activation_threshold(on_threshold)

print("Resting level : {:.2f} uV RMS".format(resting))
print("Trigger at    : {:.2f} uV RMS".format(on_threshold))
print("Release at    : {:.2f} uV RMS".format(off_threshold))
print()
print("Flex the muscle.")

muscle_on = False
while True:
    if not emg.update():
        continue
    envelope = emg.last_envelope()

    # Schmitt trigger: different levels for turning on and off.
    if not muscle_on and envelope >= on_threshold:
        muscle_on = True
        if set_led:
            set_led(1)
        print("FLEX  {:.1f} uV".format(envelope))
    elif muscle_on and envelope < off_threshold:
        muscle_on = False
        if set_led:
            set_led(0)
        print("relax")

    # emg.is_active() is the simple single-threshold version, if you don't want
    # hysteresis:
    #   if emg.is_active(): ...
