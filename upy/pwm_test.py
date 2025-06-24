#!/micropython
#
#   +---------+------------+-----------+------------+
#   | Motor   | PWM Out    | Direction | PWM In     |
#   +---------+------------+-----------+------------+
#   | Motor 0 | PB6 (X9)   | PB5 (X11) | PB7 (X10)  |
#   | Motor 1 | PB8 (Y3)   | PB13 (Y6) | PB9 (Y4)   |
#   +---------+------------+-----------+------------+
#

import time
from pyb import Pin, Timer

# ..............................................................................

# Callback to track pulses and time between them
def encoder_callback(timer):
    global last_capture, prev_capture, pulse_count
    capture = enc_channel.capture()
    if capture is None:
        return
    pulse_count += 1
    prev_capture = last_capture
    last_capture = capture

# Compute RPM using last capture interval
def calculate_rpm():
    global prev_capture, last_capture, rpm
    if last_capture is None or prev_capture is None:
        rpm = 0.0
        return
    interval = last_capture - prev_capture
    if interval < 0:
        interval += 0x10000  # for 16-bit overflow
    if interval == 0:
        rpm = 0.0
        return
    interval_sec = interval / TIMER_FREQ
    rps = 1.0 / (interval_sec * PULSES_PER_REV)
    rpm = rps * 60.0

# Output RPM and reset pulse count
def report_and_reset():
    global pulse_count
    calculate_rpm()
    print("Pulses: {:5d} | RPM: {:6.2f}".format(pulse_count, rpm))
    pulse_count = 0

# main .........................................................................

# Motor 0 Configuration
PWM_TIMER = Timer(4, freq=25000)  # 25kHz
ENC_TIMER = Timer(4, prescaler=0, period=0xFFFF)  # 16-bit timer

PWM_CHANNEL = 1  # TIM4_CH1 — PB6 (X9)
ENC_CHANNEL = 2  # TIM4_CH2 — PB7 (X10)

# Setup PWM output pin
pwm_pin = Pin('PB6')  # X9
pwm_channel = PWM_TIMER.channel(PWM_CHANNEL, Timer.PWM, pin=pwm_pin)
pwm_channel.pulse_width_percent(0)  # 100% = off in inverted logic

# Setup encoder capture input
encoder_pin = Pin('PB7')  # X10
enc_channel = ENC_TIMER.channel(ENC_CHANNEL, Timer.IC, pin=encoder_pin, polarity=Timer.BOTH)

# Shared variables
last_capture = None
prev_capture = None
pulse_count = 0
rpm = 0.0
TIMER_FREQ = 25000  # Hz
PULSES_PER_REV = 270  # Adjust if needed

enc_channel.callback(encoder_callback)

# Inverted PWM test sequence
try:
    print("inverted PWM motor test starting (Motor 0)")
    while True:
        for duty in [75, 50, 25, 0]:
            pwm_channel.pulse_width_percent(duty)
            print("Set PWM duty (inverted) to {}%".format(duty))
            time.sleep(2)
            report_and_reset()
except Exception as e:
    print("Error during test:", e)
finally:
    pwm_channel.pulse_width_percent(100)  # Fully off
    print("Motor stopped.")

