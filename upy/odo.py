
# a convenient alias for testing that forgets its self

import sys
import utime, time
import math
from logger import Logger, Level
from colorama import Fore, Style

from pyb import Timer, ExtInt, Pin

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

TICKS_PER_ROTATION = 1350
total_rotations  = 10
tick_limit =  TICKS_PER_ROTATION * total_rotations

#WHEEL_DIAMETER_MM = 48
WHEEL_DIAMETER_MM = 104
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM  # ≈ 326.73 mm

DIRECTION_FORWARD = 1
DIRECTION_REVERSE = 0

tick_count = 0
last_encoder_pulse_us = utime.ticks_us() 
debounce_period_us = 500
current_logical_direction = DIRECTION_FORWARD
last_calculated_interval_us = 0
timer4      = None
channel3    = None
encoder_irq = None


def encoder_callback(pin):
    global tick_count, last_encoder_pulse_us, current_logical_direction, last_calculated_interval_us

    current_time_us = utime.ticks_us()
    last_pulse_time = last_encoder_pulse_us
    debounce_period = debounce_period_us
    raw_interval_us = utime.ticks_diff(current_time_us, last_pulse_time)
    if raw_interval_us >= debounce_period:
        last_encoder_pulse_us = current_time_us
        if current_logical_direction == DIRECTION_FORWARD:
            tick_count += 1
        else:
            tick_count -= 1
        last_calculated_interval_us = raw_interval_us

def main():

    __log = Logger('odo', level=Level.INFO)

    __log.info(Fore.GREEN + 'counting to {}ticks…'.format(tick_limit))
    __log.info(Style.BRIGHT + '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
    __log.info(Style.BRIGHT + 'executing odo…')

    try:

        __log.info('start odo…')

        timer4 = Timer(4, freq=20000)
        channel3 = timer4.channel(3, Timer.PWM, pin=Pin('B8'))
        channel3.pulse_width_percent(100) # stop
        
        # direction: B3
        direction_pin = Pin('B3', Pin.OUT)
        direction_pin.value(DIRECTION_FORWARD)
        # encoder pin: C4
        encoder_pin = Pin('C4', Pin.IN, Pin.PULL_UP)
        encoder_irq = ExtInt(encoder_pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, encoder_callback)

        start_time = time.ticks_ms()

        # set PWM duty cycle to 50% (half speed)
        channel3.pulse_width_percent(50)

        __log.info('start moving…')

        while True:
            if tick_count <= tick_limit:
                now = time.ticks_ms()
                elapsed_ms = time.ticks_diff(now, start_time)
                elapsed_sec = elapsed_ms / 1000

                rotations =  tick_count / TICKS_PER_ROTATION
                rpm = (rotations / elapsed_sec) * 60 if elapsed_sec > 0 else 0

                distance_mm = rotations * WHEEL_CIRCUMFERENCE_MM
                speed_mm_per_sec = distance_mm / elapsed_sec
                speed_cm_per_sec = speed_mm_per_sec / 10.0

                print('{:5d} ticks; {:7.2f} RPM; {:7.2f} cm/s; {:8.3f} rotations; {:6.2f} s elapsed.'.format(
                    tick_count, rpm, speed_cm_per_sec, rotations, elapsed_sec))
            else:
                break

        total_elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
        print("Final time: {:.2f}s for {} ticks".format(total_elapsed, tick_count))

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("{} raised in test: {}".format(type(e), e))
    finally:
        if timer4:
            if channel3:
                channel3.pulse_width_percent(100)
        if encoder_irq:
            encoder_irq.disable()
            encoder_irq = None
            print('disabled IRQ.')

        print('complete.')

# for REPL usage or testing
def exec():
    main()

if __name__ == "__main__" or __name__ == "odo":
    main()
    import sys
    del sys.modules[__name__]

#EOF
