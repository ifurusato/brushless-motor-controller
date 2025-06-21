#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-06-21
#

import time
from pyb import LED
from core.logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor
from motor_controller import MotorController

_BLUE_LED = LED(1)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

DIAGNOSTIC = True
MOTOR_TEST = False
RPM_TEST   = False
SPEED_TEST = False
PULSE_TEST = False
RAMP_TEST  = True

def main():

    log = Logger('main', level=Level.INFO)
    motor_ctrl = None

    try:
        _config = ConfigLoader.configure('motor_config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        motor_ctrl = MotorController(config=_config, motors_enabled=(True, True, False, False), level=Level.INFO)
        motor_ctrl.enable()
        time.sleep(3) # annoying pre-delay to keep from bricking

        if DIAGNOSTIC:
            motor_ctrl.log_pin_configuration()

        if MOTOR_TEST:
            while True:
                motor_ctrl.set_motor_speed([0, 1], 50)    # 50% speed
                time.sleep(5)
                motor_ctrl.set_motor_speed([0, 1], 0)
                time.sleep(1)

        if RPM_TEST:
            try:

                log.info(Fore.GREEN + 'set to full speed…')
                motor_ctrl.set_motor_speed([0, 1], 100)
                while True:
                    time.sleep(1)

            except KeyboardInterrupt:
                log.info('Ctrl-C caught, user stopped execution.')
            except Exception as e:
                log.error('{} raised: {}'.format(type(e), e))
            finally:
                motor_ctrl.stop()
                time.sleep(1)

        if SPEED_TEST:
            try:
                while True:
                    _BLUE_LED.toggle()
                    # 25% ......................
                    log.info(Fore.GREEN + 'set to 25% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 25)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 50% ......................
                    log.info(Fore.GREEN + 'set to 50% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 50)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 75% ......................
                    log.info(Fore.GREEN + 'set to 75% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 75)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 100% .....................
                    log.info(Fore.GREEN + 'set to 100% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 100)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 75% ......................
                    log.info(Fore.GREEN + 'set to 75% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 75)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 50% ......................
                    log.info(Fore.GREEN + 'set to 50% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 50)
                    time.sleep(3)

                    _BLUE_LED.toggle()
                    # 25% ......................
                    log.info(Fore.GREEN + 'set to 25% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 25)
                    time.sleep(3)

            except KeyboardInterrupt:
                log.info('Ctrl-C caught, user stopped execution.')
            except Exception as e:
                log.error('{} raised: {}'.format(type(e), e))
            finally:
                motor_ctrl.stop()
                time.sleep(1)

        if PULSE_TEST:
            motor_ctrl.set_motor_speed(1, 100) # full speed
            motor0 = motor_ctrl.get_motor(0)
            motor0.measure_ticks_per_second(test_duration=10.0)

        if RAMP_TEST:

            ids = motor_ctrl.motor_ids
            motor_ctrl.enable_rpm_logger(interval_ms=250, timer_freq_hz=100)
            _delay_ms = 200

            while True:

                motor_ctrl.set_motor_direction('all', Motor.DIRECTION_FORWARD)

                # ramp all motors up to full speed
                log.info('accelerate forward…')
                motor_ctrl.accelerate(ids, Motor.FULL_SPEED, step=5, delay_ms=_delay_ms)

                log.info('decelerate…')
                # ramp all motors down to stopped
                motor_ctrl.accelerate(ids, Motor.STOPPED, step=5, delay_ms=_delay_ms)

                log.info('stop…')
                motor_ctrl.stop()

                # toggle direction                
                motor_ctrl.set_motor_direction('all', Motor.DIRECTION_REVERSE)

                # ramp all motors back up to full speed
                log.info('accelerate backward…')
                motor_ctrl.accelerate(ids, Motor.FULL_SPEED, step=5, delay_ms=_delay_ms)

                # ramp all motors down to stopped
                log.info('decelerate…')
                motor_ctrl.accelerate(ids, Motor.STOPPED, step=5, delay_ms=_delay_ms)

                log.info('stop…')
                motor_ctrl.stop()

        else:
            log.info('wait...')
            time.sleep(3)

    except KeyboardInterrupt:
        log.info('Ctrl-C caught, user stopped execution.')
    finally:
        if motor_ctrl:
            motor_ctrl.close()
        log.info('complete.')

if __name__ == '__main__':
    main()

#EOF
