#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-06-22
#

import uasyncio as asyncio
from pyb import LED
from core.logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor
from motor_controller import MotorController

DIAGNOSTIC = True
MOTOR_TEST = False
RPM_TEST   = False
SPEED_TEST = False
PULSE_TEST = False
RAMP_TEST  = True

_BLUE_LED = LED(1)

async def main():
    log = Logger('main', level=Level.INFO)
    motor_ctrl = None

    try:
        _config = ConfigLoader.configure('motor_config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        motor_ctrl = MotorController(config=_config, motors_enabled=(True, True, False, False), level=Level.INFO)
        motor_ctrl.enable()
        await asyncio.sleep(3)  # async sleep

        if DIAGNOSTIC:
            motor_ctrl.log_pin_configuration()

        if MOTOR_TEST:
            while True:
                motor_ctrl.set_motor_speed([0, 1], 50)
                await asyncio.sleep(5)
                motor_ctrl.set_motor_speed([0, 1], 0)
                await asyncio.sleep(1)

        if RPM_TEST:
            try:
                log.info(Fore.GREEN + 'set to full speed…')
                motor_ctrl.set_motor_speed([0, 1], 100)
                while True:
                    await asyncio.sleep(1)
            except KeyboardInterrupt:
                log.info('Ctrl-C caught, user stopped execution.')
            except Exception as e:
                log.error('{} raised: {}'.format(type(e), e))
            finally:
                motor_ctrl.stop()
                await asyncio.sleep_ms(1000)

        if SPEED_TEST:
            try:
                while True:
                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 25% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 25)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 50% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 50)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 75% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 75)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 100% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 100)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 75% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 75)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 50% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 50)
                    await asyncio.sleep(3)

                    _BLUE_LED.toggle()
                    log.info(Fore.GREEN + 'set to 25% speed…')
                    motor_ctrl.set_motor_speed([0, 1], 25)
                    await asyncio.sleep(3)
            except KeyboardInterrupt:
                log.info('Ctrl-C caught, user stopped execution.')
            except Exception as e:
                log.error('{} raised: {}'.format(type(e), e))
            finally:
                motor_ctrl.stop()
                await asyncio.sleep(1)

        if PULSE_TEST:
            motor_ctrl.set_motor_speed(1, 100)
            motor0 = motor_ctrl.get_motor(0)
            await motor0.measure_ticks_per_second(test_duration=10.0)

        if RAMP_TEST:
            ids = motor_ctrl.motor_ids
            motor_ctrl.enable_rpm_logger(interval_ms=250)
            _delay_ms = 200

            while True:
                motor_ctrl.set_motor_direction('all', Motor.DIRECTION_FORWARD)

                log.info('accelerate forward…')
                await motor_ctrl.accelerate(ids, Motor.FULL_SPEED, step=5, delay_ms=_delay_ms)

                log.info('decelerate…')
                await motor_ctrl.accelerate(ids, Motor.STOPPED, step=5, delay_ms=_delay_ms)

                log.info('stop…')
                motor_ctrl.stop()

                motor_ctrl.set_motor_direction('all', Motor.DIRECTION_REVERSE)

                log.info('accelerate backward…')
                await motor_ctrl.accelerate(ids, Motor.FULL_SPEED, step=5, delay_ms=_delay_ms)

                log.info('decelerate…')
                await motor_ctrl.accelerate(ids, Motor.STOPPED, step=5, delay_ms=_delay_ms)

                log.info('stop…')
                motor_ctrl.stop()

        else:
            log.info('wait...')
            await asyncio.sleep(3)

    except KeyboardInterrupt:
        log.info('Ctrl-C caught, user stopped execution.')
    finally:
        if motor_ctrl:
            motor_ctrl.close()
        log.info('complete.')

if __name__ == '__main__':
    asyncio.run(main())

#EOF
