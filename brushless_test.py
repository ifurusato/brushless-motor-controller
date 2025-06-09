#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-02
# modified: 2025-06-10
#
# This is the main test for the DC Brushless Motor. It uses the YAML
# configuration file and supports all three PWM controller implementations.
# This has an option to use a Pimoroni Digital Potentiometer to set the 
# motor speed. If not used the test accelerates the motor to full speed
# and then in the other direction.
#
# Setting the verbose flag true in configuration will provide a lot more
# information when running.
#

import time
import traceback
import pigpio
import spidev
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.controller_channel import ControllerChannel
from hardware.pwm_controller_impl import PWMControllerImpl
from hardware.brushless_motor import BrushlessMotor
from hardware.digital_pot_async import DigitalPotentiometer
from hardware.tlc59711 import TLC59711

USE_DIGITAL_POT      = False  # if available use the digital potentiometer
TARGET_DISTANCE_TEST = False  # untested as of yet

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def read_scaled_speed(digital_pot):
    normalized_value = (digital_pot.value * 2) - 1  # scale 0..1 to -1..+1
    speed = round(100.0 * normalized_value)
    return speed

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈


_log = Logger('main', level=Level.INFO)
_pi          = None
_motor       = None
_digital_pot = None

try:

    _pi = pigpio.pi()
    if not _pi.connected:
        _log.info("Failed to connect to pigpio daemon")
        exit()
    _config = ConfigLoader(Level.INFO).configure('motor-config.yaml')
    _cfg = _config['kros'].get('hardware').get('motor_controller')
    _controller_impl = PWMControllerImpl.from_string(_cfg.get('pwm_controller_impl', 'hardware')) # "software" | "hardware" | "tlc59711"
    _closed_loop_enabled = _cfg.get('closed_loop_enabled')
    _motor_max_rpm = _cfg.get('motor_max_rpm', 159)

    match _controller_impl:
        case PWMControllerImpl.SOFTWARE_CONTROLLER:
            _log.info("using software-based PWM.")
            _motor = BrushlessMotor(
                    pi=_pi,
                    config=_config,
                    closed_loop_enabled=_closed_loop_enabled,
                    pwm_impl=PWMControllerImpl.SOFTWARE_CONTROLLER)

        case PWMControllerImpl.HARDWARE_CONTROLLER:
            _log.info("using hardware-based PWM.")
            _motor = BrushlessMotor(
                    pi=_pi,
                    config=_config,
                    closed_loop_enabled=_closed_loop_enabled,
                    pwm_impl=PWMControllerImpl.HARDWARE_CONTROLLER)

        case PWMControllerImpl.TLC59711_CONTROLLER:
            _log.info("using TLC59711-based PWM.")
            _spi = spidev.SpiDev()
            _spi.open(0, 0)
            _spi.max_speed_hz = 1000000
            _tlc = TLC59711(_spi)
            _channel = ControllerChannel.from_string(_cfg.get('controller_channel', 'R0')) # ControllerChannel.CHANNEL_0
            _motor = BrushlessMotor(pi=_pi, 
                    name=_channel.pin, 
                    config=_config, 
                    closed_loop_enabled=_closed_loop_enabled,
                    pwm_impl=PWMControllerImpl.TLC59711_CONTROLLER, 
                    tlc_controller=_tlc,
                    channel=_channel, 
                    level=Level.INFO)

        case _:
            raise ValueError(f"Unhandled PWM controller implementation: {pwm_impl}")

    if USE_DIGITAL_POT:
        _digital_pot = DigitalPotentiometer()
        _digital_pot.start()
        if TARGET_DISTANCE_TEST: # not currently tested/functional
            _target_distance_mm = 500
            if _target_distance_mm > 0:
                speed = read_scaled_speed(_digital_pot)
                _motor.set_speed(speed, _target_distance_mm)
        else:
            while True:
                speed = read_scaled_speed(_digital_pot)
                if speed == 0.0:
                    _digital_pot.off()
                _motor.set_speed(speed)
                time.sleep(0.2)
    else:
        _loop_delay_sec = 0.025
        _start_time = dt.now()
        for speed in range(0, 101, 1):
            _motor.set_speed(speed)
            time.sleep(_loop_delay_sec)
        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        _log.info('reached positive mid-point complete in {} loop mode: elapsed: {:d}ms'.format(('closed' if _closed_loop_enabled else 'open'), _elapsed_ms))
        time.sleep(1.0)
        for speed in range(100, 0, -1):
            _motor.set_speed(speed)
            time.sleep(_loop_delay_sec)
        _motor.stop()
        time.sleep(0.5)
        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        _log.info('reached stop after ramp in {} loop mode: elapsed: {:d}ms'.format(('closed' if _closed_loop_enabled else 'open'), _elapsed_ms))
        for speed in range(0, -101, -1):
            _motor.set_speed(speed)
            time.sleep(_loop_delay_sec)
        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        _log.info('reached negative mid-point complete in {} loop mode: elapsed: {:d}ms'.format(('closed' if _closed_loop_enabled else 'open'), _elapsed_ms))
        for speed in range(-100, 0, 1):
            _motor.set_speed(speed)
            time.sleep(_loop_delay_sec)
        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        _log.info('ramp complete in {} loop mode: elapsed: {:d}ms'.format(('closed' if _closed_loop_enabled else 'open'), _elapsed_ms))
        _motor.stop()
        time.sleep(1.0)

except KeyboardInterrupt:
    print('')
    _log.info(Fore.YELLOW + 'ctrl-C caught, exiting…')
except Exception as e:
    _log.error('{} raised by motor controller: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _digital_pot:
        _digital_pot.stop()
    if _motor:
        _log.info("stopping motor…")
        _motor.close()
    if _pi:
        _pi.stop()
        _log.info("pigpio connection stopped.")
    _log.info("complete.")

#EOF
