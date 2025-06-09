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
# A handy utility to stop a runaway motor. This uses the YAML configuration
# so it should use the same mechanism to talk to the motor as everybody else.
#

import traceback
import pigpio
import spidev
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.controller_channel import ControllerChannel
from hardware.pwm_controller_impl import PWMControllerImpl
from hardware.brushless_motor import BrushlessMotor
from hardware.tlc59711 import TLC59711

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger('main', level=Level.INFO)
_pi          = None
_motor       = None
try:
    _pi = pigpio.pi()
    if not _pi.connected:
        _log.info("Failed to connect to pigpio daemon")
        exit()
    _config = ConfigLoader(Level.INFO).configure('motor-config.yaml')
    _cfg = _config['kros'].get('hardware').get('motor_controller')
    _closed_loop_enabled = _cfg.get('closed_loop_enabled')
    _controller_impl = PWMControllerImpl.from_string(_cfg.get('pwm_controller_impl', 'hardware')) # "software" | "hardware" | "tlc59711"
    # we don't know if we need all this machinery, but we provide it just in case the config wants it
    _spi = spidev.SpiDev()
    _spi.open(0, 0)
    _spi.max_speed_hz = 1000000
    _tlc = TLC59711(_spi)
    _channel = ControllerChannel.from_string(_cfg.get('controller_channel', 'R0')) # ControllerChannel.CHANNEL_0
    _motor = BrushlessMotor(pi=_pi, 
            name=_channel.pin, 
            config=_config, 
            closed_loop_enabled=_closed_loop_enabled,
            pwm_impl=_controller_impl,
            tlc_controller=_tlc,
            channel=_channel, 
            level=Level.INFO)
    _motor.stop()

except KeyboardInterrupt:
    print('')
    _log.info(Fore.YELLOW + 'ctrl-C caught, exiting…')
except Exception as e:
    _log.error('{} raised by motor controller: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _motor:
        _log.info("stopping motor…")
        _motor.close()
    if _pi:
        _pi.stop()
        _log.info("pigpio connection stopped.")
    _log.info("complete.")

#EOF
