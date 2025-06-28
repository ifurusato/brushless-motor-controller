#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-08
# modified: 2025-06-09
#

import time
from spidev import SpiDev
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tlc59711 import TLC59711
from hardware.controller_channel import ControllerChannel
from hardware.tlc59711_pwm_controller import TLC59711PWMController

if __name__ == '__main__':

    controller = None
    pwm_controller = None
    spi = None

    _log = Logger('main', Level.INFO)
    try:

        _config = ConfigLoader(Level.INFO).configure('motor-config.yaml')

        # initialize the SPI interface for TLC59711
        spi = SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 1000000
        # create an instance of TLC59711 (reusable across muliple controllers)
        pwm_controller = TLC59711(spi)

        # initialize the PWM controller with selected channel
        _channel = ControllerChannel.CHANNEL_0
        controller = TLC59711PWMController(pi=None, config=_config, pwm_controller=pwm_controller, channel=_channel)
        _log.info("configuring motor controller for channel {}, pin {}…".format(_channel.channel, _channel.pin))

        # ramp motor speed from 0% to 100% and then back
        _log.info("ramp-up phase:")
        for speed in range(0, 101, 1):
            controller.set_pwm(speed)
            time.sleep(0.03)
        _log.info("ramp-down phase:")
        for speed in range(100, -1, -1):
            controller.set_pwm(speed)
            time.sleep(0.03)
        # stop the motor after ramping down
        controller.stop()

    except KeyboardInterrupt:
        _log.info('caught Ctrl-C, exiting…')
    except Exception as e:
        _log.error('{} raised: {}'.format(type(e), e))
    finally:
        if pwm_controller:
            pwm_controller.stop()
        if controller:
            controller.stop() # ensure we stop the motor if controller was initialized
        if spi:
            spi.close()

#EOF
