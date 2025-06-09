#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-09
# modified: 2025-06-09
#

from enum import Enum
from typing import Any, Optional
from hardware.pwm_controller import HardwarePWMController, SoftwarePWMController
from hardware.tlc59711_pwm_controller import TLC59711PWMController
from hardware.tlc59711 import TLC59711
from hardware.tlc59711_pwm_controller import ControllerChannel

class PWMControllerImpl(Enum):
    SOFTWARE_CONTROLLER = "software"
    HARDWARE_CONTROLLER = "hardware"
    TLC59711_CONTROLLER = "tlc59711"
    '''
    An enumeration of the implementing classes of the PWMController API,
    with a factory method and a utility to return the member matching its name.
    '''
    def create(
            self, *,
            pi: Optional[Any],
            config: Optional[dict],
            pin: Optional[int],
            freq: Optional[int],
            tlc_controller: Optional[TLC59711] = None,
            channel: Optional[ControllerChannel] = None,
            level: Any = None):
        match self:
            case PWMControllerImpl.SOFTWARE_CONTROLLER:
                return SoftwarePWMController(pi, pin, freq)
            case PWMControllerImpl.HARDWARE_CONTROLLER:
                return HardwarePWMController(pi, pin, freq)
            case PWMControllerImpl.TLC59711_CONTROLLER:
                if tlc_controller is None or channel is None:
                    raise ValueError("TLC59711 controller and channel must be provided.")
                return TLC59711PWMController(pi=pi, config=config, pwm_controller=tlc_controller, channel=channel, level=level)
            case _:
                raise ValueError("unsupported PWMControllerImpl: {}".format(self))

    @staticmethod
    def from_string(name: str) -> "PWMControllerImpl":
        '''
        Convert a string representation to a PWMControllerImpl enum member.
        '''
        for member in PWMControllerImpl:
            if member.value == name:
                return member
        raise ValueError("unrecognised PWMController implementation name: {}".format(name))

#EOF
