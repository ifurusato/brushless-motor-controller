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

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ControllerChannel(Enum):
    '''
    An enumeration of the 12 channels of the TLC59711 PWM controller.
    '''
    CHANNEL_0 = ("R0", 3)
    CHANNEL_1 = ("R1", 2)
    CHANNEL_2 = ("R2", 1)
    CHANNEL_3 = ("R3", 0)

    def __init__(self, pin, channel):
        self._pin = pin
        self._channel = channel

    @property
    def pin(self):
        '''
        Return the pin name on the TCL59711.
        '''
        return self._pin

    @property
    def channel(self):
        '''
        Return the integer value of the enum member.
        '''
        return self._channel

    @staticmethod
    def from_string(pin_name: str) -> "ControllerChannel":
        '''
        Convert a string pin name (e.g., 'R0') to a ControllerChannel enum member.
        '''
        for member in ControllerChannel:
            if member.pin == pin_name:
                return member
        raise ValueError(f"Unknown ControllerChannel pin name: {pin_name}")

#EOF
