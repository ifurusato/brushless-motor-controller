#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-26

from hardware.digital_pot_async import DigitalPotentiometer
from hardware.rotary_encoder import RotaryEncoder

class ValueProvider:
    def __call__(self):
        raise NotImplementedError("subclasses must implement __call__()")

    def off(self):
        raise NotImplementedError("subclasses must implement off()")

class DigitalPotSpeedProvider(ValueProvider):

    def __init__(self):
        self._digital_pot = DigitalPotentiometer()
        self._digital_pot.start()

    def __call__(self):
        return self._digital_pot.data

    def close(self):
        self._digital_pot.off()

class RotaryEncoderCommandProvider(ValueProvider):

    def __init__(self):
        self._encoder = RotaryEncoder(i2c_addr=0x0F, multiplier=20, brightness=1.0)
        self._encoder.start()

    def __call__(self):
        mode, hue, r, g, b = self._encoder.update()
        cmd_index = int(mode * 10)
        match cmd_index:
            case 0:
                return 'CO' # color
            case 1:
                return 'EN' # enable
            case 2:
                return 'GO' # go
            case 3:
                return 'RO' # rotate
            case 4:
                return 'CR' # crab
            case 5:
                return 'DS' # disable
            case 6:
                return 'AK' # acknowledge
            case 7:
                return 'ER' # error
            case _:
                return 'CO' # color

    def close(self):
        self._encoder.off()

#EOF
