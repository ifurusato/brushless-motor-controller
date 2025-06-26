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

class ValueProvider:
    def __call__(self):
        raise NotImplementedError("subclasses must implement __call__()")

    def off(self):
        raise NotImplementedError("subclasses must implement off()")

class DigitalPotValueProvider(ValueProvider):

    def __init__(self):
        self._digital_pot = DigitalPotentiometer()
        self._digital_pot.start()

    def __call__(self):
        value, red, green, blue = self._digital_pot.data
        normalized_value = (value * 2) - 1  # scale 0..1 to -1..+1
        return round(100.0 * normalized_value), red, green, blue

    def close(self):
        self._digital_pot.off()

#EOF
