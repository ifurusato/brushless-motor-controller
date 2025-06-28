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

from uart.uart_master import UARTMaster
from hardware.value_provider import RotaryEncoderCommandProvider, DigitalPotSpeedProvider

if __name__ == "__main__":

    _command_provider = RotaryEncoderCommandProvider()
    _speed_provider   = DigitalPotSpeedProvider()

    # instantiate the UARTMaster and run in a loop
    _baudrate = 1_000_000 # 115200 460800 921600
    master = UARTMaster(baudrate=_baudrate)
    master.run(_command_provider, _speed_provider, delay_sec=0.5)

#EOF
