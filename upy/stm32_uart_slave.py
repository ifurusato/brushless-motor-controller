#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-24
#
# A UART slave for the STM32, using UART 1-4.
#
# STM32H562 (WeActStudio 64-pin CoreBoard) UART Default Pin Mapping
#
#     +--------+-------+-------+
#     | UART   |  TX   |  RX   |
#     +--------+-------+-------+
#     | UART1  |  PA9  | PA10  |
#     | UART2  |  PA2  | PA3   |
#     | UART3  | PB10  | PB11  |
#     | UART4  | PC10  | PC11  |
#     +--------+-------+-------+
#
# Note: You cannot specify tx/rx pins in MicroPython's UART() constructor,
#       it will use the default pins for each.
#
# Example usage:
#
#   uart4 = UART(4, baudrate=115200)  # Uses PC10 (TX), PC11 (RX)
#

import uasyncio as asyncio
import time
from pyb import LED, Pin, UART
from pyb import LED
from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload
from uart_slave_base import UartSlaveBase

class Stm32UartSlave(UartSlaveBase):
    def __init__(self, uart_id=1, baudrate=115200, pixel=None):
        UartSlaveBase.__init__(self, 'stm32-uart', uart_id=uart_id, baudrate=baudrate, pixel=pixel)
        self._led   = LED(1)
        self._uart  = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        # ready

#EOF
