#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-23

import uasyncio as asyncio
from colorama import Fore, Style

from payload import Payload
from logger import Logger, Level

_IS_PYBOARD = True

async def pyb_wait_a_bit():
    from pyb import LED
    _led = LED(1)
    for _ in range(3):
        _led.on()
        await asyncio.sleep_ms(50)
        _led.off()
        await asyncio.sleep_ms(950)
    _led.off()

async def wait_a_bit():
    from machine import Pin
    _led = Pin(11, Pin.OUT)
    for _ in range(3):
        _led.on()
        await asyncio.sleep_ms(50)
        _led.off()
        await asyncio.sleep_ms(950)
    _led.off()

async def main():

    _slave = None
    _log = Logger('main', Level.INFO)
    _baudrate = 1_000_000 # 115200 460800 921600 

    # delay the inevitable
    if _IS_PYBOARD:
        _log.info(Fore.GREEN + "configuring UART slave for STM32 Pyboard…")
        from stm32_uart_slave import Stm32UartSlave

        await pyb_wait_a_bit()
        _uart_id = 4
        _slave = Stm32UartSlave(uart_id=_uart_id, baudrate=_baudrate)
    else:
        _log.info(Fore.GREEN + "configuring UART slave for RP2040…")
        from rp2040_uart_slave import RP2040UartSlave

        await wait_a_bit()
        _uart_id = 1
        _slave = RP2040UartSlave(uart_id=_uart_id, baudrate=_baudrate)

    _slave.set_verbose(True)
#   _slave.enable_led(True) # unsupported
    _log.info("UART slave: waiting for command from master…")
    while True:
        packet = await _slave.receive_packet()
        if packet is not None:
#           _log.info(Fore.MAGENTA + "received payload: {}".format(packet))
            # respond with ACK + same payload command but zeroed floats (example)
            ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
            await _slave.send_packet(ack_payload)
        else:
            _log.warning("no valid packet received.")

# for use from the REPL
def exec():
    asyncio.run(main())

if __name__ == "__main__":
    asyncio.run(main())

#EOF
