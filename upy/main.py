#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-06-25
#
# This is the entry point to the UART slave application.
#

import uasyncio as asyncio
from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload
from pixel import Pixel
from payload_router import PayloadRouter
from fake_motor import FakeMotor
from colors import *

import free

class UartSlaveApp:
    def __init__(self, is_pyboard=True):
        self._is_pyboard = is_pyboard
        self._log = Logger('main', Level.INFO)
        self._pixel    = Pixel(brightness=0.1)
        self._slave    = None
        self._uart_id  = None
        self._verbose  = False
        self._baudrate = 1_000_000 # default: can be configured if needed
        self._motor_controller = FakeMotor() # for now
        self._router   = PayloadRouter(self._motor_controller)
        self._log.info('ready.')

    def pixel_on(self, color=None):
        self._pixel.set_color(color=COLOR_CYAN if color == None else color)

    def pixel_off(self):
        self._pixel.set_color(color=None)

    async def _pyb_wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        for _ in range(3):
            self.pixel_on()
            _led.on()
            await asyncio.sleep_ms(50)
            self.pixel_off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.pixel_off()
        _led.off()

    async def _wait_a_bit(self):
        from machine import Pin
        _led = Pin(11, Pin.OUT)
        for _ in range(3):
            self.pixel_on()
            _led.on()
            await asyncio.sleep_ms(50)
            self.pixel_off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.pixel_off()
        _led.off()

    async def _setup_uart_slave(self):
        if self._is_pyboard:
            self._log.info("configuring UART slave for STM32 Pyboard…")
            from stm32_uart_slave import Stm32UartSlave
            await self._pyb_wait_a_bit()
            self._uart_id = 4
            self._slave = Stm32UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, pixel=self._pixel)

        else:
            self._log.info("configuring UART slave for RP2040…")
            from rp2040_uart_slave import RP2040UartSlave
            await self._wait_a_bit()
            self._uart_id = 1
            self._slave = RP2040UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, pixel=self._pixel)

        self._slave.set_verbose(False)
        self._log.info('UART slave: ' + Fore.WHITE + 'waiting for command from master…')

    async def run(self):
        await self._setup_uart_slave()
        self._slave.enable()
        try:
            while True:
                _payload = await self._slave.receive_packet()
                if isinstance(_payload , Payload):
                    if self._verbose:
                        self._log.info("payload: {}".format(_payload))
                    self._router.route(_payload)
                    ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                    await self._slave.send_packet(ack_payload)
                elif _payload is not None:
                    if self._verbose:
                        self._log.info("packet: {} (type: {})".format(_payload, type(_payload)))
                    ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                    await self._slave.send_packet(ack_payload)
                else:
                    self._log.warning("no valid payload received.")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        self._slave.disable()
        self.pixel_off()

# for REPL usage or testing
def exec():
    app = UartSlaveApp(is_pyboard=True)
    asyncio.run(app.run())

if __name__ == "__main__":
    app = UartSlaveApp(is_pyboard=True)
    asyncio.run(app.run())

#EOF
