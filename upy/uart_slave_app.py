#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-07-03
#
# This is the entry point to the UART slave application. This uses UART 2 on the
# STM32 and UART 1 (the default) on the RP2040.
#

#import sys
import traceback
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from config_loader import ConfigLoader
from payload import Payload
from pixel import Pixel
from status import Status
from payload_router import PayloadRouter
from motor_controller import MotorController
from colors import *

import cwd
import free

__IS_PYBOARD = True   # use Pyboard or MP2040

class UartSlaveApp:
    def __init__(self, is_pyboard=True):
        self._is_pyboard = is_pyboard
        self._log = Logger('main', Level.INFO)
        self._pixel    = Pixel(pixel_count=8, brightness=0.1)
        self._status   = Status(self._pixel)
        self._slave    = None
        self._uart_id  = None
        self._verbose  = False
        self._baudrate = 1_000_000 # default
        _config = ConfigLoader.configure('/sd/config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        self._motor_controller = MotorController(config=_config, motors_enabled=(True, True, False, False), level=Level.INFO)
        self._motor_controller.enable()
        self._router   = PayloadRouter(self._status, self._motor_controller)
        self._log.info('ready.')

    def rgb(self, color=None):
        self._pixel.set_color(color)

    def off(self):
        self._status.off()

    async def _pyb_wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        for _ in range(3):
            self.rgb()
            _led.on()
            await asyncio.sleep_ms(50)
            self.off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.off()
        _led.off()

    async def _wait_a_bit(self):
        from machine import Pin
        _led = Pin(11, Pin.OUT)
        for _ in range(3):
            self.rgb()
            _led.on()
            await asyncio.sleep_ms(50)
            self.off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.off()
        _led.off()

    async def _setup_uart_slave(self):
        if self._is_pyboard:
            from stm32_uart_slave import Stm32UartSlave
            await self._pyb_wait_a_bit()
            self._uart_id = 2
            self._log.info('configuring UART{} slave for STM32 Pyboard…'.format(self._uart_id))
            self._slave = Stm32UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, status=self._status)
        
        else:
            from rp2040_uart_slave import RP2040UartSlave
            await self._wait_a_bit()
            self._uart_id = 1
            self._log.info('configuring UART{} slave for RP2040…'.format(self._uart_id))
            self._slave = RP2040UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, status=self._status)

        self._slave.set_verbose(False)
        self._log.info('UART{} slave: '.format(self._uart_id) + Fore.WHITE + 'waiting for command from master…')

    async def run(self):
        await self._setup_uart_slave()
        self._slave.enable()
        try:
            while True:
                _payload = await self._slave.receive_packet()
                if isinstance(_payload, Payload):
                    if self._verbose:
                        self._log.info("payload: {}".format(_payload))
                    if _payload.cmd == 'RS': # request status
                        self._log.info(Fore.MAGENTA + "request status…")
                        code = float(self._router.get_error_code())
                        status_cmd = "ER" if code != 0.0 else "OK"
                        status_payload = Payload(status_cmd, code, 0.0, 0.0, 0.0)
                        await self._slave.send_packet(status_payload)
                        self._router.clear_error()
                    else:
                        # route normal command payload (non-blocking)
                        asyncio.create_task(self._router.route(_payload))
                        self._router.route(_payload)
                        ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                        await self._slave.send_packet(ack_payload)
                elif _payload is not None:
                    if self._verbose:
                        self._log.warning("unknown packet: {} (type: {})".format(_payload, type(_payload)))
                    ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                    await self._slave.send_packet(ack_payload)
                else:
                    self._log.warning("no valid payload received.")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
            traceback.print_exc()
#           sys.print_exception(e)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        self._slave.disable()
        self.off()

# for REPL usage or testing
def exec():
    app = UartSlaveApp(is_pyboard=__IS_PYBOARD)
    asyncio.run(app.run())

if __name__ == "__main__":
    app = UartSlaveApp(is_pyboard=__IS_PYBOARD)
    asyncio.run(app.run())

#EOF
