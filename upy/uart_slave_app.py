#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-07-15
#
# This is the entry point to the UART slave application.

import sys
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from config_loader import ConfigLoader
from payload import Payload
from pixel import Pixel
from status import Status
from payload_router import PayloadRouter
from motor_controller import MotorController
from mode import Mode
from colors import *

import cwd
import free

class UartSlaveApp:
    def __init__(self):
        _config = ConfigLoader.configure('config.yaml')
        if _config is None:
            raise ValueError('failed to import configuration.')
        _cfg = _config['kros']['uart_slave_app']
        self._uart_id    = _cfg['uart_id'] # on RP2040 this should be 1
        self._is_pyboard = _cfg['is_pyboard'] 
        self._verbose    = _cfg['verbose']
        self._log = Logger('uart{}_slave'.format(self._uart_id), Level.INFO)
        _display_enabled = _config['kros']['display']['enable']
        if _display_enabled:
            from display import Display
            _rotation = _config['kros']['display']['rotation']
            self._display  = Display(_rotation)
        else:
            from mock_display import Display
            self._display  = Display()
        _pixel_enabled = _config['kros']['pixel']['enable']
        if _pixel_enabled:
            self._pixel = Pixel(_config, pixel_count=8, brightness=0.1)
            self._status   = Status(self._pixel)
        else:
            self._pixel = None
            self._status = None
        self._slave    = None
        self._tx_count = 0
        self._baudrate = 1_000_000 # default
        self._motor_controller = MotorController(config=_config, status=self._status, level=Level.INFO)
        self._router   = PayloadRouter(self._status, self._display, self._motor_controller)
        self._display.hello()
        self._log.info('ready.')

    def rgb(self, color=None):
        if self._pixel:
            self._pixel.set_color(color)

    async def _pyb_wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        for _ in range(3):
            self.rgb()
            _led.on()
            await asyncio.sleep_ms(50)
            if self._status:
                self._status.off()
            _led.off()
            await asyncio.sleep_ms(950)
        if self._status:
            self._status.off()
        _led.off()

    async def _wait_a_bit(self):
        from machine import Pin
        _led = Pin(11, Pin.OUT)
        for _ in range(3):
            self.rgb()
            _led.on()
            await asyncio.sleep_ms(50)
            if self._status:
                self._status.off()
            _led.off()
            await asyncio.sleep_ms(950)
        if self._status:
            self._status.off()
        _led.off()

    async def _setup_uart_slave(self):
        if self._is_pyboard:
            from stm32_uart_slave import Stm32UartSlave
            self._log.info('waiting a bit…')
            await self._pyb_wait_a_bit()
            self._log.info('done waiting.')
            self._log.info('configuring UART{} slave for STM32 Pyboard…'.format(self._uart_id))
            self._slave = Stm32UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, status=self._status)
        else:
            from rp2040_uart_slave import RP2040UartSlave
            await self._wait_a_bit()
            self._log.info('configuring UART{} slave for RP2040…'.format(self._uart_id))
            self._slave = RP2040UartSlave(uart_id=self._uart_id, baudrate=self._baudrate, status=self._status)

        self._slave.set_verbose(False)
        self._motor_controller.enable()
        self._display.ready()
        self._log.info(Fore.GREEN + 'waiting for payload…')

    async def run(self):
        await self._setup_uart_slave()
        self._slave.enable()
        try:
            while True:
                _payload = await self._slave.receive_packet()
                if isinstance(_payload, Payload):
                    self._tx_count += 1
                    if self._verbose:
                        self._log.info("payload: {}".format(_payload))
                    if _payload.code == 'PN': # just ping, no routing
                        await self._slave.send_packet(Payload(Mode.PING.code, *Payload.encode_int(self._tx_count)))
                    elif _payload.code == 'RS': # request status
                        self._log.info(Fore.MAGENTA + "request status…")
                        timestamp, code = self._router.get_error_info()
#                       status_code = "ER" if timestamp is not None else "OK"
                        if timestamp is not None:
                            status_payload = Payload(Mode.ERROR.code, timestamp, code, 0.0, 0.0)
                        else:
                            status_payload = Payload(Mode.ACK.code, 0.0, 0.0, 0.0, 0.0)
                        await self._slave.send_packet(status_payload)
                        self._router.clear_error()
                    else:
                        # route normal command payload (non-blocking)
                        asyncio.create_task(self._router.route(_payload))
                        self._router.route(_payload)
                        ack_payload = Payload(Mode.ACK.code, 0.0, 0.0, 0.0, 0.0)
                        await self._slave.send_packet(ack_payload)
                elif _payload is not None:
                    if self._verbose:
                        self._log.warning("unknown packet: {} (type: {})".format(_payload, type(_payload)))
                    ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                    await self._slave.send_packet(ack_payload)
                else:
                    self._log.warning("no valid payload received.")
        except KeyboardInterrupt:
            self._log.info("Ctrl-C caught, exiting application…")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
            sys.print_exception(e)
        finally:
            self.close()

    def close(self):
        self._log.info('closing…')
        if self._status:
            self._status.off()
        if self._motor_controller:
            self._motor_controller.close()
        if self._slave:
            self._slave.disable()
        if self._display:
            self._display.close()
        self._log.info('closed.')

# for REPL usage or testing
def exec():
    app = None
    try:
        app = UartSlaveApp()
        asyncio.run(app.run())
    except KeyboardInterrupt:
        pass
    finally:
        if app:
            app.close()

if __name__ == "__main__":
    app = None
    try:
        app = UartSlaveApp()
        asyncio.run(app.run())
    except KeyboardInterrupt:
        pass
    finally:
        if app:
            app.close()

#EOF
