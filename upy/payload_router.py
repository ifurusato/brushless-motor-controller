#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-06-29

import uasyncio as asyncio
from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload
from pixel import Pixel
from colors import *

class PayloadRouter:
    def __init__(self, motor_controller):
        self._log = Logger('router', Level.INFO)
        self._motor_controller = motor_controller
        self._errors   = 0
        self._pixel    = Pixel(brightness=0.1)
        self._last_cmd = None
        self._verbose  = False
        self._motor_numbers = [1, 2, 3, 4]
        self._check_event_loop()
        self._log.info('ready.')
            
    def _check_event_loop(self):
        try:
            loop = asyncio.get_event_loop()
            # try scheduling a no-op coroutine to confirm the loop is "usable"
            loop.create_task(self._noop())
            self._log.info('asyncio loop is running.')
        except Exception:
            self._log.warning('no asyncio event loop running: payloads will not be routed.')
        
    async def _noop(self):
        pass

    def pixel_on(self, color=COLOR_CYAN):
        self._pixel.set_color(color=color)

    def pixel_off(self):
        self._pixel.set_color(color=None)

    async def _handle_payload(self, payload):
        cmd = payload.cmd
        if self._verbose:
            if cmd != self._last_cmd:
                self._log.info(Fore.MAGENTA + "route: '{}' from payload: {}".format(payload.cmd, payload))
        if cmd == 'EN':
            self._motor_controller.enable()
        elif cmd == 'DS':
            self._motor_controller.disable()
            self.pixel_off()
        elif cmd == 'CO':
            if self._verbose:
                rgb = payload.rgb
                if rgb == COLOR_BLACK:
                    self._log.info(Fore.BLACK + 'color: black')
                else:
                    self._log.info(Fore.MAGENTA + 'color: {}'.format(rgb))
            self.pixel_on(color=payload.rgb)
        elif cmd == 'GO':
            self._motor_controller.go(payload.values)
        elif cmd == 'RO':
            self._motor_controller.rotate(payload.values)
        elif cmd == 'CR':
            self._motor_controller.crab(payload.values)
        elif cmd == 'ST':
            self._motor_controller.stop()
        elif cmd == 'AK':
            if self._verbose:
                self._log.info(Fore.MAGENTA + 'ACK.')
        elif cmd == 'ER':
            self._errors += 1
            self._log.error('ERROR. count: {}'.format(self._errors))
        else:
            self._log.error("unknown command: {}".format(payload.cmd))

    def route(self, payload):
        '''
        Routes the Payload to an appropriate recipient, then returns
        True if successfully routed.
        '''
        if payload is None:
            raise ValueError('null Payload.')
        elif not isinstance(payload, Payload):
            raise TypeError('expected a Payload, not a {}'.format(type(payload)))
        asyncio.create_task(self._handle_payload(payload))
        self._last_cmd = payload.cmd
        return True

    def close(self):
        self.pixel_off()
        self._log.info('closed.')

#EOF
