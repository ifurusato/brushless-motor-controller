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

from datetime import datetime as dt, timedelta
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from payload import Payload
from pixel import Pixel
from colors import *

class PayloadRouter:
    ACK = Payload("AK", 0.0, 0.0, 0.0, 0.0)
    ERR = Payload("ER", 0.0, 0.0, 0.0, 0.0)

    def __init__(self, status, motor_controller):
        self._log = Logger('router', Level.INFO)
        self._motor_controller = motor_controller
        self._errors   = 0
        self._status   = status
        self._last_cmd = None
        self._verbose  = False
        self._last_error = None
        self._error_time = None
        self._error_timeout = timedelta(seconds=5)  # configurable timeout
        self._check_event_loop()
        self._log.info('ready.')
            
    def _record_error(self, error_msg):
        self._last_error = error_msg
        self._error_time = dt.now()

    def _has_recent_error(self):
        if self._last_error and self._error_time:
            return (dt.now() - self._error_time) < self._error_timeout
        return False

    def clear_error(self):
        self._last_error = None
        self._error_time = None

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

    def off(self):
        self._status.off()

    async def _handle_payload(self, payload):
        cmd = payload.cmd
        if self._verbose:
            if cmd != self._last_cmd:
                self._log.info(Fore.MAGENTA + "route: '{}' from payload: {}".format(payload.cmd, payload))
        if cmd == 'CO':
            if self._verbose:
                rgb = payload.rgb
                if rgb == COLOR_BLACK:
                    self._log.info(Fore.BLACK + 'color: black')
                else:
                    self._log.info(Fore.MAGENTA + 'color: {}'.format(rgb))
            self._status.rgb(color=payload.rgb)
        elif cmd == 'GO':
            self._motor_controller.go(payload.values)
            self._status.motors(payload.values)
        elif cmd == 'ST':
            self._motor_controller.stop()
        elif cmd == 'RO':
            self._motor_controller.rotate(payload.values)
#           self._status.motors(payload.values)
        elif cmd == 'CR':
            self._motor_controller.crab(payload.values)
#           self._status.motors(payload.values)
        elif cmd == 'EN':
            self._motor_controller.enable()
        elif cmd == 'DS':
            self._motor_controller.disable()
            self.off()
        elif cmd == 'ER':
            self._errors += 1
            self._log.error('ERROR. count: {}'.format(self._errors))
        else:
            self._log.error("unknown command: {}".format(payload.cmd))

    async def route(self, payload):
        '''
        Routes the Payload to an appropriate recipient, then returns
        True if successfully routed.
        '''
        if payload is None:
            raise ValueError('null Payload.')
        elif not isinstance(payload, Payload):
            raise TypeError('expected a Payload, not a {}'.format(type(payload)))
        self._last_cmd = payload.cmd
        asyncio.create_task(self._handle_payload(payload))

    def close(self):
        self.off()
        self._log.info('closed.')

#EOF
