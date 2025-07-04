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
from mode import Mode
from pixel import Pixel
from colors import *

class PayloadRouter:
    ACK = Payload("AK", 0.0, 0.0, 0.0, 0.0)
    ERR = Payload("ER", 0.0, 0.0, 0.0, 0.0)
    NO_ERROR = 0.0

    def __init__(self, status, motor_controller):
        self._log = Logger('router', Level.INFO)
        self._motor_controller = motor_controller
        self._errors   = 0
        self._status   = status
        self._last_cmd = None
        self._verbose  = False
        self._last_error = PayloadRouter.NO_ERROR
        self._error_time = None
        self._error_timeout = timedelta(seconds=5)  # configurable timeout
        # define command handlers
        self._handlers = {
            # non-motion handlers
            Mode.COLOR:     lambda payload: self._handle_color(payload.rgb),                           # color
            Mode.ACK:       lambda payload: self._handle_ack(),                                        # acknowledge (for completeness)
            Mode.ENABLE:    lambda payload: self._motor_controller.enable(),                           # enable
#           Mode.DISABLE:   lambda payload: self._motor_controller.disable(),                          # disable
            Mode.DISABLE:   lambda payload: self._handle_disable(),                                    # mock disable
            Mode.REQUEST:   lambda payload: self._handle_request(payload),                             # request status
            Mode.ERROR:     lambda payload: self._handle_error(payload),                               # error state
            # all wheels forward/backward
            Mode.GO:        lambda payload: self._motor_controller.go(Mode.GO, payload.speeds),        # go forward or reverse
            # rotation (spin in place)
            Mode.ROT_CW:    lambda payload: self._motor_controller.go(Mode.ROT_CW, payload.speeds),    # rotate clockwise
            Mode.ROT_CCW:   lambda payload: self._motor_controller.go(Mode.ROT_CCW, payload.speeds),   # rotate counter-clockwise
            # crab movement (lateral strafe)  
            Mode.CRAB_PORT: lambda payload: self._motor_controller.go(Mode.CRAB_PORT, payload.speeds), # crab to port
            Mode.CRAB_STBD: lambda payload: self._motor_controller.go(Mode.CRAB_STBD, payload.speeds), # crab to starboard
            # crab movement (lateral strafe)  
            Mode.DIA_PFWD:  lambda payload: self._motor_controller.go(Mode.DIA_PFWD, payload.speeds),  # diagonal forward to port
            Mode.DIA_SFWD:  lambda payload: self._motor_controller.go(Mode.DIA_SFWD, payload.speeds),  # diagonal forward to starboard
            Mode.DIA_PREV:  lambda payload: self._motor_controller.go(Mode.DIA_PREV, payload.speeds),  # diagonal reverse to port
            Mode.DIA_SREV:  lambda payload: self._motor_controller.go(Mode.DIA_SREV, payload.speeds),  # diagonal reverse to starboard
            # stopped   
            Mode.STOP:      lambda payload: self._motor_controller.go(Mode.STOP, payload.speeds),      # stop
        }
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
        self._log.info(Style.DIM + 'noop: asyncio loop is running.')
        pass

    def get_error_info(self):
        '''
        Returns two floats: a timestamp (as a float) followed by an error number.
        If there has been no error the timestamp will be None, the error number 0.0.
        '''
        _error_time = self._error_time.timestamp() if self._error_time is not None else None
        return ( _error_time, self._last_error )

    def _record_error(self, error_msg):
        self._last_error = error_msg
        self._error_time = dt.now()

    def _has_recent_error(self):
        if self._last_error and self._error_time:
            return (dt.now() - self._error_time) < self._error_timeout
        return False

    def clear_error(self):
        self._last_error = PayloadRouter.NO_ERROR
        self._error_time = None

    def off(self):
        self._status.off()

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

    async def _handle_payload(self, payload):
        cmd = payload.cmd
        if self._verbose:
            if cmd != self._last_cmd:
                self._log.info(Fore.MAGENTA + "route cmd: '{}' from payload: {}".format(cmd, payload))
        mode = Mode.from_code(cmd)
        handler = self._handlers.get(mode)
        if handler:
            handler(payload)
        else:
            raise ValueError('unhandled payload: {}'.format(payload))

    def _handle_color(self, rgb):
        if self._verbose:
            if rgb == COLOR_BLACK:
                self._log.info(Fore.BLACK + 'color: black')
            else:
                self._log.info(Fore.MAGENTA + 'color: {}'.format(rgb))
        self._status.rgb(color=rgb)

    def _handle_ack(self):
        self._log.info(Fore.GREEN + 'ACK')
        pass

    def _handle_disable(self):
        '''
        Mocked disable (we don't actually disable remotely).
        '''
        self._log.info(Style.DIM + 'disable (mock)')
        pass

    def _handle_request(self, payload):
        self._log.info('REQUEST. payload: {}'.format(payload))
        pass

    def _handle_error(self, payload):
        self._errors += 1
        self._log.error('ERROR. count: {}'.format(self._errors))

    async def x_handle_payload(self, payload):
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

    def close(self):
        self.off()
        self._log.info('closed.')

#EOF
