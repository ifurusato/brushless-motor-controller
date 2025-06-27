#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-06-27

from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload
from pixel import Pixel
from colors import *

class PayloadRouter:
    def __init__(self, motor_controller):
        self._log = Logger('router', Level.INFO)
        self.motor_controller = motor_controller
        self._errors   = 0
        self._pixel    = Pixel(brightness=0.1)
        self._last_cmd = None
        self._verbose  = True
        self._log.info('ready.')

    def pixel_on(self, color=COLOR_CYAN):
        self._pixel.set_color(color=color)

    def pixel_off(self):
        self._pixel.set_color(color=None)

    def route(self, payload):
        '''
        Routes the Payload to an appropriate recipient, then returns
        True if successfully routed.
        '''
        if payload is None:
            raise ValueError('null Payload.')
        elif not isinstance(payload, Payload):
            raise TypeError('expected a Payload, not a {}'.format(type(payload)))
        cmd = payload.cmd
        if self._verbose:
            if cmd != self._last_cmd:
                self._log.info(Fore.MAGENTA + "route: '{}' from payload: {}".format(payload.cmd, payload))
        if cmd == 'EN':
            self.motor_controller.enable()
        elif cmd == 'DS':
            self.motor_controller.disable()
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
            self.motor_controller.set_speeds(payload.values)
        elif cmd == 'ST':
            self.motor_controller.stop()
        elif cmd == 'RO':
            self.motor_controller.rotate(payload.values)
        elif cmd == 'CR':
            self.motor_controller.crab(payload.values)
        elif cmd == 'AK':
            if self._verbose:
                self._log.info(Fore.MAGENTA + 'ACK.')
        elif cmd == 'ER':
            self._errors += 1
            self._log.error('ERROR. count: {}'.format(self._errors))
            return False
        else:
            self._log.error("unknown command: {}".format(payload.cmd))
            return False
        self._last_cmd = cmd
        return True

    def close(self):
        self.pixel_off()
        self._log.info('closed.')

#EOF
