#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-06-26

from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload

class PayloadRouter:
    def __init__(self, motor_controller):
        self._log = Logger('router', Level.INFO)
        self.motor_controller = motor_controller
        self._errors = 0
        self._verbose = False
        self._log.info('ready.')

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
            self._log.info(Fore.MAGENTA + "route payload: '{}' from command '{}'".format(payload, cmd))
        if cmd == 'EN':
            self.motor_controller.enable()
        elif cmd == 'DS':
            self.motor_controller.disable()
        elif cmd == 'CO':
            if self._verbose:
                self._log.info(Fore.RED + 'color: '.format(payload))
        elif cmd == 'GO':
            self.motor_controller.set_speeds(*payload.values)
        elif cmd == 'ST':
            self.motor_controller.stop()
        elif cmd == 'RO':
            self.motor_controller.rotate(*payload.values)
        elif cmd == 'CR':
            self.motor_controller.crab(*payload.values)
        elif cmd == 'AK':
            if self._verbose:
                self._log.info('ACK.')
        elif cmd == 'ER':
            self._errors += 1
            self._log.error('ERROR. count: {}'.format(self._errors))
            return False
        else:
            self._log.error("unknown command: {}".format(payload.cmd))
            return False
        return True

#EOF
