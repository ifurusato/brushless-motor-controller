#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-06-24

from core.logger import Logger, Level

class PayloadRouter:
    def __init__(self, motor_controller):
        self._log = Logger('router', Level.INFO)
        self.motor_controller = motor_controller
        self._errors = 0
        self._log.info('ready.')

    def route(self, payload):
        match payload.command:
            case 'EN':
                self.motor_controller.enable()
            case 'DS':
                self.motor_controller.disable()
            case 'GO':
                self.motor_controller.set_speeds(*payload.values)
            case 'ST':
                self.motor_controller.stop()
            case 'RO':
                self.motor_controller.rotate(*payload.values)
            case 'CR':
                self.motor_controller.crab(*payload.values)
            case 'AK':
                self._log.info('ACK.')
            case 'ER':
                self._errors += 1
                self._log.error('ERROR. count: {}'.format(self._errors))
            case _:
                raise ValueError("unknown command: {}".format(payload.command))

#EOF
