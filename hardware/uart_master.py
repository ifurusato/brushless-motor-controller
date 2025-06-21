#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-12

import time
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from hardware.async_uart_manager import AsyncUARTManager
from hardware.payload import Payload
from core.logger import Logger, Level

class UARTMaster:

    ERROR_PAYLOAD = Payload("ER", -1.0, -1.0, -1.0, -1.0)  # Singleton error payload

    def __init__(self, port='/dev/serial0', baudrate=115200):
        self._log = Logger('uart-master', Level.INFO)
        self.uart = AsyncUARTManager(port=port, baudrate=baudrate)
        self.uart.open()
        self._log.info('UART master ready at baud rate: {}.'.format(baudrate))

    def send_payload(self, payload):
        '''
        Send a Payload object after converting it to bytes.
        '''
        self.uart.send_packet(payload)
        self._log.debug("sent: {}".format(payload))

    def receive_payload(self):
        '''
        Receive a Payload object.
        '''
        response_payload = self.uart.receive_packet()
        if response_payload:
#           self._log.info("received: {}".format(response_payload))
            return response_payload
        else:
            raise ValueError("no valid response received.")

    def send_receive_payload(self, payload):
        '''
        Accept a Payload, send it, then wait for the response and return the Payload result.
        This method can be used without needing to run the full loop. If an error occurs
        this returns the ERROR_PAYLOAD.
        '''
        self.send_payload(payload)
        try:
            response_payload = self.receive_payload()
            return response_payload
        except ValueError as e:
            self._log.error("error during communication: {}".format(e))
#           return None
            return self.ERROR_PAYLOAD

    def run(self):
        '''
        Main loop for communication with elapsed time measurement. This is currently
        used for testing but could easily be modified for continuous use.
        '''
        try:
            # create Payload with cmd (2 letters) and floats for pfwd, sfwd, paft, saft
            payload = Payload("MO", 10.0, 20.0, -10.0, -20.0)

            while True:
                start_time = dt.now()
                # send the Payload object
                self.send_payload(payload)
                try:
                    self.receive_payload()
                except ValueError as e:
                    self._log.error("error receiving payload: {}:".format(e))
                    continue  # optionally, continue the loop without stopping
                # calculate elapsed time
                end_time = dt.now()
                elapsed_time = (end_time - start_time).total_seconds() * 1000  # Convert to milliseconds
                self._log.info(Fore.GREEN + "tx elapsed: {:.2f} ms".format(elapsed_time))
                # no sleep here, running as fast as the system allows

        except KeyboardInterrupt:
            self._log.info("ctrl-c caught, exiting…")
        finally:
            self.uart.close()

#EOF
