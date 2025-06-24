#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-23

import time
from typing import Callable, Optional
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from uart.async_uart_manager import AsyncUARTManager
from uart.sync_uart_manager import SyncUARTManager
from uart.payload import Payload
from core.logger import Logger, Level

class UARTMaster:

    ERROR_PAYLOAD = Payload("ER", -1.0, -1.0, -1.0, -1.0) # singleton error payload

    def __init__(self, port='/dev/serial0', baudrate=115200):
        self._log = Logger('uart-master', Level.INFO)
        _use_async_uart_manager = False # config?
        if _use_async_uart_manager:
            self.uart = AsyncUARTManager(port=port, baudrate=baudrate)
        else:
            self.uart = SyncUARTManager(port=port, baudrate=baudrate)
        self.uart.open()
        self._log.info('UART master ready at baud rate: {}.'.format(baudrate))

    def send_payload(self, payload):
        '''
        Send a Payload object after converting it to bytes.
        '''
        packet_bytes = payload.to_bytes()
#       self._log.info(f"MASTER TX BYTES: {packet_bytes.hex(' ')}") # TEMP
        self.uart.send_packet(payload)
        self._log.info(Fore.MAGENTA + "master sent: {}".format(payload))

    def receive_payload(self):
        '''
        Receive a Payload object.
        '''
        response_payload = self.uart.receive_packet()
        if response_payload:
            self._log.info(Fore.MAGENTA + "received: {}".format(response_payload))
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
            return self.ERROR_PAYLOAD

    def run(self, source: Optional[Callable[[], int]] = None):
        '''
        Main loop for communication with elapsed time measurement. This is currently
        used for testing but could easily be modified for continuous use.
        '''
        try:
            if source is None:
                print(Fore.GREEN + "source not provided, using counter.")
            else:
                print(Fore.GREEN + "using source for data.")

            count = 0.0

            while True:

                if source is not None:
                    data = source()
                    print("data: '{}'".format(data))
                else:
                    count += 1.0
                    data = count

                start_time = dt.now()
                # create Payload with cmd (2 letters) and floats for pfwd, sfwd, paft, saft
                payload = Payload("MO", data, data, -10.0, -20.0)
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
                # with no sleep here, would be running as fast as the system allows
#               time.sleep(0.25)

        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
        except KeyboardInterrupt:
            self._log.info("ctrl-c caught, exiting…")
        finally:
            self.uart.close()

#EOF
