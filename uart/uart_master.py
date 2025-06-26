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
import traceback
import itertools
from typing import Callable, Optional
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from uart.async_uart_manager import AsyncUARTManager
from uart.sync_uart_manager import SyncUARTManager
from uart.payload import Payload
from hardware.value_provider import DigitalPotValueProvider
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
        self._hindered = False
        self._verbose = False
        self._log.info('UART master ready at baud rate: {}.'.format(baudrate))

    def send_payload(self, payload):
        '''
        Send a Payload object after converting it to bytes.
        '''
        packet_bytes = payload.to_bytes()
#       self._log.info(f"MASTER TX BYTES: {packet_bytes.hex(' ')}") # TEMP
        self.uart.send_packet(payload)
        if self._verbose:
            self._log.info(Fore.MAGENTA + "tx: {}".format(payload))

    def receive_payload(self):
        '''
        Receive a Payload object.
        '''
        response_payload = self.uart.receive_packet()
        if response_payload:
            if self._verbose:
                self._log.info(Fore.MAGENTA + "rx: {}".format(response_payload))
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

    def run(self, 
                speed_source: Optional[Callable[[], int]] = None,
                delay_sec=0):
        '''
        Main loop for communication with elapsed time measurement. This is currently
        used for testing but could easily be modified for continuous use.
        '''
        try:
            if speed_source is None:
                print(Fore.GREEN + "speed source not provided, using counter.")
            else:
                print(Fore.GREEN + "using speed source for data.")

            value = 0.0
            red = green = blue = 0.0
            counter = itertools.count()

            while True:

                if speed_source is not None:
                    value, red, green, blue = speed_source()
                else:
                    value += 1.0

                start_time = dt.now()
                # create Payload with cmd (2 letters) and floats for pfwd, sfwd, paft, saft
                
                payload = Payload("CO", red, green, blue, 0.0)
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
                if next(counter) % 10 == 0: # every 10th time
                    self._log.info("tx: {:.2f} ms elapsed.".format(elapsed_time))
                # with no sleep here, would be running as fast as the system allows
                if self._hindered:
                    time.sleep(delay_sec)

        except KeyboardInterrupt:
            print('\n')
            self._log.info("ctrl-c caught, exiting…")
        except Exception as e:
            self._log.error("{} raised in run loop: {}".format(type(e), e))
            traceback.print_exception(type(e), e, e.__traceback__)
        finally:
            self.uart.close()
            if speed_source:
                if isinstance(speed_source, DigitalPotValueProvider):
                    speed_source.close()
                else:
                    self._log.info("speed source: {}".format(type(speed_source)))

#EOF
