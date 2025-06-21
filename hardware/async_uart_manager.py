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

import serial
import asyncio
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from colorama import init, Fore, Style
init()

from hardware.payload import Payload
from core.logger import Logger, Level

class AsyncUARTManager:
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=1):
        self._log = Logger('async-uart-mgr', Level.INFO)
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.executor = ThreadPoolExecutor(max_workers=1)
        # dedicated asyncio loop and thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()
        self._log.info('ready.')
        
    def open(self):
        if self.ser is None or not self.ser.is_open:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
            self._log.info("serial port {} opened.".format(self.port_name))
            
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self._log.info("serial port closed.")
        self.executor.shutdown(wait=False)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        
    def _send_packet_sync(self, payload):
        packet_bytes = bytes(payload)  # Calls __bytes__ internally or to_bytes explicitly
        self.ser.write(packet_bytes)
        self.ser.flush()
#       self._log.debug(Style.DIM + "sent: {}".format(repr(payload)))

    def send_packet(self, payload):
        '''
        Synchronous wrapper: schedule async send on background loop.
        '''
        future = asyncio.run_coroutine_threadsafe(
            self._send_packet_async(payload), self.loop)
        return future.result()  # wait for completion
        
    async def _send_packet_async(self, payload):
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(self.executor, self._send_packet_sync, payload)
        
    def _receive_packet_sync(self):
        packet = self.ser.read(Payload.PACKET_SIZE)
        if len(packet) < Payload.PACKET_SIZE:
            return None
        try:
            payload = Payload.from_bytes(packet)
            return payload
        except ValueError as e:
            self._log.error("receive error: {}".format(e))
            return None
        
    def receive_packet(self):
        '''
        Synchronous wrapper: schedule async receive on background loop.
        '''
        future = asyncio.run_coroutine_threadsafe(
            self._receive_packet_async(), self.loop)
        return future.result()
        
    async def _receive_packet_async(self):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(self.executor, self._receive_packet_sync)
    
    def receive_values(self):
        '''
        Convenience method to receive a Payload and return the tuple (cmd, pfwd, sfwd, paft, saft).
        '''
        payload = self.receive_packet()
        if payload:
            return (payload.cmd.decode('ascii'), payload.pfwd, payload.sfwd, payload.paft, payload.saft)
        return None

#EOF
