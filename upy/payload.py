#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-27

import struct
from crc8_table import CRC8_TABLE

class Payload:
    # sync header: 'zz' for human-readability. To switch to a binary header, just uncomment the next line.
    SYNC_HEADER = b'\x7A\x7A'
#   SYNC_HEADER = b'\xAA\x55'
    PACK_FORMAT = '<2sffff'  # 2-char cmd, 4 floats
    PAYLOAD_SIZE = struct.calcsize(PACK_FORMAT) # size of cmd+floats only, no CRC or header
    CRC_SIZE = 1
    PACKET_SIZE = len(SYNC_HEADER) + PAYLOAD_SIZE + CRC_SIZE  # header + payload + crc

    def __init__(self, cmd, pfwd, sfwd, paft, saft):
        self._cmd  = cmd
        self._pfwd = pfwd
        self._sfwd = sfwd
        self._paft = paft
        self._saft = saft

    @property
    def cmd(self):
        return self._cmd

    @property
    def pfwd(self):
        return self._pfwd

    @property
    def sfwd(self):
        return self._sfwd

    @property
    def paft(self):
        return self._paft

    @property
    def saft(self):
        return self._saft

    @property
    def rgb(self):
        return self._pfwd, self._sfwd, self._paft

    @property
    def values(self):
        return self._pfwd, self._sfwd, self._paft, self._saft

    @property
    def speeds(self):
        '''
        An alias for values when they are assumed a four motor speeds.
        '''
        return self.values

    def __repr__(self):
        return "Payload(cmd={:>2}, pfwd={:7.2f}, sfwd={:7.2f}, paft={:7.2f}, saft={:7.2f})".format(
            self._cmd, self._pfwd, self._sfwd, self._paft, self._saft
        )

    def to_bytes(self):
        '''
        A convenience method that calls __bytes__(). This is probably safer as in
        certain cases __bytes__() doesn't get triggered correctly in MicroPython.
        '''
        return self.__bytes__()

    def __bytes__(self):
        # pack the data into bytes (cmd, floats)
        packed = struct.pack(self.PACK_FORMAT, self._cmd, self._pfwd, self._sfwd, self._paft, self._saft)
        crc = self.calculate_crc8(packed)
        return Payload.SYNC_HEADER + packed + bytes([crc])

    @classmethod
    def from_bytes(cls, packet):
        if len(packet) != cls.PACKET_SIZE:
            raise ValueError("invalid packet size: {}".format(len(packet)))
        if packet[:len(Payload.SYNC_HEADER)] != Payload.SYNC_HEADER:
            raise ValueError("invalid sync header")
        data_and_crc = packet[len(Payload.SYNC_HEADER):]
        data, crc = data_and_crc[:-1], data_and_crc[-1]
        calc_crc = cls.calculate_crc8(data)
        if crc != calc_crc:
            raise ValueError("CRC mismatch.")
        cmd, pfwd, sfwd, paft, saft = struct.unpack(cls.PACK_FORMAT, data)
        return cls(cmd.decode('ascii'), pfwd, sfwd, paft, saft)

    @staticmethod
    def calculate_crc8(data: bytes) -> int:
        crc = 0
        for b in data:
            crc = CRC8_TABLE[crc ^ b]
        return crc

#EOF
