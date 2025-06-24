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

import struct

class Payload:
    PACK_FORMAT = '<2s4f'
    PACKET_SIZE = struct.calcsize(PACK_FORMAT) + 1  # payload + crc

    def __init__(self, cmd: str, pfwd: float, sfwd: float, paft: float, saft: float):
        if len(cmd) != 2 or not cmd.isalpha():
            raise ValueError("cmd must be 2 alphabetic characters")
        self.cmd = cmd.upper().encode('ascii')
        self.pfwd = pfwd
        self.sfwd = sfwd
        self.paft = paft
        self.saft = saft

    def to_bytes(self) -> bytes:
        # pack the data into bytes, including the CRC
        packed = struct.pack(self.PACK_FORMAT, self.cmd, self.pfwd, self.sfwd, self.paft, self.saft)
        crc = self.calculate_crc8(packed)
        return packed + bytes([crc])

    def __bytes__(self) -> bytes:
        # pack the data into bytes, including the CRC
        packed = struct.pack(self.PACK_FORMAT, self.cmd, self.pfwd, self.sfwd, self.paft, self.saft)
        crc = self.calculate_crc8(packed)
        return packed + bytes([crc])

    @classmethod
    def from_bytes(cls, packet: bytes):
        if len(packet) != cls.PACKET_SIZE:
            raise ValueError("invalid packet size: {}".format(len(packet)))
        data, crc = packet[:-1], packet[-1]
        calc_crc = cls.calculate_crc8(data)
        if crc != calc_crc:
            raise ValueError("CRC mismatch.")
        cmd, pfwd, sfwd, paft, saft = struct.unpack(cls.PACK_FORMAT, data)
        return cls(cmd.decode('ascii'), pfwd, sfwd, paft, saft)

    @staticmethod
    def calculate_crc8(data: bytes, poly=0x07, init=0x00) -> int:
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def __repr__(self):
        return "Payload(cmd={}, pfwd={:7.2f}, sfwd={:7.2f}, paft={:7.2f}, saft={:7.2f})".format(
            self.cmd.decode('ascii'), self.pfwd, self.sfwd, self.paft, self.saft)

#EOF
