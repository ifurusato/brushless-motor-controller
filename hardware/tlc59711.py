#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-02
# modified: 2025-06-09
#
# This is a CPython module for the TLC59711 16-bit 12 channel PWM LED Driver,
# originally designed to drive four RGB LEDs with 16-bit PWM per Color.
#
# This is a highly-simplified derivation (18% the size of the original source)
# for single-channel PWM control of a brushless motor. The original source by
# Tony DiCola and Stefan Kruger is distributed under the MIT license (see
# LICENSE-tlc59711.txt). For detailed documentation, see the original.
#
# Product page:
#
#   Adafruit 12-Channel 16-bit PWM LED Driver - SPI Interface - TLC59711
#   <https://www.adafruit.com/product/1455>`_ (Product ID: 1455)
#

import struct
import spidev
from typing import Dict, List, Optional, Tuple

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_CHIP_BUFFER_BYTE_COUNT = 28
_COLORS_PER_PIXEL = 3
_PIXEL_PER_CHIP   = 4
_CHANNEL_PER_CHIP = _COLORS_PER_PIXEL * _PIXEL_PER_CHIP
_BUFFER_BYTES_PER_COLOR = 2
_BUFFER_BYTES_PER_PIXEL = _BUFFER_BYTES_PER_COLOR * _COLORS_PER_PIXEL
_BC_CHIP_BUFFER_BIT_OFFSET = 0
_BC_FIELDS = {
    "BCR": { "offset":  0, "length": 7, "mask": 0b01111111},
    "BCG": { "offset":  7, "length": 7, "mask": 0b01111111},
    "BCB": { "offset": 14, "length": 7, "mask": 0b01111111},
}
_BC_BIT_COUNT = 3 * 7
_FC_CHIP_BUFFER_BIT_OFFSET = _BC_BIT_COUNT
_FC_BIT_COUNT = 5
_FC_FIELDS = {
    "BLANK":  {"offset": 0, "length": 1, "mask": 0b1},
    "DSPRPT": {"offset": 1, "length": 1, "mask": 0b1},
    "TMGRST": {"offset": 2, "length": 1, "mask": 0b1},
    "EXTGCK": {"offset": 3, "length": 1, "mask": 0b1},
    "OUTTMG": {"offset": 4, "length": 1, "mask": 0b1},
}
_WC_CHIP_BUFFER_BIT_OFFSET = _FC_BIT_COUNT + _BC_BIT_COUNT
_WC_BIT_COUNT = 6
_WC_FIELDS = {"WRITE_COMMAND": {"offset": 0, "length": 6, "mask": 0b111111}}
_WRITE_COMMAND = 0b100101
_CHIP_BUFFER_HEADER_BIT_COUNT = _WC_BIT_COUNT + _FC_BIT_COUNT + _BC_BIT_COUNT
_CHIP_BUFFER_HEADER_BYTE_COUNT = _CHIP_BUFFER_HEADER_BIT_COUNT // 8

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class TLC59711:
    '''
    :param spi:  an instance of the SPI bus connected to the chip.
    '''
    def __init__(self, spi: spidev.SpiDev) -> None:
        self.spi = spi
        self.pixel_count = 4
        self.channel_count = self.pixel_count * _COLORS_PER_PIXEL
        self.chip_count = self.pixel_count // 4
        self._buffer = bytearray(_CHIP_BUFFER_BYTE_COUNT * self.chip_count)
        # initialize the brightness channel values to max (these are 7-bit values).
        self.bcr = 127
        self.bcg = 127
        self.bcb = 127
        # initialize external user-facing state for the function control bits of the chip.
        self.outtmg = True
        self.extgck = False
        self.tmgrst = True
        self.dsprpt = True
        self.blank = False
        # now initialize buffer to default values
        self._init_buffer()
        self._buffer_index_lookuptable = []
        self._init_lookuptable()

    def _init_buffer(self) -> None:
        for chip_index in range(self.chip_count):
            self.chip_set_BCData(chip_index, bcr=self.bcr, bcg=self.bcg, bcb=self.bcb)
            self._chip_set_FunctionControl(chip_index)
            self._chip_set_WriteCommand(chip_index)

    def set_chipheader_bits_in_buffer(self, *, chip_index: int = 0, part_bit_offset: int = 0, field: Optional[Dict[str, int]] = None, value: int = 0,) -> None:
        '''
        Set chip header bits in buffer.
        '''
        if field is None:
            field = {"mask": 0, "length": 0, "offset": 0}
        offset = part_bit_offset + field["offset"]
        value &= field["mask"]
        value = value << offset
        header_start = chip_index * _CHIP_BUFFER_BYTE_COUNT
        header = struct.unpack_from(">I", self._buffer, header_start)[0]
        mask = field["mask"] << offset
        header &= ~mask
        header |= value
        struct.pack_into(">I", self._buffer, header_start, header)

    def chip_set_BCData(self, chip_index: int, bcr: int = 127, bcg: int = 127, bcb: int = 127) -> None:
        '''
        Set BC-Data.
        '''
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET, field=_BC_FIELDS["BCR"], value=bcr)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET, field=_BC_FIELDS["BCG"], value=bcg)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET, field=_BC_FIELDS["BCB"], value=bcb)

    def _chip_set_FunctionControl(self, chip_index: int) -> None:
        '''
        Set Function Control Bits in Buffer.
        '''
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET, field=_FC_FIELDS["OUTTMG"], value=self.outtmg)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET, field=_FC_FIELDS["EXTGCK"], value=self.extgck)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET, field=_FC_FIELDS["TMGRST"], value=self.tmgrst)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET, field=_FC_FIELDS["DSPRPT"], value=self.dsprpt)
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET, field=_FC_FIELDS["BLANK"], value=self.blank)

    def _chip_set_WriteCommand(self, chip_index: int) -> None:
        '''
        Set WRITE_COMMAND.
        '''
        self.set_chipheader_bits_in_buffer(chip_index=chip_index, part_bit_offset=_WC_CHIP_BUFFER_BIT_OFFSET, field=_WC_FIELDS["WRITE_COMMAND"], value=_WRITE_COMMAND)

    def _init_lookuptable(self) -> None:
        for channel_index in range(self.channel_count):
            buffer_index = (_CHIP_BUFFER_BYTE_COUNT // _BUFFER_BYTES_PER_COLOR) * ( channel_index // _CHANNEL_PER_CHIP) + channel_index % _CHANNEL_PER_CHIP
            buffer_index *= _BUFFER_BYTES_PER_COLOR
            buffer_index += _CHIP_BUFFER_HEADER_BYTE_COUNT
            self._buffer_index_lookuptable.append(buffer_index)

    def set_pixel_16bit_value(self, pixel_index: int, value_r: int, value_g: int, value_b: int) -> None:
        '''
        Set the value for pixel.
        '''
        # struct 157ms to 16ms (@144pixel on ItsyBitsy M4)
        pixel_start = pixel_index * _COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_b & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_g & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_r & 0xFF

    @staticmethod
    def _check_and_convert(value: List[int]):
        if isinstance(value[0], float):
            if not 0.0 <= value[0] <= 1.0:
                raise ValueError(f"value[0] {value[0]} not in range: 0..1")
            value[0] = int(value[0] * 65535)
        elif not 0 <= value[0] <= 65535:
            raise ValueError(f"value[0] {value[0]} not in range: 0..65535")
        if isinstance(value[1], float):
            if not 0.0 <= value[1] <= 1.0:
                raise ValueError(f"value[1] {value[1]} not in range: 0..1")
            value[1] = int(value[1] * 65535)
        elif not 0 <= value[1] <= 65535:
            raise ValueError(f"value[1] {value[1]} not in range: 0..65535")
        if isinstance(value[2], float):
            if not 0.0 <= value[2] <= 1.0:
                raise ValueError(f"value[2] {value[2]} not in range: 0..1")
            value[2] = int(value[2] * 65535)
        elif not 0 <= value[2] <= 65535:
            raise ValueError(f"value[2] {value[2]} not in range: 0..65535")

    def __setitem__(self, key: int, value: Tuple[float, float, float]) -> None:
        '''
        Set the R, G, B values for the pixel.

        :param int key: 0..(pixel_count)
        :param tuple value: 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        '''
        if 0 <= key < self.pixel_count:
            value = list(value)
            if len(value) != _COLORS_PER_PIXEL:
                raise IndexError("length of value {} does not match _COLORS_PER_PIXEL = {}".format(len(value), _COLORS_PER_PIXEL))
            self._check_and_convert(value)
            self.set_pixel_16bit_value(key, value[0], value[1], value[2])
        else:
            raise IndexError(f"index {key} out of range [0..{self.pixel_count}]")
        self._write()

    def _write(self) -> None:
        '''
        Write the current buffer to the SPI device.
        '''
        self.spi.xfer2(self._buffer)  # SPI transfer

    def stop(self):
        pass

#EOF
