#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-11
# modified: 2025-07-11
#
# mount external flash on spi1 on stm32h723
# used winbond from https://github.com/brainelectronics/micropython-winbond
# mpremote connect /dev/cu.usbmodem3054386A35322 mip install "github:brainelectronics/micropython-winbond"

import os
from machine import SPI, Pin
from winbond import W25QFlash

from colorama import Fore, Style
from logger import Logger, Level


class FlashMount:
    def __init__(self, mount_point='/external', write_enable=False, level=Level.INFO):
        self._log = Logger('flash-mount', level=level)
        self._flash = W25QFlash(spi=SPI(1), cs=Pin('D6'), baud=2000000, software_reset=True)
        self._flash_mount_point = mount_point
        self._write_enable = write_enable
        self._log.info('ready.')

    def _mkfs(self):
        if self._write_enable:
            try:
                self._log.info('creating filesystem for external flash…')
                self._log.info(Style.DIM + 'this might take up to 10 seconds.')
                os.VfsFat.mkfs(self._flash)
            except Exception as e:
                self._log.error('{} raised by mkfs: {}'.format(type(e), e))
        else:
            self._log.warning('external flash mkfs is not enabled.')

    def _format_flash(self):
        # takes some seconds/minutes (approx. 40 sec for 128MBit/16MB)
        if self._write_enable:
            try:
                self._log.info(Style.BRIGHT + 'formatting external flash…')
                self._log.info(Style.DIM + 'this might take up to 60 seconds.')
                # !!! only required on the very first start (will remove everything)
                self._flash.format()
                # create the filesystem, this takes some seconds (approx. 10 sec)
                self._log.info('Creating filesystem for external flash…')
                self._log.info(Style.DIM + 'this might take up to 10 seconds.')
                # !!! only required on first setup and after formatting
                os.VfsFat.mkfs(self._flash)
                self._log.info('filesystem for external flash created.')
            except Exception as e:
                self._log.error('{} raised by format_flash: {}'.format(type(e), e))
        else:
            self._log.warning('external flash formatting is not enabled.')

    def _rescue_mount(self):
        try:
            self._log.info('attempting to mount external flash…')
            # finally mount the external flash
            os.mount(self._flash, self._flash_mount_point)
            self._log.info(Fore.GREEN + 'external flash Mounted.')
        except Exception as e:
            self._log.error('{} raised by mount: {}'.format(type(e), e))

    def mount(self):
        try:
            self._log.info('mounting external flash…')
            os.mount(self._flash, self._flash_mount_point)
            self._log.info(Fore.GREEN + 'external flash Mounted.')
        except Exception as e:
            if e.errno == 19:
                # [Errno 19] ENODEV aka "No such device"
                # create the filesystem, this takes some seconds (approx. 10 sec)
                self._mkfs()
            else:
                self._format_flash()
            self._rescue_mount()
        self._log.info("directories: {}".format(os.listdir('/')))

def main():
    fm = FlashMount(write_enable=True)
    fm.mount()

if __name__ == "__main__":
    main()

#EOF
