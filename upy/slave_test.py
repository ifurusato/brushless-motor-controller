#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-23

from payload import Payload
from logger import Logger, Level

from uart_slave import UARTSlave

def main():
    _log = Logger('main', Level.INFO)
    _uart_id = 1
    _baudrate = 460800 # 115200 921600 1_000_000 
    slave = UARTSlave(uart_id=_uart_id, baudrate=_baudrate)
    _log.info("UART slave: waiting for command from master…")
    while True:
        packet = slave.receive_packet()
        if packet is not None:
#           _log.info("received payload: {}".format(packet))
            # respond with ACK + same payload command but zeroed floats (example)
            ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
            slave.send_packet(ack_payload)
        else:
            _log.warning("no valid packet received.")

if __name__ == "__main__":
    main()  # Call the main function to start the loop

#EOF
