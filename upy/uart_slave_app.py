#!/micropython

import uasyncio as asyncio
from colorama import Fore, Style

from core.logger import Logger, Level
from payload import Payload
from pixel import Pixel
from payload_router import PayloadRouter
from fake_motor import FakeMotor
from colors import *

class UartSlaveApp:
    def __init__(self, is_pyboard=True):
        self._is_pyboard = is_pyboard
        self._log = Logger('main', Level.INFO)
        self._pixel    = Pixel(brightness=0.1)
        self._slave    = None
        self._uart_id  = None
        self._baudrate = 1_000_000  # Can be configured if needed
        self._motor_controller = FakeMotor()
        self._router   = PayloadRouter(self._motor_controller)
        self._log.info('ready.')

    def pixel_on(self):
        self._pixel.set_pixel(color=COLOR_CYAN)

    def pixel_off(self):
        self._pixel.set_pixel(color=None)

    async def _pyb_wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        for _ in range(3):
            self.pixel_on()
            _led.on()
            await asyncio.sleep_ms(50)
            self.pixel_off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.pixel_off()
        _led.off()

    async def _wait_a_bit(self):
        from machine import Pin
        _led = Pin(11, Pin.OUT)
        for _ in range(3):
            self.pixel_on()
            _led.on()
            await asyncio.sleep_ms(50)
            self.pixel_off()
            _led.off()
            await asyncio.sleep_ms(950)
        self.pixel_off()
        _led.off()

    async def setup_uart_slave(self):
        if self._is_pyboard:
            self._log.info(Fore.GREEN + "configuring UART slave for STM32 Pyboard…")
            from stm32_uart_slave import Stm32UartSlave
            await self._pyb_wait_a_bit()
            self._uart_id = 4
            self._slave = Stm32UartSlave(uart_id=self._uart_id, baudrate=self._baudrate)
        else:
            self._log.info(Fore.GREEN + "configuring UART slave for RP2040…")
            from rp2040_uart_slave import RP2040UartSlave
            await self._wait_a_bit()
            self._uart_id = 1
            self._slave = RP2040UartSlave(uart_id=self._uart_id, baudrate=self._baudrate)
        
        self._slave.set_verbose(True)
        self._log.info("UART slave: waiting for command from master…")

    async def run(self):
        await self.setup_uart_slave()
        while True:
            packet = await self._slave.receive_packet()
            if packet is not None:
                self._log.info(Fore.WHITE + Style.BRIGHT + "packet: {} (type: {}).".format(packet, type(packet)))
                ack_payload = Payload("AK", 0.0, 0.0, 0.0, 0.0)
                await self._slave.send_packet(ack_payload)
            else:
                self._log.warning("no valid packet received.")

    def close(self):
        self.pixel_off()

# for REPL usage or testing
def exec():
    app = UartSlaveApp(is_pyboard=True)
    asyncio.run(app.run())

if __name__ == "__main__":
    app = UartSlaveApp(is_pyboard=True)
    asyncio.run(app.run())

#EOF
