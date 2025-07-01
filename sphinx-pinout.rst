
======
Pinout
======

Here are the pin configurations for connecting a Raspberry Pi to a STM32H562.

UART Pins
---------

This uses a custom build of MicroPython for the STM32H562, whose board definition
can be found at `mp-weact-stm32h562 <https://github.com/ifurusato/mp-weact-stm32h562/>`__.
The pins used for this implementation are defined in the board definition.

The STM32H562 build defines UARTs 1-3. UART 1 is discouraged as it is used to
print console communications over ``/dev/serial0`` when connected to the board
(e.g., over rshell). UART 4's pins conflict with the SD card so support for it
as not built into MicroPython for the STM32H562. Therefore, only UART 2 or
UART 3 are suitable: UART 2 is configured as the default.

+--------+-------+-------+
| UART   |  TX   |  RX   |
+========+=======+=======+
| UART1  |  PA9  | PA10  |
+--------+-------+-------+
| UART2  |  PA2  | PA3   |
+--------+-------+-------+
| UART3  | PB10  | PB11  |
+--------+-------+-------+

.. note::

    You cannot specify tx/rx pins in MicroPython's ``UART()`` constructor;
    it will use the defined pins for each.

The primary UART on the Raspberry Pi is GPIO 14 (TX) and GPIO 15 (RX). Using
UART 2, GPIO 14 is therefore connected to PA3, GPIO 15 to PA2.

If using a NeoPixel or NeoPixel strip, it is connected to PA1. This is currently
hard-coded in the ``upy/pixel.py`` class.


Motor Pins
----------

Apart from power, there are three connections to each motor: a PWM pin used for
speed control; a direction pin that sets the motor direction; and an encoder
feedback pin whose ticks indicate motor rotation.

Hardware Timers on the STM32 have four channels. We use all four channels of
two Timers to supply our PWM and encoder signals.

{{PINOUT}}

All four motors use the same PWM Timer ({{PWM_TIMER}}) and encoder Timer
({{ENC_TIMER}}).

