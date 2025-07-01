
======
Pinout
======

Here are the pin configurations for connecting a Raspberry Pi to a STM32H562.

UART Pins
---------

This uses a custom build of MicroPython for the STM32H562, whose board definition
can be found at `mp-weact-stm32h562 <https://github.com/ifurusato/mp-weact-stm32h562/>`__.
The pins used for this implementation are defined in the board definition.

A custom build of Micro

The UART slave for the STM32 is able to use UART 1-3. 

UART 1 is discouraged as it is used to print console communications over ``/dev/serial0``
when connected to the board (e.g., over rshell). UART 4's pins conflict with the SD card
so support for it as not built into MicroPython for the STM32H562.

Therefore, only UART 2 or 3 are suitable: UART 2 is configured as the default.

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
    it will use the default pins for each.


Motor Pins
----------

{{PINOUT}}

All four motors use the same PWM timer ({{PWM_TIMER}}) and encoder timer ({{ENC_TIMER}}).

