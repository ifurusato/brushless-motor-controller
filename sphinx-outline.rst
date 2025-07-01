
========
Overview
========

This project uses Python and MicroPython for control of up to four DFRobot 12V DC
brushless motors (model number FIT0441). It is designed for use with a Raspberry Pi
but could be adapted to any hardware with similar capabilities.

The DFRobot brushless motors require a Pulse-Width Modified (PWM) signal to control
their speed, in an inverted mode: when fed a 100% duty cycle the motor is stopped,
a 50% duty cycle is half speed, a 0% duty cycle is full speed.

Because the Raspberry Pi is a user-space OS rather than an RTOS, its software PWM is
not particularly stable, as it is influenced by system load. The Pi has only two
channels of hardware PWM available. Therefore, if you are trying to control more than
two motors you'll need either an external PWM board or connect to a microcontroller
over a UART. In this latter mode the project supports either an RP2040 or an STM32
(i.e., the STM32H562, though it could with some pin configuration changes be used with
others).

References
----------

* Product page: `Brushless DC Motor with Encoder 12V 159RPM FIT0441 <https://www.dfrobot.com/product-1364.html>`__
* Documentation: `FIT0441 Brushless DC Motor with Encoder 12V 159RPM <https://wiki.dfrobot.com/FIT0441_Brushless_DC_Motor_with_Encoder_12V_159RPM>`__


Installation
------------

The Python files for the project are located in the root directory, ``core``, ``hardware``
and ``uart`` directories. The ``upy`` directory contains the contents to be uploaded to
the microcontroller.


