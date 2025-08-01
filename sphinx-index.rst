
.. admonition:: Important

    This documentation was last updated {{date}}, so it may not be up to date with the repository.

    The project is still a work in progress, i.e., things are still changing frequently, not everything works, or works as expected.


Brushless Motor Controller
**************************

A Python Controller for the DFRobot Brushless DC Motor with Encoder 12V 159RPM (FIT0441)
----------------------------------------------------------------------------------------

*Welcome!*

In 2025 DFRobot released the `Brushless DC Motor with Encoder 12V 159RPM (FIT0441) <https://www.dfrobot.com/product-1364.html>`__,
a small (24mm diameter, 40mm length), relatively inexpensive 12V DC brushless motor
with its own internal controller, suitable for integration with the rest of the
Python-based robot OS (`KROS <https://github.com/ifurusato/krzos>`__).

The motor is controlled by three pins: a pulse-width-modulated (PWM) pin, a direction
pin, and an ``FG`` pin providing feedback from its internal motor encoder, permitting
closed-loop control. Note that the encoder pin does not provide direction information,
only an indication of movement.

Also note that the motor's PWM is *inverted*: 100% is stopped, 0% is full speed.

The GitHub repository for this project may be found at:
`brushless_motor_controller <https://github.com/ifurusato/brushless-motor-controller/tree/main>`__


References
----------

* Product page: `Brushless DC Motor with Encoder 12V 159RPM FIT0441 <https://www.dfrobot.com/product-1364.html>`__
* Documentation: `FIT0441 Brushless DC Motor with Encoder 12V 159RPM <https://wiki.dfrobot.com/FIT0441_Brushless_DC_Motor_with_Encoder_12V_159RPM>`__


========
Features
========

The project operates in one of five hardware modes:

1. using a **software PWM** from the Raspberry Pi to the motors: does provide a stable frequency
2. using a **hardware PWM** from the Raspberry Pi to the motors: there are only two
   hardware PWM pins available on the Raspberry Pi
3. using an **external PWM controller** such as the TI TLC59711 PWM controller (supplied by
   an Adafruit board [*]_, see below): requires an additional SPI device but is both
   stable and there are 12 PWM channels available
4. connecting via **UART** to either an RP2040 or STM32 microcontroller: the microcontroller
   reliably handles the PWM to the motors, but may be limited by the performance of the UART

.. note::
    While the fourth option describes a Raspberry Pi master connected to a microcontroller slave,
    strictly speaking the Raspberry Pi is not necessary if control of the MotorController were
    fleshed out with a higher-level robot controller on the microcontroller itself (though that
    is beyond the scope of this project). I.e., a robot based solely on a microcontroller could
    use this project's MotorController without a Raspberry Pi at all.


The motor controller itself includes support for open- or closed-loop control,
stall-and-recovery, deadband control, and control by target RPM when operating in
closed-loop mode. Provided a wheel diameter this also provides for odometric
distance and speed measurements.

This uses a YAML configuration file for the application itself, and another on the
microcontroller for the motor pin configuration, specifically when using the UART
in the MicroPython environent.

.. [*] The `Adafruit 12 Channel 16-bit PWM LED Driver - SPI Interface <https://www.adafruit.com/product/1455>`__
       is designed to control up to 12 channels of LEDs, or four RGB LEDs. In our case we only need a much smaller
       number of channels, one per motor. These channels are enumerated in the `ControllerChannel` class.


===================
Support & Liability
===================

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


===================
Copyright & License
===================

All contents (including software, documentation and images)
Copyright 2020-2025 by Ichiro Furusato. All rights reserved.

Software and documentation are distributed under the MIT License, see the LICENSE
file included with the project.

