
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

The repository may be found at:
`brushless_motor_controller <https://github.com/ifurusato/brushless-motor-controller/tree/main>`__


========
Features
========

The project operates in one of five hardware modes:

* using a software PWM from the Raspberry Pi to the motors: does provide a stable frequency
* using a hardware PWM from the Raspberry Pi to the motors: there are only two hardware PWM pins available on the Raspberry Pi
* using a TI TLC59711 PWM controller (supplied by an Adafruit board [†]_, see below): requires an additional SPI device but is both stable and there are 12 PWM channels available
* connecting via UART to either an RP2040 or STM32 microcontroller: the microcontroller handles the PWM to the motors, but is limited by the performance of the UART

.. note::
    One of the options is a microcontroller connected over a UART; with this approach
    the Raspberry Pi would not be necessary if control of the MotorController were
    fleshed out as necessary (though beyond the scope of this project). That is, e.g.,
    a robot based on a microcontroller could use the MotorController as a component
    without the Raspberry Pi as master.

The motor controller itself includes support for open- or closed-loop control,
stall-and-recovery, deadband control, and control by target RPM when operating in
closed-loop mode. Provided a wheel diameter this also provides for odometric
distance and speed measurements.

This uses a YAML configuration file for the application itself, and another for
the motor pin configuraion when using the UART in the MicroPython environent.

.. [†] The `Adafruit 12 Channel 16-bit PWM LED Driver - SPI Interface <https://www.adafruit.com/product/1455>`__
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

