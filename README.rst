*********************************************************************************
A controller for the DFRobot Brushless DC Motor with Encoder 12V 159RPM (FIT0441)
*********************************************************************************

Background
**********

.. figure:: img/FIT0441_Main_01.jpg
   :width: 1000px
   :align: center
   :alt: The DFRobot Brushless DC Motor with Encoder 12V 159RPM

   The DFRobot Brushless DC Motor with Encoder 12V 159RPM

DFRobot recently released a small (24mm diameter, 40mm length) 12V DC brushless
motor with its own internal controller, and I was looking for a controller to
integrate it with the rest of my Python-based robot OS.

The motor is controlled by three pins: a PWM pin, a direction pin, and an FG
pin providing feedback from its internal encoder, permitting closed-loop
control.

Note that its PWM is inverted: 100% is stopped, 0% is full speed.


Features
********

This motor controller includes support for open- or closed-loop control,
stall-and-recovery, deadband control, and control by target RPM when operating
in closed-loop mode. Given a wheel diameter this also provides for odometric
distance and speed measurements.

There are three PWM controller implementations: software PWM, hardware PWM,
and using a TI TLC59711 PWM controller (supplied by an Adafruit board, see
below).

The TLC69711 is usually designed to control up to 12 channels of LEDs, or
four RGB LEDs. In our case we only need a much smaller number of channels,
on per motor. These channels are enumerated in the ControllerChannel Enum.

This uses a YAML configuration file.


Software Requirements
*********************

A requirements.txt file is provided.

colorama==0.4.6
pigpio==1.78
PyYAML==6.0.2
spidev==3.5


The implementation uses pigpio, which requires running a daemon. It
probably wouldn't be horribly difficult to replace this with a different
Raspberry Pi GPIO support library, but pigpio provides a reliable and
high-performance API over the hardware PWM pins.


Hardware Requirements
*********************

The motor is available from vendors selling DFRobot products.

The product page for the is:
`DFRobot Brushless DC Motor with Encoder 12V 159RPM (FIT0441) <https://www.dfrobot.com/product-1364.html>`__
with the support wiki page at:
`FIT0441 Brushless DC Motor with Encoder 12V 159RPM <https://wiki.dfrobot.com/FIT0441_Brushless_DC_Motor_with_Encoder_12V_159RPM>`__

If you choose to use the TCL59711, one option is the:
`Adafruit 12-Channel 16-bit PWM LED Driver - SPI Interface - TLC59711 <https://www.adafruit.com/product/1455>`__

with support documentation at:
`TLC5947 and TLC59711 PWM LED Driver Breakouts <https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout>`__


Hardware Configuration
**********************

Depending on your choise of software PWM, hardware PWM, or a TLC59711 the wiring
will be different. The Raspberry Pi has two channels available for hardware PWM,
with two pins (on standard, one alternate) for each. Hardware PWM will provide a
much more stable, reliable signal than the software PWM, which can wobble due to
system load.

If your project requires two motors then just use the hardware PWM pins, but if
your project requires four motors, you can either use the software PWM pins (less
than ideal) or the external PWM controller board.

The direction and FG (encoder feedback) pins can be configured to use any
available GPIO pins.

+-----------------+-----------------+---------------+
| Pin             |  GPIO pin/alt   | Notes         |
+=================+=================+===============+
| PWM Channel 0   |  GPIO 18 / 12   | hardware PWM  |
| PWM Channel 1   |  GPIO 19 / 13   | hardware PWM  |
| Direction       |  GPIO 23        | any GPIO pin  |
| FG (encoder)    |  GPIO 24        | any GPIO pin  |
+-----------------+-----------------+---------------+


Status
******

This is a first release and the motor has only been tested on the bench, with
the motor spinning free, so it's expected that PID tuning, deadband configuration,
etc. will be necessary once the motor has been installed and is running under load.

.. note::

   The project is being exposed publicly so that those interested can follow its progress.
   It is not remotely considered production quality and there are very likely bugs that
   have not yet been uncovered, and a few that are known but have not been fixed†.

† e.g., you can fool the controller by rapidly switching from high speed positive to
   high speed negative, as the FG encoder feedback pin is not directional. So far,
   some mitigations have been attempted but if the change happens fast enough the
   motor will just continue rotating in its current direction. Either tuning the
   ramp function or the slew limiter may help, but a higher-level control should not
   permit such fast changes anyway, so this is a low priority bug.


Support & Liability
*******************

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


Copyright & License
*******************

All contents (including software, documentation and images)
Copyright 2020-2025 by Murray Altheim. All rights reserved.

Software and documentation are distributed under the MIT License, see LICENSE
file included with project.

