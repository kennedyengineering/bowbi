# bowbi
Software for B.O.W.B.I personal robotics project. (balancing on wheels by itself)

# pid_controller
This directory contains the firmware for the Polou A-Star 32u4 Micro Arduino compatible microcontroller that controls the speed of bowbi's two encoder gear motors.
https://www.pololu.com/product/3101
The microcontroller is dedicated as a PID loop controller.
Uses interrupt pins to read the two quadrature encoders.
Interacts with L298N dual h-bridge for motor control.

# Dependencies
The firmware relies on Teensy Quadrature encoder library
https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize

Also uses the Arduino PID Library - Version 1.2.1 by Brett Beauregard
https://github.com/br3ttb/Arduino-PID-Library/

# Notes
In order to prevent Ubuntu computer from locking up the A-Star you must first stop ModemManager which interferes with the Arduino upload.
run: sudo systemctl stop ModemManager.service
If the Pololu board does lock up use their "bootloader before uploading method"

The Pololu A-Star 32u4 Micro uses the ATmega32U4 micro controller.
That means the interrupt pins are 0, 1, 2, 3 and are used to read the quadrature encoder.

