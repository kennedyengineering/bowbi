# bowbi
Software for B.O.W.B.I personal robotics project. (balancing on wheels by itself)

# main_controller
This directory contains the firmware for the Arduino Mega microcontroller that controls bowbi's hardware.

Polls the MPU6050 for orientation data.
Uses interrupt pins to read the two quadrature encoders.
Interacts with L298N dual h-bridge for motor control.

# Dependencies
The firmware relies on Teensy Quadrature encoder library
https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize

Also uses the Arduino PID Library - Version 1.2.1 by Brett Beauregard
https://github.com/br3ttb/Arduino-PID-Library/

# Notes
The Arduino Mega has interrupts on pins 2, 3, 18, 19, 20, 21
though 20, 21 are used for I2C communication

