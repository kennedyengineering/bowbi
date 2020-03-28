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

Uses the I2cdevlib and MPU6050 class by jrowberg
https://github.com/jrowberg/i2cdevlib

# Notes
The Arduino Mega has interrupts on pins 2, 3, 18, 19, 20, 21
though 20, 21 are used for I2C communication

Motor 1:
	encoder:
		green wire --> 	pin 2
		yellow wire --> pin 3
	h-bridge:
		enable --> 	pin 4
		dir_1 -->	pin 50
		dir_2 -->	pin 51


Motor 2:
	encoder:
		green wire --> 	pin 18
		yellow wire --> pin 19
	h-bridge:
		enable --> 	pin 5
		dir_1 -->	pin 52
		dir_2 -->	pin 53

MPU6050:
	SDA/SCL -- pins 20, 21
