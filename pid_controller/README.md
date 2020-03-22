# bowbi
Software for B.O.W.B.I personal robotics project. (balancing on wheels by itself)

# pid_controller
This directory contains the firmware for the Polou A-Star 32u4 Micro Arduino compatible microcontroller that controls the speed of bowbi's two encoder gear motors.

The microcontroller is dedicated as a PID loop controller.
Uses interrupt pins to read the two quadrature encoders.
Interacts with L298N dual h-bridge for motor control.

