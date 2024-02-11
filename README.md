# ScaraRobotControl_ArduinoDue_ARM_SAM3X8E

Program to control 4-axis SCARA robot wih an Arduino DUE board (SAM3X8E ARM Cortex-M3).

The robot uses stepper motors controlled by pulse-direction signals. The control architecture uses pre-computed 3rd order polynomial trajectories. In real-time, the motor steps are controlled by timer interrupts using a look-up table for faster execution.

This ScaraDue class contains function to create polynomial trajectories, run steppers in motor-only mode or linear stage mode, position and velocity control and homing.

It also includes functions to configure the motor pins, timers, limit switches or get feedback.
