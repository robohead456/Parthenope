#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("Finding an odrive")
my_drive = odrive.find_any()

print("Setting hardware parameters")

# Axis 0 Hardware Parameters - 785W motor
my_drive.axis0.motor.config.current_lim = 50	 	# [A]			-> limit is 55 from datasheet
my_drive.axis0.controller.config.vel_limit = 120000 # [counts/sec]	-> 600 counts/rev * 200 rev/sec = 120000 counts/sec
my_drive.axis0.motor.config.calibration_current = 2 # [A] 			-> just a guess, no idea if a good number or not
my_drive.axis0.motor.config.pole_pairs = 7			# [#] 			-> 14 pairs / 2
my_drive.axis0.encoder.config.cpr = 600
my_drive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT # Not a gimbal motor

# Global Hardware Parameters
my_drive.config.brake_resistance = 0.5 # [Ohms]

# Calibrate
print("Calibrating")
my_drive.axis0.encoder.config.use_index = True
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while my_drive.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

print("Done Calibrating\n Motor phase resistance: " + str(my_drive.axis1.motor.config.phase_resistance) + " [ohms]")

# Set to velocity control
print("Changing state to closed loop control")
my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

# Print any errors
while my_drive.axis1.current_state != CTRL_MODE_VELOCITY_CONTROL:
    print("axis errors are:")
    print(hex(my_drive.axis1.error))	
    print("motor errors are:")
    print(hex(my_drive.axis1.motor.error))
    print("encoder errors are:")
    print(hex(my_drive.axis1.encoder.error))
    quit()
    time.sleep(0.1)

# 1 rps
my_drive.axis0.controller.vel_setpoint = 1 * 600
time.sleep(3)

# 2.5 rps
my_drive.axis0.controller.vel_setpoint = 2.5 * 600
time.sleep(3)

# 10 rps
my_drive.axis0.controller.vel_setpoint = 10 * 600
time.sleep(3)

# 40 rps
my_drive.axis0.controller.vel_setpoint = 40 * 600
time.sleep(3)

# 160 rps
my_drive.axis0.controller.vel_setpoint = 160 * 600
time.sleep(3)

# 0 rps
my_drive.axis0.controller.vel_setpoint = 0
time.sleep(3)

# 160 rps
my_drive.axis0.controller.vel_setpoint = 160 * 600
time.sleep(1)

# 0 rps
my_drive.axis0.controller.vel_setpoint = 0
time.sleep(1)
