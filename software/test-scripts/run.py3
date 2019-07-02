#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("\r\nFinding an ODrive...")
odrv0 = odrive.find_any()

print("\r\nODrive Found")
print("Bus voltage is:" + str(odrv0.vbus_voltage) + "V")

print("Requesting state 5\r\n")
odrv0.axis1.requested_state = 5

time.sleep(5)

print("A2\r\n")
odrv0.axis1.controller.vel_setpoint = 1382.30076758 /2.0#368.61353802
time.sleep(5)

print("A3\r\n")
odrv0.axis1.controller.vel_setpoint = 2764.60153516 /2.0
time.sleep(5)

print("A4\r\n")
odrv0.axis1.controller.vel_setpoint = 5529.20307032 /2.0
time.sleep(5)


print("0RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 0
time.sleep(1)


print("Requesting state 1\r\n")
odrv0.axis1.requested_state = 1
