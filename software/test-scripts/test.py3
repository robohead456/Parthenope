#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("\r\nFinding an ODrive...")
odrv0 = odrive.find_any()

# Erase Current Configuration
# my_drive.erase_configuration()

print("\r\nODrive Found")
print("Bus voltage is:" + str(odrv0.vbus_voltage) + "V")

#print("Calibration current:" + str(odrv0.axis1.motor.config.calibration_current) + "A")

odrv0.axis1.motor.config.current_lim = 10
odrv0.config.brake_resistance = 0.5
odrv0.axis1.motor.config.pole_pairs = 2
odrv0.axis1.motor.config.resistance_calib_max_voltage = 1.0
odrv0.axis1.controller.config.vel_gain = 0.01
odrv0.axis1.controller.config.vel_integrator_gain = 0.05
odrv0.axis1.controller.config.control_mode = 2
odrv0.axis1.sensorless_estimator.config.pm_flux_linkage = 0.000467228

print("\r\nset parameters\r\n\r\nsaving config and rebooting\r\n")

odrv0.save_configuration()

try:
    odrv0.reboot()
except Exception:
    pass

print("done rebooting\r\n")

print("\r\nFinding an ODrive...\r\n")
odrv0 = odrive.find_any()

print("Found an ODrive\r\n")
print("Axis state: ")
print(odrv0.axis1.current_state)
print("\r\n")

print("Requesting state 4\r\n")
odrv0.axis1.requested_state = 4

print("Axis state: ")
print(odrv0.axis1.current_state)
print("\r\n")

while odrv0.axis1.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

print("phase resistance: ", odrv0.axis1.motor.config.phase_resistance)
print("phase inductance: ", odrv0.axis1.motor.config.phase_inductance)

print("\r\nIs calibrated? ", odrv0.axis1.motor.is_calibrated)
print("")

print("Axis state: ")
print(odrv0.axis1.current_state)
print("\r\n")

print("Requesting state 5\r\n")
odrv0.axis1.requested_state = 5

print("1000RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 1000 * 2*3.14159/60 * 2
time.sleep(10)

print("2000RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 2000 * 2*3.14159/60 * 2
time.sleep(10)

print("1000RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 1000 * 2*3.14159/60 * 2
time.sleep(10)

print("10000RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 10000 * 2*3.14159/60 * 2
time.sleep(10)

print("0RPM\r\n")
odrv0.axis1.controller.vel_setpoint = 0
time.sleep(1)


print("Requesting state 1\r\n")
odrv0.axis1.requested_state = 1
#odrv0.axis1.controller.vel_setpoint = 0

#print("\r\naxis errors are:")
#print(hex(odrv0.axis1.error))	
#print("motor errors are:")
#print(hex(odrv0.axis1.motor.error))
#print("encoder errors are:")
#print(hex(odrv0.axis1.encoder.error))
#print("sensorless estimator errors are:")
#rint(hex(odrv0.axis1.sensorless_estimator.error))