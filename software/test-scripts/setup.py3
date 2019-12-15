#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("\r\nFinding an ODrive...")
odrv0 = odrive.find_any()

# Erase Current Configuration
odrv0.erase_configuration()

print("\r\nODrive Found")
print("Bus voltage is:" + str(odrv0.vbus_voltage) + "V")

#print("Calibration current:" + str(odrv0.axis1.motor.config.calibration_current) + "A")

odrv0.config.brake_resistance = 0.5

odrv0.axis0.config.startup_motor_calibration = True
odrv0.axis0.motor.config.requested_current_range = 90
odrv0.axis0.motor.config.direction = 1
odrv0.axis0.motor.config.pre_calibrated = 0
odrv0.axis0.controller.config.pos_gain = 20
odrv0.axis0.controller.config.vel_limit = 20000
odrv0.axis0.encoder.config.use_index = 0
odrv0.axis0.encoder.config.pre_calibrated = 0
odrv0.axis0.encoder.config.cpr = 8192

odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.motor.config.pole_pairs = 2
odrv0.axis0.motor.config.resistance_calib_max_voltage = 1.0
odrv0.axis0.motor.config.calibration_current = 10
odrv0.axis0.controller.config.vel_gain = 0.1
odrv0.axis0.controller.config.vel_integrator_gain = 0.05
odrv0.axis0.controller.config.control_mode = 2
odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 0.000467228

odrv0.axis1.config.startup_motor_calibration = True
odrv0.axis1.motor.config.requested_current_range = 90
odrv0.axis1.motor.config.direction = 1
odrv0.axis1.motor.config.pre_calibrated = 0
odrv0.axis1.controller.config.pos_gain = 20
odrv0.axis1.controller.config.vel_limit = 20000
odrv0.axis1.encoder.config.use_index = 0
odrv0.axis1.encoder.config.pre_calibrated = 0
odrv0.axis1.encoder.config.cpr = 8192

odrv0.axis1.motor.config.current_lim = 40
odrv0.axis1.motor.config.pole_pairs = 2
odrv0.axis1.motor.config.resistance_calib_max_voltage = 1.0
odrv0.axis0.motor.config.calibration_current = 10
odrv0.axis1.controller.config.vel_gain = 0.1
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

############################################

print("\r\nFinding an ODrive...\r\n")
odrv0 = odrive.find_any()

print("Found an ODrive\r\n")
print("Axis state: ")
print(odrv0.axis0.current_state)
print("\r\n")
print(odrv0.axis1.current_state)
print("\r\n")

print("Requesting state 4 - Axis0\r\n")
odrv0.axis0.requested_state = 4

print("Axis0 state: ")
print(odrv0.axis0.current_state)
print("\r\n")

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

print("phase resistance: ", odrv0.axis0.motor.config.phase_resistance)
print("phase inductance: ", odrv0.axis0.motor.config.phase_inductance)

print("Requesting state 4 - Axis1\r\n")
odrv0.axis1.requested_state = 4

print("Axis1 state: ")
print(odrv0.axis1.current_state)
print("\r\n")

while odrv0.axis1.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

print("phase resistance: ", odrv0.axis1.motor.config.phase_resistance)
print("phase inductance: ", odrv0.axis1.motor.config.phase_inductance)

time.sleep(10)

print("\r\nIs calibrated? ", odrv0.axis0.motor.is_calibrated)
print("\r\nIs calibrated? ", odrv0.axis1.motor.is_calibrated)
print("")

#odrv0.axis0.motor.config.pre_calibrated = True
#odrv0.axis1.motor.config.pre_calibrated = True

print("Axis state: ")
print(odrv0.axis1.current_state)
print(odrv0.axis0.current_state)
print("\r\n")

odrv0.save_configuration()
print("Rebooting\r\n")

try:
    odrv0.reboot()
except Exception:
    pass

print("\r\nFinding an ODrive...\r\n")
odrv0 = odrive.find_any()

print("Found an ODrive\r\n")

print("\r\nIs calibrated? ", odrv0.axis0.motor.is_calibrated)
print("\r\nIs calibrated? ", odrv0.axis1.motor.is_calibrated)
print("")

print("\r\naxis errors are:")
print(hex(odrv0.axis1.error))	
print("motor errors are:")
print(hex(odrv0.axis1.motor.error))
print("encoder errors are:")
print(hex(odrv0.axis1.encoder.error))
print("sensorless estimator errors are:")
print(hex(odrv0.axis1.sensorless_estimator.error))
