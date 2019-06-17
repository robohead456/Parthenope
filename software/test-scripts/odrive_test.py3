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

odrv0.config.brake_resistance = 0.5
odrv0.axis1.motor.config.pole_pairs = 2
odrv0.axis1.motor.config.resistance_calib_max_voltage = 5.0
odrv0.axis1.sensorless_estimator.config.pm_flux_linkage = 0.000467228

print("\r\nset parameters\r\n\r\nsaving configuration and rebooting")

odrv0.save_configuration()

try:
    odrv0.reboot()
except Exception:
    pass

    print("\r\ndone rebooting\r\n")

odrv0.axis1.requested_state = 4

print("phase resistance: ", odrv0.axis1.motor.config.phase_resistance)

time.sleep(5)
print("\r\naxis errors are:")
print(hex(odrv0.axis1.error))	
print("motor errors are:")
print(hex(odrv0.axis1.motor.error))
print("encoder errors are:")
print(hex(odrv0.axis1.encoder.error))
print("sensorless estimator errors are:")
print(hex(odrv0.axis1.sensorless_estimator.error))

print("\r\nIs calibrated? ", odrv0.axis1.motor.is_calibrated)
print("")