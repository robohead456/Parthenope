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

print("Axis state: ")
print(odrv0.axis1.current_state)
print("\r\n")

'''
Finding an ODrive...

ODrive Found
Bus voltage is:11.866625785827637V

set parameters

saving config and rebooting

done rebooting


Finding an ODrive...

Found an ODrive

Axis state: 
1


Requesting state 4

Axis state: 
4


phase resistance:  0.013660892844200134

axis errors are:
0x2
motor errors are:
0x0
encoder errors are:
0x0
sensorless estimator errors are:
0x0

Is calibrated?  False

Axis state: 
1
'''