# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

# Simple demo of the VL53L1X distance sensor.
# Will print the sensed range/distance every second.

import time
from machine import I2C
from pyb import Pin
import adafruit_vl53l1x

#i2c2_scl = Pin(Pin.cpu.B13)
#i2c2_scl.init(mode=Pin.ALT, alt=Pin.AF4_I2C2)
#i2c2_sda = Pin(Pin.cpu.B14)
#i2c2_sda.init(mode=Pin.ALT, alt=Pin.AF4_I2C2)

i2c = I2C(1, freq=400_000)

vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# OPTIONAL: can set non-default values
vl53.distance_mode = 1
vl53.timing_budget = 50

print("VL53L1X Simple Test.")
print("--------------------")
model_id, module_type, mask_rev = vl53.model_info
print("Model ID: 0x{:0X}".format(model_id))
print("Module Type: 0x{:0X}".format(module_type))
print("Mask Revision: 0x{:0X}".format(mask_rev))
print("Distance Mode: ", end="")
if vl53.distance_mode == 1:
    print("SHORT")
elif vl53.distance_mode == 2:
    print("LONG")
else:
    print("UNKNOWN")
print("Timing Budget: {}".format(vl53.timing_budget))
print("--------------------")

vl53.start_ranging()

while True:
    if vl53.data_ready:
        print("Distance: {} mm".format(vl53.distance))
        vl53.clear_interrupt()
        time.sleep(0.2)
