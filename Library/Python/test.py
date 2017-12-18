#!/usr/bin/python
from BNO055 import BNO055
import time

sensor = BNO055(0x29)

while True:
    sensor.readEul()
    print("X: {:.1f}, Y: {:.1f}, Z: {:.1f}".format(sensor.euler['x'], sensor.euler['y'], sensor.euler['z']))
    time.sleep(0.010)
