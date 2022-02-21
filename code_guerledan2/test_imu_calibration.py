import time
import sys

from imu9_driver_v2 import Imu9IO
import numpy as np

from ddboat_tools import *


imu = Imu9IO()

while (1) :
	raw_imu = imu.read_mag_raw()

	cap = get_compass_from_raw(raw_imu)

	print(cap)
