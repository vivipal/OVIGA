from ddboat_tools import *


import gps_driver_v2 as gpsdrv


import time

gps = gpsdrv.GpsIO()


while 1 :


    v = read_next_message(self)

    print(v)

    time.sleep(0.1)



gps.close()
