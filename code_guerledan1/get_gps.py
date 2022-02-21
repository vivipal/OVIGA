from ddboat_tools import *


import gps_driver_v2 as gpsdrv


import time

gps = gpsdrv.GpsIO()


while 1 :


    gps_data = gps.read_gll()
    lat,lon = cvt_gll_ddmm_2_dd(gps_data)

    f = open("gps","w")
    f.write("{} {}".format(lat,lon))
    f.close()

    print(lat,lon)

    time.sleep(0.1)

gps.close()
