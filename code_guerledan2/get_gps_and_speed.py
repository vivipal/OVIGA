from ddboat_tools import *


import gps_driver_v2 as gpsdrv


import time

gps = gpsdrv.GpsIO()


while 1 :


    gps_data = gps.read_gll()

    # f = open("gps","w")
    # f.write("{} {}".format(lat,lon))
    # f.close()

    gll_ok,gll_data=gps.read_gll_non_blocking()
    if gll_ok:
        lat,lon = cvt_gll_ddmm_2_dd(gps_data)
        print("coord = " + str(lat) + str(lon)))
    time.sleep(0.01)

    vtg_ok,vtg_data=gps.read_vtg_non_blocking()
    if vtg_ok:
        print("speed = " + str(vtg_data)))
    time.sleep(0.1)

gps.close()
