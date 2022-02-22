from ddboat_tools import *


import gps_driver_v2 as gpsdrv


import time

gps = gpsdrv.GpsIO()


while 1 :


    v = gps.read_next_message()

    if len(v) > 0 :

        sentence_id = gps.get_sentence(v)

        if sentence_id == "$GPGLL" :
            gps_data = gps.get_gps_data(v)
            lat,lon = cvt_gll_ddmm_2_dd(gps_data)
            print(gps_data)
            f = open("gps","w")
            f.write("{} {}".format(lat,lon))
            f.close()

        elif sentence_id == "$GPVTG" :
            speed_data = gps.get_speed_data(v)
            print(speed_data)
            f = open("speed","w")
            f.write("{}".format(speed_data))
            f.close()

    time.sleep(0.1)



gps.close()
