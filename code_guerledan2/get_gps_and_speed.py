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
            print(gps_data)

        else if sentence_id == "$GPVTG" :
            speed_data = gps.get_speed_data(v)
            print(speed_data)
            
    time.sleep(0.1)



gps.close()
