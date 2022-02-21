import time
import sys

from imu9_driver_v2 import Imu9IO
import gps_driver_v2 as gpsdrv
import gpxpy.gpx
import arduino_driver_v2 as arddrv
import encoders_driver_v2 as encoddrv
from scipy import signal
import numpy as np
from gps_receiver import *
from ddboat_tools import *




ard = arddrv.ArduinoIO()
encod = encoddrv.EncoderIO()
encod.get_sync()
#gps = gpsdrv.GpsIO()
imu = Imu9IO()

sync,data_encoders = encod.read_packet(debug=False)
old_odo1,old_odo2 = data_encoders[3], data_encoders[4]
#top1,top2 = data_encoders[3], data_encoders[4]
#odo1,odo2 = [top1,top1,top1], [top2,top2,top2]


filename = "log/DDBoat_essai-"+time.strftime("%d-%H:%M:%S")+".log"
with open(filename,'w') :

    input()
    t_motor = time.time()
    t0 = time.time()
    u1,u2 = 0,0


    for _ in range(1000000):
        raw_imu = imu.read_mag_raw()
        #gps_data = gps.read_gll()

        cap = get_compass_from_raw(raw_imu)

        gps = open('log.txt', 'r')
        gps_data = gps.readlines()
        gps.close()

        print(gps_data)

        if time.time()-t0 <= 20:
            cap_cons = -np.pi/2
        elif 20 < time.time()-t0 <= 40:
            cap_cons = 0
        elif 40 < time.time()-t0 <= 60:
            cap_cons = np.pi/2
        elif 60 < time.time()-t0 <= 80:
            cap_cons = np.pi
        elif time.time()-t0 > 80:
            break
        #print("Cons={}, Cap={}".format(round(cap_cons,4),round(cap,4)))

        dt = time.time() - t_motor
        if dt > 0.05:
            w1_cons, w2_cons = cmdcap(cap_cons,cap)
            # print (w1_cons, w2_cons)
            old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
            t_motor = time.time()
            #print("Consignes: u1={}, u2={}".format(round(u1,4),round(u2,4)))
            ard.send_arduino_cmd_motor(u1,u2)

        # if 10 < time.time() - t0 <= 12.5:
        #     print("nord",cap)
        #     cap_cons = 0
        #     w1_cons, w2_cons = cmdcap(cap_cons,cap)
        #     odo1,odo2,T,u1,u2 = cmd_moteur(encod,odo1,odo2,T,u1,u2,w1_cons,w2_cons)
        #     print(u1,u2)
        #     ard.send_arduino_cmd_motor(u1,u2)
        #
        # if 12.5 < time.time() - t0 <= 22.5:
        #     print("est",cap)
        #     cap_cons = np.pi
        #     w1_cons, w2_cons = cmdcap(cap_cons,cap)
        #     odo1,odo2,T,u1,u2 = cmd_moteur(encod,odo1,odo2,T,u1,u2,w1_cons,w2_cons)
        #     print(u1,u2)
        #     ard.send_arduino_cmd_motor(u1,u2)

        #print("{};{};{};{}".format(time.time()-t0,lat,lon,cap))
    ard.send_arduino_cmd_motor(0,0)

    #gps.close()
