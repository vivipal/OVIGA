import time

import arduino_driver_v2 as arddrv
import encoders_driver_v2 as encoddrv
from imu9_driver_v2 import Imu9IO
import gpxpy.gpx

from ddboat_tools import *

from roblib import *

def get_gps():
    f=open("gps",'r')
    gps_data = f.readlines()
    f.close()

    lat,lon = gps_data[0].split(' ')

    return float(lat),float(lon)



imu = Imu9IO()

# ax = init_figure(-50, 50, -50, 50)
# clear(ax)


ard = arddrv.ArduinoIO()
encod = encoddrv.EncoderIO()
encod.get_sync()

sync,data_encoders = encod.read_packet(debug=False)
old_odo1,old_odo2 = data_encoders[3], data_encoders[4]

t_motor = time.time()
u1,u2 = 0,0
w1_cons,w2_cons=125,125
X,Y = [],[]

WPs = [(48.19923,-3.01474),(48.19953,-3.01636),(48.1989 , -3.01626)]
nb_wp = len(WPs)
t0 = time.time()

filename = "log/DDBoat_essai-"+time.strftime("%d-%H:%M:%S")+".log"

with open(filename,'w') as log:

    lat,lon = get_gps()

    for i in range(nb_wp): # on parcourt tt les waypoints
        next_wp = WPs[(i+1)%nb_wp]
        while not waypoint_passed(next_wp,WPs[i],(lat,lon)) : # tant qu'on a pas passé le next_waypoint
            lat,lon = get_gps()
            raw_imu = imu.read_mag_raw()
            cap = get_compass_from_raw(raw_imu)
            cap_cons = follow_line_coord(WPs[i],next_wp,(lat,lon),6) - np.pi




            dt = time.time() - t_motor
            if dt > 0.05:
                w1_cons, w2_cons = cmdcap(cap_cons,cap)
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
                t_motor = time.time()
                #print("Consignes: u1={}, u2={}".format(round(u1,4),round(u2,4)))
                ard.send_arduino_cmd_motor(u1,u2)

            print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons))
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))







        time.sleep(0.5)
