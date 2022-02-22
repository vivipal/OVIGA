#!/usr/bin/env python3
import time

import arduino_driver_v2 as arddrv
import encoders_driver_v2 as encoddrv
from imu9_driver_v2 import Imu9IO
import gpxpy.gpx
from datetime import datetime

from ddboat_tools import *

from roblib import *
from lissajou import *
def get_gps():
    f=open("gps",'r')
    gps_data = f.readlines()
    f.close()

    lat,lon = gps_data[0].split(' ')

    return float(lat),float(lon)

imu = Imu9IO()

ard = arddrv.ArduinoIO()
encod = encoddrv.EncoderIO()
encod.get_sync()

sync,data_encoders = encod.read_packet(debug=False)
old_odo1,old_odo2 = data_encoders[3], data_encoders[4]

t_motor = time.time()
u1,u2 = 0,0
w1_cons,w2_cons=125,125
X,Y = [],[]

point_depart = (48.1989495, -3.0148023)
a1, a2 = 48.19881166666667,-3.0156366666666665
R1 = 40
nbato = 9
alphai = nbato*2*pi/9
point_cercle = (a1 + R1*cos(alphai), a2 + R1*sin(alphai))
point_bouee = (48.19926166, -3.01513833)

WPs = [point_depart,point_bouee]

nb_wp = len(WPs)
t0 = time.time()

filename = "log/jour_2-"+time.strftime("%d-%H:%M:%S")+".log"

#Définition des heures de départ
start_min = 30
start_h = 10
start_d = 8
start_m = 10
start_y = 2021

temps_attente = int(10/60)

datetime_start = datetime(start_y, start_m, start_d,start_h,start_min,0)
datetime_wait_wp = (datetime(start_y, start_m, start_d,start_h,start_min+temps_attente,0),datetime(start_y,start_m,start_d,start_h,start_min,0))

while datetime.now() < datetime_start :
    print("Start in {} seconds".format((datetime_start-datetime.now()).seconds))
    time.sleep(1)

t0 = time.time() # use to display time in log

with open(filename,'w') as log:

    lat,lon = get_gps()

    for i in range(nb_wp): # on parcourt tt les waypoints
        print("going to the wp n°", i)
        next_wp = WPs[(i+1)%nb_wp]
        while not waypoint_passed(next_wp,WPs[i],(lat,lon)) or coords_dist((lat,lon),next_wp)<5 : # tant qu'on a pas passé le next_waypoint
            try :
                lat,lon = get_gps()
            except :
                pass
            raw_imu = imu.read_mag_raw()
            cap = get_compass_from_raw(raw_imu)
            cap_cons = follow_line_coord(WPs[i],next_wp,(lat,lon),6) - np.pi
            # print(cap, cap_cons)
            dt = time.time() - t_motor
            if dt > 0.05:
                w1_cons, w2_cons = cmdcap(cap_cons,cap)
                try:
                    old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
                except :
                    print("Erreur encodeur")
                t_motor = time.time()
                print(cap)
                ard.send_arduino_cmd_motor(u1,u2)

            # print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons))
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))

        time.sleep(0.5)

        ard.send_arduino_cmd_motor(0,0) # stop the motors
        while datetime.now()<datetime_wait_wp[i]:
            print("Resume in {} seconds".format((datetime_wait_wp[i]-datetime.now()).seconds))
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
            time.sleep(1)

            tw0 = time.time()
            #On commence à syncroniser si il reste 5 seconde
            if (datetime_wait_wp[i]-datetime.now()).seconds < 5 :
                try :
                    encod.get_sync()
                except:
                    pass

            #Sinon on retourne vers le point
            elif coords_dist(next_wp, get_gps()) > 8:
                dt = time.time() - t_motor
                if dt > 0.05:
                    w1_cons, w2_cons = cmdcap(cap_cons,cap)
                    try:
                        old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
                    except :
                        print("Erreur encodeur")
                    t_motor = time.time()
                    ard.send_arduino_cmd_motor(u1,u2)

    print("LISSAJOU")
    #En lissajou :
    while True:
        x = array([[get_gps()[0]],[get_gps()[1]],[get_compass_from_raw(raw_imu)],[0.5]])
        u = lissajou(x)
        wmax_old=(w1_cons+w2_cons)/2

        dt = time.time() - t_motor
        if dt > 0.05:
            w1_cons, w2_cons, w_max_old = cmdlissajou(u,w_max_old)
            try:
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
            except :
                print("Erreur encodeur")
            t_motor = time.time()
            ard.send_arduino_cmd_motor(u1,u2)

        # print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons))
        log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
