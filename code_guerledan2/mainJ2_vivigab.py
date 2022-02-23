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


## Calcul du point de départ à rejoindre
wpt_ponton = (48.1989495, -3.0148023)
bouee_lissajou = (48.19881166666667,-3.0156366666666665)
a1, a2 = coord2cart(bouee_lissajou).flatten()
R1, ind, N = 40, 9, 9    # rayon du cercle de départ, indice du bato, nombre de batos
wpt_cercle = cart2coord((a1 + R1*cos(alphai), a2 + R1*sin(alphai)))  # point à rejoindre sur le cercle


## Initialisation
imu = Imu9IO()
ard = arddrv.ArduinoIO()

encod = encoddrv.EncoderIO()
encod.get_sync()
sync,data_encoders = encod.read_packet(debug=False)
old_odo1,old_odo2 = data_encoders[3], data_encoders[4]

u1,u2 = 0,0
w1_cons,w2_cons=125,125

t0 = time.time()        # use to display time in log
dt_loop = 0.05
t_motor = time.time()   # use to control loop duration

filename = "log/jour_2_vivi-"+time.strftime("%d-%H:%M:%S")+".log"


## Main
with open(filename,'w') as log:

    lat,lon = get_gps()

    ## Rejoindre le point d'attente sur le cercle
    while not waypoint_passed(wpt_cercle,wpt_ponton,(lat,lon)) or coords_dist((lat,lon),wpt_cercle)<5 : # tant qu'on a pas passé le next_waypoint
        try :
            lat,lon = get_gps()
        except :
            pass
        raw_imu = imu.read_mag_raw()
        cap = get_compass_from_raw(raw_imu)
        cap_cons = follow_line_coord(wpt_ponton,wpt_cercle,(lat,lon),6) - np.pi
        dt = time.time() - t_motor
        if dt > dt_loop:
            w1_cons, w2_cons = cmdcap(cap_cons,cap)
            try:
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
            except :
                print("Erreur command moteur")
            t_motor = time.time()
            print(cap)
            ard.send_arduino_cmd_motor(u1,u2)
        log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
    ard.send_arduino_cmd_motor(0,0) # stop the motors
    print("Waiting point reached")


    ## Station keeping for a moment (for now just a pause)
    t_wait = time.time()
    while time.time()-t_wait < 15:
        dt = time.time() - t_motor
        if dt > dt_loop:
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
            t_motor = time.time()
    print("Waiting done")


    ## Faire le pattern Lissajou
    t_lissajou = time.time()
    while time.time()-t_lissajou < 120:
        try :
            lat,lon = get_gps()
        except :
            pass
        px, py=coord2cart((lat,lon)).flatten()
        x = array([[px],[py],[get_compass_from_raw(raw_imu)],[0.5]])
        u = lissajou(x)
        wmax_old=(w1_cons+w2_cons)/2

        dt = time.time() - t_motor
        if dt > dt_loop:
            w1_cons, w2_cons, w_max_old = cmdlissajou(u,w_max_old)
            try:
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
            except :
                print("Erreur encodeur")
            t_motor = time.time()
            ard.send_arduino_cmd_motor(u1,u2)

        # print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons))
        log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
    print("Lissajou done")


    ## Retour ponton
    wpt_start = (lat,lon)
    while not waypoint_passed(wpt_ponton,wpt_start,(lat,lon)) or coords_dist((lat,lon),wpt_ponton)<5 : # tant qu'on a pas passé le next_waypoint
        try :
            lat,lon = get_gps()
        except :
            pass
        raw_imu = imu.read_mag_raw()
        cap = get_compass_from_raw(raw_imu)
        cap_cons = follow_line_coord(wpt_start,wpt_ponton,(lat,lon),6) - np.pi
        dt = time.time() - t_motor
        if dt > dt_loop:
            w1_cons, w2_cons = cmdcap(cap_cons,cap)
            try:
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
            except :
                print("Erreur command moteur")
            t_motor = time.time()
            print(cap)
            ard.send_arduino_cmd_motor(u1,u2)
        log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))
    ard.send_arduino_cmd_motor(0,0) # stop the motors
    print("Home sweet home")
