import time
import requests as r
from threading import Thread


import arduino_driver_v2 as arddrv
import encoders_driver_v2 as encoddrv
from imu9_driver_v2 import Imu9IO


from ddboat_tools import *


def get_gps():
    '''
    ouvre le fichier 'gps' et retourne les valeurs de lat et lon
    '''
    f=open("gps",'r')
    gps_data = f.readlines()
    f.close()

    lat,lon = gps_data[0].split(' ')
    return float(lat),float(lon)

def send() :
    '''
    fonction utilisé pour envoyer les coordonnées du bateau sur le site internet
    cette fonction est appelé par un thread
    '''
    while 1 :
        response = r.get(url)
        time.sleep(1)


t = Thread(target = send) # thread pour l'envoi sur le site internet



imu = Imu9IO() # inertial measurement unit
ard = arddrv.ArduinoIO() # arduino
encod = encoddrv.EncoderIO() # motor encoder

encod.get_sync()
sync,data_encoders = encod.read_packet(debug=False)
old_odo1,old_odo2 = data_encoders[3], data_encoders[4]


id = '610' # id pour l'envoi de coord sur le site

t_motor = time.time()
u1,u2 = 0,0 # vitesse initial des moteurs
w1_cons,w2_cons=125,125 # consigne initial des moteurs

correc,p_time,p_dist= 0,0,0


# waypoint liste
#triangle :
WPs = [[48.19923, -3.01474],[48.1994, -3.01565],[48.19881, -3.01542]]
timel = (120,120,120)
# # secret :
# WPs = ((48.199223,-3.014926),(48.198990,-3.015532),(48.198969,-3.015758),(48.199040,-3.015935),(48.199133,-3.015940),(48.199237,-3.015758),(48.199337,-3.015946),(48.199451,-3.015940),(48.199534,-3.015752),(48.199484,-3.015516),(48.199208,-3.014889))
nb_wp = len(WPs) # nombre de waypoint


# filename for log
filename = "log/DDBoat-regul_temps_vivi-"+time.strftime("%d-%H:%M:%S")+".log"

lat,lon = get_gps() # get inital coord
url = "https://enstapocalypse.casa/campagne/SOS/add_pos_ios.php?lat="+str(lat)+"&lon="+str(lon)+"&id="+id # url pour envoyer les coords
t.start() # start the thread





t0 = time.time() # use to display time in log
with open(filename,'w') as log:

    for i in range(nb_wp) :
        log.write(str(WPs[i][0])+","+str(WPs[i][1]))
        if i <nb_wp-1 :
            log.write(';')
        else :
            log.write("\n")


    for i in range(nb_wp): # on parcourt tous les waypoints
        next_wp = WPs[(i+1)%nb_wp] # on recupere le prochain waypoint ie le i+1
        t_wp = time.time()

        I =0
        while not waypoint_passed(next_wp,WPs[i],(lat,lon)) : # tant qu'on a pas passé le next_waypoint ou qu'on est à moins de 5m
            try : # on essaye de recupere une nouvelle valeur de gps et on met a jour l'url
                lat,lon = get_gps()
                url = "https://enstapocalypse.casa/campagne/SOS/add_pos_ios.php?lat="+str(lat)+"&lon="+str(lon)+"&id="+id
            except :
                pass

            raw_imu = imu.read_mag_raw() # lecture des données du magnétometre de l'imu
            cap = get_compass_from_raw(raw_imu) # calcul du cap du bateau
            cap_cons = follow_line_coord(WPs[i],next_wp,(lat,lon),6) - np.pi # calcul du cap a suivre

            dist_next = coords_dist((lat,lon),next_wp)
            dist_prev = coords_dist((lat,lon),WPs[i])
            p_dist = dist_prev/(dist_next+dist_prev)

            t_ecoule = time.time()-t_wp
            p_time = t_ecoule/120



            dt = time.time() - t_motor
            if dt > 0.05:
                w1_cons, w2_cons = cmdcap(cap_cons,cap)
                try :
                    old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
                except :
                    pass
                    
                t_motor = time.time()

                epsilon = p_dist - p_time
                if abs(epsilon) < 0.01 :
                    I =0
                I+=epsilon


                correc = -100*epsilon + 1*I

                ard.send_arduino_cmd_motor(u1+correc,u2+correc)



            time.sleep(0.3)
            print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n correction : {} | distance : {:.1f}% | temps : {:.1f}%\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons,correc,p_dist*100,p_time*100))
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))







        time.sleep(0.5)

    ard.send_arduino_cmd_motor(0,0) # stop the motors
