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



# waypoint liste
# # triangle1 :
# WPs = [(48.19923,-3.01474),(48.19953,-3.01636),(48.1989 , -3.01626)]
#triangle2 :
WPs = [[48.19923, -3.01474],[48.1994, -3.01565],[48.19881, -3.01542]]
# secret :
# WPs = ((48.199223,-3.014926),(48.198990,-3.015532),(48.198969,-3.015758),(48.199040,-3.015935),(48.199133,-3.015940),(48.199237,-3.015758),(48.199337,-3.015946),(48.199451,-3.015940),(48.199534,-3.015752),(48.199484,-3.015516),(48.199208,-3.014889))
# logo ENSTA
# WPs = ((48.199174, -3.01484), (48.198834, -3.014996), (48.198623, -3.015077), (48.198488, -3.015136), (48.198094, -3.015243), (48.197962, -3.015159), (48.197716, -3.015099), (48.197455, -3.015119), (48.197195, -3.015289), (48.197008, -3.015549), (48.196888, -3.015889), (48.19685, -3.016343), (48.196868, -3.01683), (48.197015, -3.017231), (48.197202, -3.017501), (48.197469, -3.017691), (48.197796, -3.017701), (48.198036, -3.017591), (48.198148, -3.017507), (48.198277, -3.017324), (48.198352, -3.017174), (48.198416, -3.016997), (48.198484, -3.016691), (48.198295, -3.016729), (48.197308, -3.016729), (48.196868, -3.016686), (48.196879, -3.016793), (48.196997, -3.016836), (48.197079, -3.016927), (48.197133, -3.017094), (48.197147, -3.017195), (48.197179, -3.017233), (48.197211, -3.017222), (48.197243, -3.01712), (48.197222, -3.016852), (48.19719, -3.016568), (48.197143, -3.016391), (48.197022, -3.016225), (48.196871, -3.016117), (48.196853, -3.016348), (48.196861, -3.016557), (48.197319, -3.016562), (48.198313, -3.0166), (48.198488, -3.016659), (48.198509, -3.0163), (48.198502, -3.015956), (48.198398, -3.015661), (48.198238, -3.015366), (48.198102, -3.015248), (48.197991, -3.01586), (48.197898, -3.016455), (48.197826, -3.017507), (48.197737, -3.017008), (48.197583, -3.016605), (48.197419, -3.016294), (48.197186, -3.016106), (48.197472, -3.015978), (48.19778, -3.01579), (48.197937, -3.015516), (48.198037, -3.015221), (48.19823, -3.015372), (48.198398, -3.015672), (48.198498, -3.015962), (48.198513, -3.016294), (48.198491, -3.016686), (48.198413, -3.016992), (48.198345, -3.017179), (48.198273, -3.017335), (48.198159, -3.017507), (48.198141, -3.017056), (48.198112, -3.016643), (48.198073, -3.016316), (48.198016, -3.01609), (48.197916, -3.015817), (48.197801, -3.015608), (48.197676, -3.015511), (48.197522, -3.015527), (48.197433, -3.015693), (48.197354, -3.015908), (48.197311, -3.016321), (48.197333, -3.016638), (48.197376, -3.016981), (48.197465, -3.017179), (48.19764, -3.017394), (48.197655, -3.018032), (48.198481, -3.017855), (48.198724, -3.016235), (48.199192, -3.014824))
nb_wp = len(WPs) # nombre de waypoint


# filename for log
filename = "log/DDBoat-triangle2-"+time.strftime("%d-%H:%M:%S")+".log"

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
        while not waypoint_passed(next_wp,WPs[i],(lat,lon)) or coords_dist((lat,lon),next_wp)<5 : # tant qu'on a pas passé le next_waypoint ou qu'on est à moins de 5m
            try : # on essaye de recupere une nouvelle valeur de gps et on met a jour l'url
                lat,lon = get_gps()
                url = "https://enstapocalypse.casa/campagne/SOS/add_pos_ios.php?lat="+str(lat)+"&lon="+str(lon)+"&id="+id
            except :
                pass

            raw_imu = imu.read_mag_raw() # lecture des données du magnétometre de l'imu
            cap = get_compass_from_raw(raw_imu) # calcul du cap du bateau
            cap_cons = follow_line_coord(WPs[i],next_wp,(lat,lon),6) - np.pi # calcul du cap a suivre



            dt = time.time() - t_motor
            if dt > 0.05:
                w1_cons, w2_cons = cmdcap(cap_cons,cap)
                old_odo1,old_odo2,u1,u2 = cmd_moteur(encod,old_odo1,old_odo2,dt,u1,u2,w1_cons,w2_cons)
                t_motor = time.time()
                ard.send_arduino_cmd_motor(u1,u2)

            print("\nt={:.3f}\n heading to {} wp\n {} {}\n cap consigne : {:.0f} cap réel : {:.0f}\n u1 = {} u2 = {}\n w1= {} w2 = {}\n\n\n----------------".format(time.time()-t0,i+1,lat,lon,cap_cons*180/np.pi,cap*180/np.pi,u1,u2,w1_cons,w2_cons))
            log.write("{};{};{};{};{};{}\n".format(time.time()-t0,i+1,lat,lon,cap_cons,cap))







        time.sleep(0.5)

    ard.send_arduino_cmd_motor(0,0) # stop the motors
