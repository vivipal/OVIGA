import serial
import os
import time
import math

# distance calculation
def haversine(coord1, coord2):
    R = 6372800  # Earth radius in meters
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi       = math.radians(lat2 - lat1)
    dlambda    = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + \
        math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2

    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))

def ddmmtorad(x):
    x1,x2 = str(x).split(".")

    return(int(x1+"."+str(int(int(x2)*100/60))))



def cvt_gll_ddmm_2_dd (st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat/100))
    olon = float(int(ilon/100))
    olat_mm = (ilat%100)/60
    olon_mm = (ilon%100)/60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat,olon


def DMS2Rad(x):
    # 4811.1552


    M = x%100
    return ((D+M/60+S/3600)/180*3.141592654)

# the GPS sensor gives informations using the NMEA standard
# https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard
# https://en.wikipedia.org/wiki/NMEA_0183

class GpsIO:
    def __init__(self):
        # open serial line connected to the GPS sensor
        self.init_line()
        #time.sleep(1.0)
        #print(ser)

    def init_line(self,timeout=1.0):
        self.ser = serial.Serial('/dev/ttyS0',timeout=timeout)

    def close(self):
        self.ser.close()


    def read_next_message(self):
        v=self.ser.readline().decode("utf-8")
        #print (v)
        return v

    # read the position in the GPGLL message
    # by default one GPGLL message is expected every 20 messages
    def read_gll(self,n_try_max=20):
        val=[0.,'N',0.,'W',0.]
        for i in range(n_try_max):
            v=self.ser.readline().decode("utf-8")
            if str(v[0:6]) == "$GPGLL":
                vv = v.split(",")
                if len(vv[1]) > 0:
                    val[0] = float(vv[1])
                if len(vv[2]) > 0:
                    val[1] = vv[2]
                if len(vv[3]) > 0:
                    val[2] = float(vv[3])
                if len(vv[4]) > 0:
                    val[3] = vv[4]
                if len(vv[5]) > 0:
                    val[4] = float(vv[5])
                break # GPGLL found !  exit !
        return val

if __name__ == "__main__":
    gps = GpsIO()

    # display the 20 first messages
    #for i in range(20):
    #    print (gps.read_next_message())

    # gps coord place d'arme
    # lat_pa = 48.251306
    # lon_pa = -4.283464

    lat_pa = 48.418843
    lon_pa = -4.472451

    # lat_pa = ddmmtorad(lat_pa)
    # lon_pa = ddmmtorad(lon_pa)

    i = 0
    # display the 20 positions (GPGLL) messages
    while 1 :
        i+=1
        data = gps.read_gll()
        # lat = data[0]/100
        # lon = data[2]/100
        #
        # lat = ddmmtorad(lat)
        # lon = ddmmtorad(lon)

        lat,lon = cvt_gll_ddmm_2_dd(data)



        # if data[1] == 'S':
        #     lat*= -1
        #
        # if data[3] == 'W' :
        #     lon *= -1

        print("data n°" + str(i))
        print("coord boat :",lat,lon)
        print("coord place d'arme",lat_pa,lon_pa)
        print("dist",haversine([lat,lon],[lat_pa,lon_pa]))
        print("--------------")

    gps.close()
