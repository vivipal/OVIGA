import numpy as np
import numpy.linalg as npl
import math
from math import cos,sin

x1 = np.array([[-919,-3237.5,6216]]).T
xm1 = np.array([[5090.5,-3069.5,5981]]).T
x2 = np.array([[1578.5,-18.5,6089]]).T
x3 = np.array([[1794.5,-3517.5,8968.5]]).T
b = -(x1+xm1)/2
beta = 46000e-9 # champ magnétique a la surface de la terre
A = 1/beta * np.hstack((x1+b,x2+b,-x3-b))


R = 6372800 # Earth radius in meter

def regu_mot(w_cons, w_reel, u, K):
    err = w_cons - w_reel
    u += K*err
    return u

def cmd_moteur(encod,ard,w1_cons,w2_cons,Km1=0.2,Km2=0.2):
    """ Fonction régulation en vitesse des moteurs """
    encod.get_sync()

    sync,data_encoders = encod.read_packet(debug=False)
    odo1.pop(0)
    odo2.pop(0)
    T.pop(0)
    odo1.append(data_encoders[3])
    odo2.append(data_encoders[4])
    t0 = time.time()
    T.append(t0)

    w1_reel = (odo1[2] - odo1[0]) / (8*(T[2]-T[0]))
    w2_reel = (odo2[0] - odo2[2]) / (8*(T[2]-T[0]))

    u1 = max(regu_mot(w1_cons, w1_reel, u1, Km1),350)
    u2 = max(regu_mot(w2_cons, w2_reel, u2, Km2),350)

    ard.send_arduino_cmd_motor(u1,u2)

    return odo1, odo2, T

def correct_measure(raw) :

    return  npl.inv(A)@(np.array([raw]).T+b)

def get_compass_from_raw(raw):

    y = correct_measure(raw)

    return (np.arctan2(y[1],y[0])*180/np.pi)[0]


def coords_dist(coord1,coord2):
    '''
    return distance between 2 coords points
    '''

    lat1, lon1 = coord1
    lat2, lon2 = coord2

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi       = math.radians(lat2 - lat1)
    dlambda    = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2

    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))


def coords_bearing(coord1,coord2):
    '''
    return bearing between 2 coords points
    '''

    lat1, lon1 = coord1
    lat2, lon2 = coord2

    y = sin(lon2-lon1) * cos(lat2);
    x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
    theta = np.arctan2(y, x);
    bearing = (theta*180/np.pi + 360) % 360;

    return bearing


def cvt_gll_ddmm_2_dd (st):
    '''
    convert ddmm to rad
    '''

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


def coord2cart(coords,coords_ref):
    '''
    in :
        coords = (lat,lon)
        coords_ref = centre du plan
    return :
        x_tilde et y_tilde coord sur le plan centre en coord_ref
    '''


    ly,lx = coords
    lym,lxm = coords_ref

    x_tilde = R * cos(ly*np.pi/180)*(lx-lxm)*np.pi/180
    y_tilde = R * (ly-lym)*np.pi/180

    return np.array([x_tilde,y_tilde]).T

def follow_line(a,b,m,r=6) :
    '''
    in :
        a et b sont les 2 points qui forment la ligne
        m est le point du bateau
        r precision en metre (par défaut 6m)

    return  :
        theta barre cap de consigne
    '''


    u = (b-a)/npl.norm(b-a)
    e = npl.det(np.hstack((u,m-a)))
    phi = np.arctan2((b-a)[1],(b-a)[0])
    theta_bar = phi - np.arctan(e/r)

    return theta_bar[0]


def follow_line_coord(ca,cb,cm,r=6) :
    '''
    in :
        ca et cb sont les 2 points en lat lon qui forment la ligne
        cm est le point du bateau en lat lon
        r precision en metre (par défaut 6m)

    return  :
        theta barre cap de consigne
    '''

    a = coord2cart(ca)
    b = coord2cart(cb)
    m = coord2cart(cm)

    return follow_line(a,b,m,r)



def waypoint_passed(wp,m):
    return (b-a)@(m-b).T > 0




if __name__ =='__main__' :


    # # test coord2cart()
    # ref = (  48.198768 ,-3.014307)
    # c = ( 48.199466, -3.016688  )
    #
    # print(coord2cart(c,ref))


    # test follow_line()
    import matplotlib.pyplot as plt
    a = np.array([[0,0]]).T
    b = np.array([[18,9]]).T

    m = np.array([[7.5,4]]).T

    plt.figure()

    plt.plot((a[0],b[0]),(a[1],b[1]),marker='o')


    for i in np.arange(0,17.5,0.6):
        for j in np.arange(0,10,0.6):
            m = np.array([[i,j]]).T

            plt.scatter(m[0],m[1],color='red')
            theta_bar = follow_line(a,b,m,6)
            d=0.5
            plt.arrow(m[0,0],m[1,0],d*cos(theta_bar),d*sin(theta_bar),head_width=0.1,head_length=0.3)

    plt.grid()
    plt.show()
