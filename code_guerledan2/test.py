from math import*
import numpy as np

def coord2cart(coords,coords_ref=(48.198802, -3.014848)):
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

    return np.array([[x_tilde,y_tilde]]).T

R = 6372800 # Earth radius in meter
a1, a2 = 48.19881166666667,-3.0156366666666665
x,y = coord2cart((a1,a2)).flatten()
print(x,y)
R1 = 60
nbato = 9
alphai = nbato*2*pi/9
point_cercle = (x + R1, y)

print(point_cercle)
