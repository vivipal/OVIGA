#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
from ddboat_tools import *

def f(x,u):
    x,u=x.flatten(),u.flatten()
    xdot = array([[x[3]*cos(x[2])],[x[3]*sin(x[2])],[u[1]],[u[0]]])
    return(xdot)

def control_fb(x,w,dw,ddw):
    x1, x2, x3, x4 = x.flatten()

    Ax = array([[cos(x3), -x4*sin(x3)],
                [sin(x3), x4*cos(x3)]])
    y = array([[x1],
               [x2]])
    dy = array([[x4*cos(x3)],
                [x4*sin(x3)]])

    ddy = w-y + 2*(dw-dy) + ddw

    u = inv(Ax)@ddy

    return u

def control(x,w,dw):
    x1, x2, x3, x4 = x.flatten()
    # x4 = 0.5
    Ax = array([[cos(x3), -x4*sin(x3)],
                [sin(x3), x4*cos(x3)]])
    y = array([[x1],
               [x2]])
    dy = array([[x4*cos(x3)],
                [x4*sin(x3)]])
    K = 5
    ddy = K*sign((w-y + (dw-dy)))
    u = inv(Ax)@ddy
    print(u)
    return u

def lissajou(x,t,centre=(48.19881166666667,-3.0156366666666665),R=30,i=9,N=9,T=120):
    a1,a2 = coord2cart(centre).flatten()

    w  =array([ [a1 + R*cos(2*np.pi*(t/T+i/N))]               , [a2 + R*sin(4*np.pi*(t/T+i/N))] ])
    dw =array([ [-(2/T)*np.pi*R*sin(2*np.pi*(t/T+i/N))]       , [(4/T)*np.pi*R*cos(4*np.pi*(t/T+i/N))] ])
    ddw=array([ [-((2/T)*np.pi)**2 *R*cos(2*np.pi*(t/T+i/N))] , [-((4/T)*np.pi)**2 *R*sin(4*np.pi*(t/T+i/N))] ])
    draw_disk(ax,w,0.5,"red")
    return(control(x,w,dw))

if __name__ == "__main__":
    centre=(48.19881166666667,-3.0156366666666665)
    a1,a2 = coord2cart(centre).flatten()

    x = array([[a1],[a2],[0.1],[0.1]])

    ax=init_figure(a1-50,a1+50,a2-50,a2+50)
    dt=0.05
    for t in arange(0,30,dt) :
        clear(ax)
        u=lissajou(x,t)
        x = x + dt*f(x,u)
        draw_tank(x,'red')
