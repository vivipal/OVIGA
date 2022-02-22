#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


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
    x4 = 0.5
    Ax = array([[cos(x3), -x4*sin(x3)],
                [sin(x3), x4*cos(x3)]])
    y = array([[x1],
               [x2]])
    dy = array([[x4*cos(x3)],
                [x4*sin(x3)]])

    K = 100
    ddy = K*sign((w-y + (dw-dy)))

    u = inv(Ax)@ddy

    return u

def lissajou(x):
    a1, a2 = 48.19881166666667,-3.0156366666666665
    ax=init_figure(a1-50,a1+50,a2-50,a2+50)
    dt = 0.02
    R2 = 30
    T = 2*pi
    N = 1
    nbato = 1

    # for t in arange(0,30,dt) :

    k = 2*pi/T
    phi = 2*pi*nbato/N
    w=array([[a1 + R2*cos(k*t + phi)], [a2 + R2*sin(k*t + phi)]])
    dw=array([[-R2*k*sin(k*t + phi)], [k*R2*cos(k*t + phi)]])
    ddw=array([[-R2*k**2*cos(k*t + phi)], [-k**2*R2*sin(k*t + phi)]])
    return(control(x,w,dw))

if __name__ == "__main__":
    a1, a2 = 48.19881166666667,-3.0156366666666665
    ax=init_figure(a1-50,a1+50,a2-50,a2+50)
    x = array([[10],[0],[1],[1]])
    dt = 0.02
    x = array([[10],[0],[1],[1]])
    R2 = 30
    T = 2*pi
    N = 1
    nbato = 1

    for t in arange(0,30,dt) :
        clear(ax)
        # plot(L*cos(s), L*sin(3*s),color='magenta')

        k = 2*pi/T
        phi = 2*pi*nbato/N
        w=array([[a1 + R2*cos(k*t + phi)], [a2 + R2*sin(k*t + phi)]])
        dw=array([[-R2*k*sin(k*t + phi)], [k*R2*cos(k*t + phi)]])
        ddw=array([[-R2*k**2*cos(k*t + phi)], [-k**2*R2*sin(k*t + phi)]])
        draw_disk(ax,w,0.5,"red")
        u=control(x,w,dw)
        print(u)
        pause(1)
        # u=control_fb(x,w,dw,ddw)
        x = x + dt*f(x,u)
        draw_tank(x,'red')
