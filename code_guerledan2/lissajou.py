#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


def f(x,u):
    x,u=x.flatten(),u.flatten()
    xdot = array([[x[3]*cos(x[2])],[x[3]*sin(x[2])],[u[0]],[u[1]]])
    return(xdot)

def control(x,w,dw,ddw):
    x1, x2, x3, x4 = x.flatten()

    Ax = array([[cos(x3), -x4*sin(x3)],
                [sin(x3), x4*cos(x3)]])
    y = array([[x1],
               [x2]])
    dy = array([[x4*cos(x3)],
                [x4*sin(x3)]])

    K = 100
    ddy = K*sign((w-y + 2*(dw-dy)) + ddw)

    u = inv(Ax)@ddy

    return u



dt = 0.02
x = array([[10],[0],[1],[1]])
L=10
s = arange(0,2*pi,0.01)
R2 = 30
T = 10
N = 9
nbato = 12
a1, a2 = 0,0

for t in arange(0,30,dt) :

    k = 2*pi/T
    phi = 2*pi*nbato/N
    w=array([[a1 + R2*cos(k*t + phi)], [a2 + R2*sin(k*t + phi)]])
    dw=array([[-R2*k*sin(k*t + phi)], [k*R2*cos(k*t + phi)]])
    ddw=array([[-R2*k**2*cos(k*t + phi)], [-k**2*R2*sin(k*t + phi)]])

    u=control(x,w,dw,ddw)

    x = x + dt*f(x,u)
