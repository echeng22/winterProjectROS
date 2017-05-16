import numpy as np
from math import pi, cos, radians, sin, atan

def distance(a,b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

#################################
# SPECIFIED DIMENSIONS (meters) #
#################################
# radius of the base and platform:
# R_base = 0.164
# R_platform = 0.022
# # leg lengths
# L_base = 0.524
# L_platform = 1.244
# R_base = 1
# R_platform = 1
# # leg lengths
# L_base = 1
# L_platform = 1



R_base = .4
R_platform = .05
L_base = .3
L_platform = .58
def ikDelta(x,y,z):
    p = np.array([x,y,z])
    blist = np.array([[R_base, 0, 0],
             [R_base*cos(radians(120)), R_base*sin(radians(120)), 0],
             [R_base*cos(radians(240)), R_base*sin(radians(240)), 0]])
    plist = np.array([[R_platform, 0, 0],
             [R_platform*cos(radians(120)), R_platform*sin(radians(120)), 0],
             [R_platform*cos(radians(240)), R_platform*sin(radians(240)), 0]])

    pmblist = plist - blist
    bmplist = blist - plist

    G = np.zeros((1,3))
    E = np.zeros((1,3))
    F = np.zeros((1,3))
    tp = np.zeros((1,3))
    tm = np.zeros((1,3))
    thetap = np.zeros((1,3))
    thetam = np.zeros((1,3))
    theta = [None, None, None]


    for i in range(3):
        print i
        G[0][i] = L_platform**2 - L_base**2 - distance(p,bmplist[i])**2
        E[0][i] = 2*p[2]*L_base + 2*pmblist[i][2]*L_base
        F[0][i] = 2*L_base*((p[0] + pmblist[i][0])*cos(radians((i + 1)*120-120)) + (p[1]+pmblist[i][1])*sin(radians((i + 1)*120-120)))
        tp[0][i] = (-F[0][i] + np.sqrt(E[0][i]**2 + F[0][i]**2 - G[0][i]**2))/(G[0][i] - E[0][i])
        tm[0][i] = (-F[0][i] - np.sqrt(E[0][i]**2 + F[0][i]**2 - G[0][i]**2))/(G[0][i] - E[0][i])
        thetap[0][i] = 2*atan(tp[0][i])
        thetam[0][i] = 2*atan(tm[0][i])
        if -pi/4 <= thetap[0][i] and thetap[0][i] <= pi/2:
            print np.isreal(thetap[0][i])
            if np.isreal(thetap[0][i]):
                theta[i] = thetap[0][i]

        if -pi/4 <= thetam[0][i] and thetam[0][i] <= pi/2:
            if np.isreal(thetam[0][i]):
                theta[i] = thetam[0][i]
    return np.array(theta)




print ikDelta(-.0299,.1119,.5708)*-180/pi