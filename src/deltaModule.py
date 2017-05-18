import numpy as np
from math import cos, sin, radians, atan, pi

def distance(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

# Parameters are x, y, z coordinates and a dictionary of model parameters
def ik_delta(x, y, z, modelParams):
    p = np.array([x, y, z])
    blist = np.array([[modelParams['rBase'], 0, 0],
                      [modelParams['rBase'] * cos(radians(120)), modelParams['rBase'] * sin(radians(120)), 0],
                      [modelParams['rBase'] * cos(radians(240)), modelParams['rBase'] * sin(radians(240)), 0]])
    plist = np.array([[modelParams['rEE'], 0, 0],
                      [modelParams['rEE'] * cos(radians(120)), modelParams['rEE'] * sin(radians(120)), 0],
                      [modelParams['rEE'] * cos(radians(240)), modelParams['rEE'] * sin(radians(240)), 0]])

    pmblist = plist - blist
    bmplist = blist - plist

    G = np.zeros((1, 3))
    E = np.zeros((1, 3))
    F = np.zeros((1, 3))
    tp = np.zeros((1, 3))
    tm = np.zeros((1, 3))
    thetap = np.zeros((1, 3))
    thetam = np.zeros((1, 3))
    theta = [None, None, None]

    for i in range(3):
        G[0][i] = modelParams['upLinkLength'] ** 2 - modelParams['lowLinkLength'] ** 2 - distance(p, bmplist[i]) ** 2
        E[0][i] = 2 * p[2] * modelParams['lowLinkLength'] + 2 * pmblist[i][2] * modelParams['lowLinkLength']
        F[0][i] = 2 * modelParams['lowLinkLength'] * (
            (p[0] + pmblist[i][0]) * cos(radians((i + 1) * 120 - 120)) + (p[1] + pmblist[i][1]) * sin(
                radians((i + 1) * 120 - 120)))
        tp[0][i] = (-F[0][i] + np.sqrt(E[0][i] ** 2 + F[0][i] ** 2 - G[0][i] ** 2)) / (G[0][i] - E[0][i])
        tm[0][i] = (-F[0][i] - np.sqrt(E[0][i] ** 2 + F[0][i] ** 2 - G[0][i] ** 2)) / (G[0][i] - E[0][i])
        thetap[0][i] = 2 * atan(tp[0][i])
        thetam[0][i] = 2 * atan(tm[0][i])
        if -pi / 4 <= thetap[0][i] <= pi / 2:
            print np.isreal(thetap[0][i])
            if np.isreal(thetap[0][i]):
                theta[i] = thetap[0][i]

        if -pi / 4 <= thetam[0][i] <= pi / 2:
            if np.isreal(thetam[0][i]):
                theta[i] = thetam[0][i]
    return np.array(theta) * -1

if __name__=="__main__":
    print "ERROR"