import numpy as np
from math import pi

#################################
# SPECIFIED DIMENSIONS (meters) #
#################################
# radius of the base and platform:
R_base = 0.164
R_platform = 0.022
# leg lengths
L_base = 0.524
L_platform = 1.244


#########################
# CALCULATED DIMENSIONS #
#########################
# size of equilateral triangles:
s_base = R_base*6./np.sqrt(3)
s_platform = R_platform*6./np.sqrt(3)
# center of circles to platform vertices:
u_base = s_base*np.sqrt(3)/3.
u_platform = s_platform*np.sqrt(3)/3.
# center of circles to triangle edges
w_base = R_base
w_platform = R_platform
# useful, arbitrary constants:
a = w_base - u_platform
b = s_platform/2. - np.sqrt(3)/2.*w_base
c = w_platform - 1/2.*w_base


#########################
# KINEMATICS ALGORITHMS #
#########################
def ik_delta(x,y,z):
    """
    Input values represent the desired location of the output platform center
    point expressed in the base platform center point frame.

    Output is the angle of the three actuated axes.
    """
    # joint one:
    E1 = 2.*L_base*(y+a)
    F1 = 2.*z*L_base
    G1 = x**2. + y**2. + z**2. + a**2. + L_base**2. + 2.*y*a - L_platform**2.
    # joint two:
    E2 = -L_base*(np.sqrt(3)*(x+b) + y + c)
    F2 = 2.*z*L_base
    G2 = x**2. + y**2. + z**2. + b**2. + c**2. + L_base**2. + 2.*x*b + 2*y*c - L_platform**2.
    # joint three:
    E3 = L_base*(np.sqrt(3)*(x-b) - y - c)
    F3 = 2.*z*L_base
    G3 = x**2. + y**2. + z**2. + b**2. + c**2. + L_base**2. - 2.*x*b + 2*y*c - L_platform**2.
    # loop:
    E = [E1, E2, E3]
    F = [F1, F2, F3]
    G = [G1, G2, G3]
    th = [0, 0, 0]
    for i,(e,f,g) in enumerate(zip(E,F,G)):
        t1 = (-f + np.sqrt(e**2. + f**2. - g**2))/(g-e)
        t2 = (-f - np.sqrt(e**2. + f**2. - g**2))/(g-e)
        th1 = 2*np.arctan(t1)
        th2 = 2*np.arctan(t2)
        # now choose angle that is in correct domain:
        # for now, let's just assume the answer is t2
        th[i] = th2 * 180/pi
    return th

print ik_delta(.3,.5,-1.1)