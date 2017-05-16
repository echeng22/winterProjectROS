#!/usr/bin/env python
import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.constraints
import trep.visual as visual

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

# r1 = .4
# r2 = .05
# l1 = .3
# l2 = .58

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
        th[i] = th2
    return th


def fk_delta(th1, th2, th3):
    """
    Input angles of three actuated joints, output the location of the EE
    platform center expressed in the base platform center point frame.
    """
    # SPHERE PARAMETERIZATION:
    # leg 1
    x1 = 0
    y1 = -w_base - L_base*np.cos(th1) + u_platform
    z1 = -L_base*np.sin(th1)
    # leg 2
    x2 = np.sqrt(3)/2.*(w_base + L_base*np.cos(th2)) - s_platform/2.
    y2 = 1/2.*(w_base + L_base*np.cos(th2)) - w_platform
    z2 = -L_base*np.sin(th2)
    # leg 3
    x3 = -np.sqrt(3)/2.*(w_base + L_base*np.cos(th3)) + s_platform/2.
    y3 = 1/2.*(w_base + L_base*np.cos(th3)) - w_platform
    z3 = -L_base*np.sin(th3)
    # RADII
    r1 = r2 = r3 = L_platform
    if np.isclose(th1, th3) and np.isclose(th2, th3):
        zn = z1
        # INTERSECTION ALGORITHM:
        a = 2*(x3-x1)
        b = 2*(y3-y1)
        c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
        d = 2*(x3-x2)
        e = 2*(y3-y2)
        f = r2**2 - r3**2 - x2**2 - y2**2 + x3**2 + y3**2
        # SOLVE:
        x = (c*e - b*f)/(a*e - b*d)
        y = (a*f - c*d)/(a*e - b*d)
        B = -2*zn
        C = zn**2. - r1**2. + (x-x1)**2. + (y-y1)**2.
        z = (-B + np.sqrt(B**2. - 4*C))/2.
    elif np.isclose(th1, th3):
        x = y = z = 0
    elif np.isclose(th2, th3):
        x = y = z = 0
    else:
        a11 = 2*(x3-x1)
        a12 = 2*(y3-y1)
        a13 = 2*(z3-z1)
        a21 = 2*(x3-x2)
        a22 = 2*(y3-y2)
        a23 = 2*(z3-z2)
        b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
        b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2
        # equations A.6
        a1 = a11/a13 - a21/a23
        a2 = a12/a13 - a22/a23
        a3 = b2/a23 - b1/a13
        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2-a21*a5)/a23
        # eq. A.8
        a = a4**2. + 1 + a6**2.
        b = 2*a4*(a5-x1) - 2*y1 + 2*a6*(a7-z1)
        c = a5*(a5-2*x1) + a7*(a7-2*z1) + x1**2. + y1**2. + z1**2. - r1**2.
        # finally, calculate:
        # positive:
        y = -b + np.sqrt(b**2 - 4*a*c)
        # xp = a4*yp + a5
        # zp = a6*yp + a7
        # negative:
        # y = -b - np.sqrt(b**2 - 4*a*c)
        x = a4*y + a5
        z = a6*y + a7
    return x,y,z
    
    
class DeltaRobotTrep( trep.System ):
    def __init__(self, L_base, L_platform, R_base, R_platform):
        # first let's create an empty system, and build the base platform:
        super(DeltaRobotTrep, self).__init__()
        # triangle sizes:
        s_base = R_base*6./np.sqrt(3)
        s_platform = R_platform*6./np.sqrt(3)
        # center of circles to platform vertices:
        u_base = s_base*np.sqrt(3)/3.
        u_platform = s_platform*np.sqrt(3)/3.
        # center of circles to triangle edges
        w_base = R_base
        w_platform = R_platform
        # LEG 1 ATTACHMENT
        _ = trep.Frame(self.world_frame, trep.TY, -w_base, "base_attach_leg1")
        # LEG 2 ATTACHMENT
        child = trep.Frame(self.world_frame, trep.TX, w_base*np.cos(30*pi/180.))
        child = trep.Frame(child, trep.TY, w_base*np.sin(30*pi/180.))
        _ = trep.Frame(child, trep.RZ, 120*pi/180., "base_attach_leg2")
        # LEG 3 ATTACHMENT
        child = trep.Frame(self.world_frame, trep.TX, -w_base*np.cos(30*pi/180.))
        child = trep.Frame(child, trep.TY, w_base*np.sin(30*pi/180.))
        _ = trep.Frame(child, trep.RZ, 240*pi/180., "base_attach_leg3")
        # BUILD LEGS
        self.build_leg(self.get_frame("base_attach_leg1"), 1, L_base, L_platform)
        self.build_leg(self.get_frame("base_attach_leg2"), 2, L_base, L_platform)
        self.build_leg(self.get_frame("base_attach_leg3"), 3, L_base, L_platform)
        # ADD PLATFORM AND ATTACHMENTS:
        child = trep.Frame(self.world_frame, trep.TX, "platform_x", "platform_x")
        child = trep.Frame(child, trep.TY, "platform_y", "platform_y")
        self.platform_center = trep.Frame(child, trep.TZ, "platform_z", "platform_z", mass=.15)
        _ = trep.Frame(self.platform_center, trep.TY, -u_platform, "platform_attach_leg1")
        child = trep.Frame(self.platform_center, trep.TX, u_platform*np.cos(30*pi/180.))
        child = trep.Frame(child, trep.TY, u_platform*np.sin(30*pi/180.))
        _ = trep.Frame(child, trep.RZ, 120*pi/180., "platform_attach_leg2")
        child = trep.Frame(self.platform_center, trep.TX, -u_platform*np.cos(30*pi/180.))
        child = trep.Frame(child, trep.TY, u_platform*np.sin(30*pi/180.))
        _ = trep.Frame(child, trep.RZ, 240*pi/180., "platform_attach_leg3")
        # NOW ADD CONSTRAINTS
        trep.constraints.PointToPoint3D(self, "platform_attach_leg1", "leg_1_end")
        trep.constraints.PointToPoint3D(self, "platform_attach_leg2", "leg_2_end")
        trep.constraints.PointToPoint3D(self, "platform_attach_leg3", "leg_3_end")
        # ADD SPRINGS
        q_neutral = 22.5*pi/180.
        stiffness = 10
        trep.potentials.ConfigSpring(self, 'leg_1_th1', k=stiffness, q0=q_neutral)
        trep.potentials.ConfigSpring(self, 'leg_2_th1', k=stiffness, q0=q_neutral)
        trep.potentials.ConfigSpring(self, 'leg_3_th1', k=stiffness, q0=q_neutral)
        # ADD GRAVITY AND DAMPING
        trep.potentials.Gravity(self, gravity=[0,0,-9.8], name="gravity")
        trep.forces.Damping(self, 0.05)
        return

    def build_leg(self, attach, n, L, l):
        # upper leg:
        child = trep.Frame(attach, trep.RX, "leg_%d_th1"%n, "leg_%d_th1"%n)
        child = trep.Frame(child, trep.TY, -L/2., "leg_%d_upper_inertial"%n, mass=.10)
        child = trep.Frame(child, trep.TY, -L/2., "leg_%d_upper_end"%n)
        # first universal joint:
        child = trep.Frame(child, trep.RX, "leg_%d_th2"%n, "leg_%d_th2"%n)
        child = trep.Frame(child, trep.RZ, "leg_%d_th3"%n, "leg_%d_th3"%n)
        # lower leg:
        child = trep.Frame(child, trep.TY, -l/2, "leg_%d_lower_inertial"%n, mass=.07)
        child = trep.Frame(child, trep.TY, -l/2., "leg_%d_lower_end"%n)
        # second universal joint:
        child = trep.Frame(child, trep.RZ, "leg_%d_th4"%n, "leg_%d_th4"%n)
        child = trep.Frame(child, trep.RX, "leg_%d_th5"%n, "leg_%d_end"%n)
        return

    def forward(self, th1, th2, th3):
        q1 = self.get_config('leg_1_th1')
        q2 = self.get_config('leg_2_th1')
        q3 = self.get_config('leg_3_th1')
        q1.q = th1
        q2.q = th2
        q3.q = th3
        self.satisfy_constraints(constant_q_list=[q1,q2,q3])
        g = self.platform_center.g()
        return g[0:3,-1]

    def inverse(self, x, y, z):
        x_f = self.get_config('platform_x')
        y_f = self.get_config('platform_y')
        z_f = self.get_config('platform_z')
        x_f.q = x
        y_f.q = y
        z_f.q = z
        self.satisfy_constraints(constant_q_list=[x_f, y_f, z_f])
        return [
            self.q[self.get_config('leg_1_th1').index],
            self.q[self.get_config('leg_2_th1').index],
            self.q[self.get_config('leg_3_th1').index]
            ]


def main():
    # first let's build a system:
    sys = DeltaRobotTrep(L_base, L_platform, R_base, R_platform)

    dt = 0.05
    tf = 10.0

    # setup initial conditions
    _ = sys.forward(*(-0.2*np.ones(3)))
    _ = sys.inverse(0.0,0.0,1.2)
    q0 = sys.q


    # add external force:
    trep.forces.BodyWrench(sys, sys.platform_center, (0, 0, "external_z", 0, 0, 0))
    def f_func(t):
        if t>3.0:
            return 50
        else:
            return 0
        # amp = 20
        # period = 1.0
        # return amp*np.sin(t*2*pi/period) + 40
    
    mvi = trep.MidpointVI(sys)
    mvi.initialize_from_configs(0.0, q0, dt, q0)
    q = [mvi.q2]
    t = [mvi.t2]
    while mvi.t1 < tf:
        mvi.step(mvi.t2+dt, u1=[f_func(mvi.t2)])
        q.append(mvi.q2)
        t.append(mvi.t2)
    sys = DeltaRobotTrep(L_base, L_platform, R_base, R_platform)
    visual.visualize_3d([ visual.VisualItem3D(sys, t, q) ])

if __name__ == '__main__':
    main()
