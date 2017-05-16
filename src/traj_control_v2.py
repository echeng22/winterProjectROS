#!/usr/bin/env python
import rospy
import numpy as np
from math import cos, sin, pi, radians, atan
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion, Point

# Initializes a publisher and subscriber for delta robot
# Subscribes to the joint states of the delta robot
# Publishes out desired joint velocities to move robot

class deltaControl(object):

    def __init__(self):
        # self._jAnglePub = rospy.Publisher("/Delta_base/desired_joint_pos", JointState)
        self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=1) # Publishes joint velocities
        self._jointSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.loadJointsPos) # Sub to joint positions

        # Currently using obtaining the EE position from the simulation instead of using FK algorithm.
        # This is due to how complex the FK solution is, and is currently being looked at in understanding it.
        self._eePosSub = rospy.Subscriber("/Delta_base/ee_pos", Pose, self.control) # Sub to ee_pos

        self._baseParamSub = rospy.Subscriber("/Delta_base/base_radius", Float64, self.setBase) # Sub to base radius
        self._eeParamSub = rospy.Subscriber("/Delta_base/ee_radius", Float64, self.setEE) # Sub to ee radiues
        self._lowLinkParamSub = rospy.Subscriber("/Delta_base/lower_link", Float64, self.setLowerLink) # Sub to lower link length
        self._upLinkParamSub = rospy.Subscriber("/Delta_base/upper_link", Float64, self.setUpperLink) # Sub to upper link length
        self.num_param = 4 # Number of params to load when the model is set up in V-REP
        self.updateCount = 0 # Keeps track of how many params are loaded

        self.jointNames = [] # Stores revolute joint name
        self.jointAngles = [] # Stores joint angle

        # Dictionary that will contain model parameters. Keys are {'s_base', 's_ee','u_base', 'u_ee','w_base', 'w_ee', 'a', 'b', 'c'}
        self.modelParam = {}
        #Link and base values
        self.lowLinkLength = 0
        self.upLinkLength = 0
        self.rBase = 0
        self.rEE = 0
        self.eeCounter = 0

        # self.desiredPos = {} # Subscribes to IK desired position from another node....

        #Trajectory Parameters
        self.trajectory = self.setUpTraj()
        self.prevTrajAngle = 0
        self.counter = 0

        # Control Parameters
        self.eInt = 0
        self.eprev = 0
        self.prevJAngle = 0
        self.dt = .01

        self.pubRate = rospy.Rate(100)

    def loadIK(self):
        if self.updateCount == self.num_param:
            self.setUpIK()
            self.updateCount = 0

    def setBase(self,value):
        self.rBase = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setEE(self,value):
        self.rEE = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setLowerLink(self,value):
        self.lowLinkLength = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setUpperLink(self,value):
        self.upLinkLength = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def loadJointsPos(self, joint_data):
        self.jointNames = joint_data.name
        self.jointAngles = joint_data.position

    def setUpIK(self):
        # Source of IK algorithm: http://www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf
        # Adjustments were made to account for design differences (Model uses circles as base/ee, paper uses triangles)
        #
        # Note: From the paper, the model base we designed would act as an inscribed circle in the equilateral triangle
        #       and the model ee would act as a circumscribed circle around the equilateral triangle.

        #------------------------------------------------------------------------------------------------
        # Calculated values
        # Size of equilateral triangles of base and EE
        s_base = self.rBase * 6./np.sqrt(3)
        s_ee = self.rEE * 6./np.sqrt(3)
        self.modelParam.update({'s_base': s_base})
        self.modelParam.update({'s_ee': self.rEE * 6. / np.sqrt(3)})

        # Distance from vertices of equilateral triangles to center
        u_base = s_base * np.sqrt(3)/3
        u_ee = self.rEE
        self.modelParam.update({'u_base': u_base})
        self.modelParam.update({'u_ee': u_ee})

        # Distance from midpoint of equilateral triangles sides to center
        w_base = self.rBase
        w_ee = s_ee * np.sqrt(3) / 3
        self.modelParam.update({'w_base': w_base})
        self.modelParam.update({'w_ee': w_ee})

        # Constants used in IK calculations
        a = w_base - u_ee
        b = (s_ee/2) - (np.sqrt(3)*w_base/2)
        c = w_ee - w_base/2
        self.modelParam.update({'a': a})
        self.modelParam.update({'b': b})
        self.modelParam.update({'c': c})

    def ik_delta(self, x, y, z):
        p = np.array([x, y, z])
        blist = np.array([[self.rBase, 0, 0],
                          [self.rBase * cos(radians(120)), self.rBase * sin(radians(120)), 0],
                          [self.rBase * cos(radians(240)), self.rBase * sin(radians(240)), 0]])
        plist = np.array([[self.rEE, 0, 0],
                          [self.rEE * cos(radians(120)), self.rEE * sin(radians(120)), 0],
                          [self.rEE * cos(radians(240)), self.rEE * sin(radians(240)), 0]])

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
            G[0][i] = self.upLinkLength ** 2 - self.lowLinkLength ** 2 - self.distance(p, bmplist[i]) ** 2
            E[0][i] = 2 * p[2] * self.lowLinkLength + 2 * pmblist[i][2] * self.lowLinkLength
            F[0][i] = 2 * self.lowLinkLength * (
            (p[0] + pmblist[i][0]) * cos(radians((i + 1) * 120 - 120)) + (p[1] + pmblist[i][1]) * sin(
                radians((i + 1) * 120 - 120)))
            tp[0][i] = (-F[0][i] + np.sqrt(E[0][i] ** 2 + F[0][i] ** 2 - G[0][i] ** 2)) / (G[0][i] - E[0][i])
            tm[0][i] = (-F[0][i] - np.sqrt(E[0][i] ** 2 + F[0][i] ** 2 - G[0][i] ** 2)) / (G[0][i] - E[0][i])
            thetap[0][i] = 2 * atan(tp[0][i])
            thetam[0][i] = 2 * atan(tm[0][i])
            if -pi / 4 <= thetap[0][i] and thetap[0][i] <= pi / 2:
                print np.isreal(thetap[0][i])
                if np.isreal(thetap[0][i]):
                    theta[i] = thetap[0][i]

            if -pi / 4 <= thetam[0][i] and thetam[0][i] <= pi / 2:
                if np.isreal(thetam[0][i]):
                    theta[i] = thetam[0][i]
        return np.array(theta)*-1

    def control(self, eePos):
        # if self.eeCounter == 10:
        desiredX = self.trajectory[0][self.counter]
        desiredY = self.trajectory[1][self.counter]
        desiredZ = self.trajectory[2][self.counter]
        rospy.loginfo("Trajectory XYZ")
        rospy.loginfo(desiredX)
        rospy.loginfo(desiredY)
        rospy.loginfo(desiredZ)
        trajPos = np.transpose(self.ik_delta(desiredX, desiredY , desiredZ)) # Turn desired X,Y,Z position into desired joint angles
        rospy.loginfo("Desired Joint Angles")
        rospy.loginfo(trajPos)
        self.prevTrajAngle = trajPos
        rospy.loginfo("Current Joint Angles:")
        rospy.loginfo(self.jointAngles)
        curAng = np.transpose(self.jointAngles)

        self.prevJAngle = curAng

        errorAng = trajPos - curAng
        self.eInt += errorAng

        Derror = (errorAng - self.eprev)/self.dt
        self.eprev = errorAng

        Kp = 400
        Ki = 10
        Kd = 1

        control_vel = ((Kp*errorAng + Ki*self.eInt + Kd*Derror) * (np.pi/180))
        rospy.loginfo("Control Velocity")
        rospy.loginfo(control_vel)
        controlInfo = JointState()
        controlInfo.name = self.jointNames

        controlInfo.velocity = [control_vel[0], control_vel[1], control_vel[2]]
        self._jVelPub.publish(controlInfo)
        self.eeCounter = 0
        self.pubRate.sleep()
        self.counter += 1
        print self.counter
        if self.counter == len(self.trajectory[0]):
            self.counter = 0
        # else:
        #     self.eeCounter += 1

    @staticmethod
    def setUpTraj():
        # Set up a trajectory to move ee in circle or radius .1
        radius = .25
        traj = [[], [], []]
        rads = np.linspace(0, 2 * np.pi, 250)
        for i in range(len(rads)):
            traj[0].append(np.cos(rads[i]) * radius)
            traj[1].append(np.sin(rads[i]) * radius)
            traj[2].append(.6)
        traj[0].extend(traj[0][::-1])
        traj[1].extend(traj[1][::-1])
        traj[2].extend(traj[2][::-1])
        return traj

    @staticmethod
    def distance(a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

def main():
    rospy.init_node('delta_control')
    test = deltaControl()
    rospy.spin()



if __name__=="__main__":
    main()

