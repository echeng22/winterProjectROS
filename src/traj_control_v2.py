#!/usr/bin/env python
import rospy
import numpy as np
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

    def ik_delta(self,x,y,z):
        # Takes in x,y,z coordinates and returns a list of joints angles

        # Joint 1:
        E1 = 2.*self.lowLinkLength * (y + self.modelParam['a']) # 2*L*(y + a)
        F1 = 2. * z * self.lowLinkLength # 2*z*L
        G1 = x**2 + y**2 + z**2  + self.modelParam['a']**2 + self.lowLinkLength**2 + 2.*y*self.modelParam['a'] - \
            self.upLinkLength**2 # x^2 + y^2 + z^2 + a^2 + L^2 + 2*y*a - l^2

        # Joint 2:
        E2 = -self.lowLinkLength*(np.sqrt(3)*(x + self.modelParam['b']) + y +self.modelParam['c']) # -L*(sqrt(3)*(x + b) + y +c)
        F2 = 2*z*self.lowLinkLength #2*z*L
        G2 = x**2 + y**2 + z**2 + self.modelParam['b']**2 + self.modelParam['c']**2 + self.lowLinkLength**2 + 2*x*self.modelParam['b'] + \
            2*y*self.modelParam['c'] - self.upLinkLength**2 # x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*x*b + 2*y*c - l^2

        #Joint 3
        E3 = self.lowLinkLength * (np.sqrt(3) * (x - self.modelParam['b']) - y - self.modelParam['c']) #L*(sqrt(3)*(x-b) - y -c)
        F3 = 2*z*self.lowLinkLength #2*z*L
        G3 = x**2 + y**2 + z**2 + self.modelParam['b']**2 + self.modelParam['c']**2 + self.lowLinkLength**2 - \
            2*x*self.modelParam['b'] + 2*y*self.modelParam['c'] - self.upLinkLength**2
        #x^2 + y^2 + z^2 + b^2 + c^2 + L^2 -2*x*b + 2*y*c - l^2

        E = [E1, E2, E3]
        F = [F1, F2, F3]
        G = [G1, G2, G3]
        th = [0, 0, 0]

        for i,(e,f,g) in enumerate(zip(E,F,G)):
            t1 = (-f + np.sqrt(e**2 + f**2 - g**2))/(g - e)
            t2 = (-f - np.sqrt(e ** 2 + f ** 2 - g ** 2)) / (g - e)

            th1 = 2*np.arctan(t1)
            th2 = 2 * np.arctan(t2)

            #
            # Choose angle that is in correct domain:
            # For now, we assume that the answer is t2
            th[i] = th2
        return th

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

        Kp = 1
        Ki = 1
        Kd = 1

        control_vel = ((Kp*errorAng + Ki*self.eInt + Kd*Derror) * (np.pi/180))/1000
        rospy.loginfo("Control Velocity")
        rospy.loginfo(control_vel)
        controlInfo = JointState()
        controlInfo.name = self.jointNames

        controlInfo.velocity = [control_vel[0], control_vel[1], control_vel[2]]
        self._jVelPub.publish(controlInfo)
        self.eeCounter = 0
        self.pubRate.sleep()
        self.counter += 1
        if self.counter == len(self.trajectory):
            self.counter = 0
        # else:
        #     self.eeCounter += 1

    @staticmethod
    def setUpTraj():
        # Set up a trajectory to move ee in circle or radius .1
        traj = [[], [], []]
        rads = np.linspace(0, 2 * np.pi, 1000)
        for i in range(len(rads)):
            traj[0].append(np.cos(rads[i]) * .5)
            traj[1].append(np.sin(rads[i]) * .5)
            traj[2].append(-.5)
        traj[0].extend(traj[0][::-1])
        traj[1].extend(traj[1][::-1])
        traj[2].extend(traj[2][::-1])
        return traj

def main():
    rospy.init_node('delta_control')
    test = deltaControl()
    rospy.spin()



if __name__=="__main__":
    main()

