#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import deltaModule as delta

# Initializes a publisher and subscriber for delta robot
# Subscribes to the joint states of the delta robot
# Publishes out desired joint velocities to move robot

class deltaControl(object):

    def __init__(self):
        # self._jAnglePub = rospy.Publisher("/Delta_base/desired_joint_pos", JointState)
        self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=1) # Publishes joint velocities
        self._jointSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.loadJointsPos) # Sub to joint positions


        # Subscribers to model paramters
        self._baseParamSub = rospy.Subscriber("/Delta_base/base_radius", Float64, self.setBase) # Sub to base radius
        self._eeParamSub = rospy.Subscriber("/Delta_base/ee_radius", Float64, self.setEE) # Sub to ee radiues
        self._lowLinkParamSub = rospy.Subscriber("/Delta_base/lower_link", Float64, self.setLowerLink) # Sub to lower link length
        self._upLinkParamSub = rospy.Subscriber("/Delta_base/upper_link", Float64, self.setUpperLink) # Sub to upper link length
        self.num_param = 4 # Number of params to load when the model is set up in V-REP
        self.updateCount = 0 # Keeps track of how many params are loaded

        #Arrays to store subscribed angle values
        self.jointNames = [] # Stores revolute joint name
        self.jointAngles = [] # Stores joint angle

        #Dictionary of link and base values
        self.modelParams = dict.fromkeys(('rBase', 'rEE', 'lowLinkLength', 'upLinkLength'))

        #Trajectory Parameters
        self.trajectory = 0
        self.counter = 0

        # Control Parameters
        self.eInt = 0
        self.eprev = 0
        self.dt = .01

        # Publishing Rate. Currently 100 Hz
        self.pubRate = rospy.Rate(100)

    def loadIK(self):
        if self.updateCount == self.num_param:
            self.trajectory = self.setUpTraj()
            rospy.set_param('model_param',self.modelParams)
            self.updateCount = 0
        else:
            rospy.logerr("Not enough parameters!. %d parameters loaded!", self.updateCount)

    def setBase(self,value):
        self.modelParams['rBase'] = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setEE(self,value):
        self.modelParams['rEE'] = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setLowerLink(self,value):
        self.modelParams['lowLinkLength'] = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def setUpperLink(self,value):
        self.modelParams['upLinkLength'] = value.data
        self.updateCount = self.updateCount + 1
        self.loadIK()

    def loadJointsPos(self, joint_data):
        self.jointNames = joint_data.name
        self.jointAngles = joint_data.position
        self.control()

    def control(self):
        desiredX = self.trajectory[0][self.counter]
        desiredY = self.trajectory[1][self.counter]
        desiredZ = self.trajectory[2][self.counter]

        # Converted desired trajectory point into desired joint angle values
        trajPos = np.transpose(delta.ik_delta(desiredX, desiredY , desiredZ, self.modelParams)) # Turn desired X,Y,Z position into desired joint angles

        # Process current joint angles
        curAng = np.transpose(self.jointAngles)

        #Calculate error between desired joint angles and current joint angles. Updates error integral term
        errorAng = trajPos - curAng
        self.eInt += errorAng

        #Calculated derivative error term. Updates the previous error value
        Derror = (errorAng - self.eprev)/self.dt
        self.eprev = errorAng

        # PID Parameters
        Kp = 400
        Ki = 10
        Kd = 1

        # Calculate the control velocity
        control_vel = ((Kp*errorAng + Ki*self.eInt + Kd*Derror) * (np.pi/180))

        # Create a JointState message that will be published with control velocity values
        controlInfo = JointState()
        controlInfo.name = self.jointNames
        controlInfo.velocity = [control_vel[0], control_vel[1], control_vel[2]]

        # Publish JointState message. Sleeps by publishing rate to simulate 100 Hz control system
        self._jVelPub.publish(controlInfo)
        self.pubRate.sleep()

        # Update trajectory path
        self.counter += 1
        if self.counter == len(self.trajectory[0]):
            self.counter = 0

    # Function that builds the circle trajectory the EE will follow.

    def setUpTraj(self):
        # Set up a trajectory to move ee in circle or radius .1
        radius = .25
        traj = [[], [], []]
        rads = np.linspace(0, 2 * np.pi, 250)
        for i in range(len(rads)):
            traj[0].append(np.cos(rads[i]) * radius)
            traj[1].append(np.sin(rads[i]) * radius)
            traj[2].append(self.modelParams['upLinkLength'])
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

