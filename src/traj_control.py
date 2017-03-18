#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion, Point
import time

# Initializes a publisher and subscriber for delta robot
# Subscribes to the joint states of the delta robot
# Publishes out desired joint velocities to move robot

class deltaControl:
    def __init__(self):
        self._jAnglePub = rospy.Publisher("/Delta_base/desired_joint_vel", JointState, queue_size=10)
        # self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=10)
        self._jointSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.loadJoints)
        self._eePosSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.control)
        self.pubRate = rospy.Rate(200)
        self.counter = 0
        self.jointNames = []
        self.jointAngles = []
        self.trajectory = self.setUpTraj
        self.eInt = 0
        self.eprev = 0


    def control(self, eePos):
        trajPos = np.transpose([self.trajectory[0][self.counter], self.trajectory[1][self.counter], self.trajectory[2][self.counter]])
        currentPos = np.transpose([eePos.position.x, eePos.position.y, eePos.position.z])
        error = trajPos - currentPos
        KP = 1
        KI = 0
        KD = 0
        self.eInt += error
        eDiff = error - self.eprev
        self.eprev = error

        control_vel = KP * error + KI * self.eInt + KD * eDiff

        for i in range(len(control_vel)):
            if control_vel[i] > 30:
                control_vel[i] = 30
            elif control_vel[i] < -30:
                control_vel[i] = -30

        if self.counter == len(self.trajectory[1]):
            self.counter = 0
        self.counter += 1

        controlInfo = JointState()
        controlInfo.name = self.jointNames

        controlInfo.velocity = [control_vel[0], control_vel[1], control_vel[2]]
        self._jAnglePub.publish(controlInfo)
        self.pubRate.sleep()

    def loadJoints(self, joint_data):
        self.jointNames = joint_data.name
        self.jointAngles = joint_data.position

    def setUpTraj(self):
        # Set up a trajectory to move ee in circle or radius .5
        traj = [[], [], []]
        rads = np.linspace(0,2*np.pi, 200)
        for i in range(len(rads)):
            traj[0].append(np.cos(rads[i]) * .25)
            traj[1].append(np.sin(rads[i]) * .25)
            traj[2].append(.75)
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

