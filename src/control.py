#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState

# Initializes a publisher and subscriber for delta robot
# Subscribes to the joint states of the delta robot
# Publishes out desired joint velocities to move robot

class deltaControl:
    def __init__(self):
        self._jAnglePub = rospy.Publisher("/Delta_base/desired_joint_pos", JointState, queue_size=10)
        # self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=10)
        self._jointSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.control)
        self.pubRate = rospy.Rate(200)
        self.jointArray1 = np.linspace(0, np.pi/4)
        self.jointArray2 = np.linspace(0, np.pi/6)
        self.jointArray3 = np.linspace(0, np.pi/3)
        self.counter = 0

    def control(self, joint_data):
        controlInfo = JointState()
        controlInfo.name = joint_data.name
        if self.counter == len(self.jointArray1):
            self.counter = 0
            self.jointArray1 = self.jointArray3[::-1]
            self.jointArray2 = self.jointArray1[::-1]
            self.jointArray3 = self.jointArray2[::-1]
        angle1 = self.jointArray1[self.counter]
        angle2 = self.jointArray2[self.counter]
        angle3 = self.jointArray3[self.counter]
        self.counter += 1
        controlInfo.position = [angle1, angle2, angle3]
        self._jAnglePub.publish(controlInfo)
        self.pubRate.sleep()



def main():
    rospy.init_node('delta_control')
    test = deltaControl()
    rospy.spin()



if __name__=="__main__":
    main()

