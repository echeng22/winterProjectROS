#!/usr/bin/env python
import rospy
import numpy as np
from math import cos, sin, pi, radians, atan
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion, Point

class joyStick(object):

    def __init__(self):
        # Reads the mode of the simulation. If true, read inputs from the PS3 controller.
        self._modeSub = rospy.Subscriber("/eeMode", Bool, self.setBase)  # Sub to base radius

        # Publishes desired joint velocities
        self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=1)  # Publishes joint velocities

        self.modelParams = self.loadModelParams()
        self.homePos = [0,0,0]


    def loadModelParams(self):
