#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Joy
import numpy as np
import modelModule as model
import deltaModule as delta

class joyStick(object):

    def __init__(self):
        # Reads the mode of the simulation. If true, read inputs from the PS3 controller.
        self._modeSub = rospy.Subscriber("/eeMode", Bool, self.setMode)  # Sub to base radius
        self._jointSub = rospy.Subscriber("/Delta_base/rev_joint", JointState, self.loadJointsPos)  # Sub to joint positions
        self._joySub = rospy.Subscriber("/joy", Joy,
                                          self.updatePos)  # Sub to joint positions

        # Publishes desired joint velocities
        self._jVelPub = rospy.Publisher("/Delta_base/joint_vel", JointState, queue_size=1)  # Publishes joint velocities

        self.mode = False
        self.modelParams = model.getModelParams()
        self.homePos = [0,0,self.modelParams["upLinkLength"]]
        self.initPos = [0,0,self.modelParams["upLinkLength"]]

        # Arrays to store subscribed angle values
        self.jointNames = []  # Stores revolute joint name
        self.jointAngles = []  # Stores joint angle

        # Arrays to store subscribed joystick axes values
        self.joyAxes = []  # Stores joystick inputs from controller

        # Control Parameters
        self.eInt = [0, 0, 0]
        self.eprev = 0
        self.dt = .01

        # Publishing Rate. Currently 100 Hz
        self.pubRate = rospy.Rate(100)

    def loadJointsPos(self, joint_data):
        self.jointNames = joint_data.name
        self.jointAngles = joint_data.position
        self.control()

    def updatePos(self, joy_data):
        self.joyAxes = joy_data.axes

    def setMode(self, mode_data):
        self.mode = mode_data.data
        if not self.mode:
            self.initPos = self.homePos

    def control(self):
        if (self.mode):
            xVal = self.joyAxes[0]  # Left Joystick side to side
            yVal = self.joyAxes[1]  # Left Joystick up and down
            zVal = self.joyAxes[3]  # Right Joystick up and down
            scaleFactor = 100
            check = np.array(self.initPos) + np.array([-yVal / scaleFactor, -xVal / scaleFactor, zVal / scaleFactor])
            rospy.loginfo(check)
            valid = delta.ik_delta(check[0], check[1], check[2], self.modelParams)
            rospy.loginfo("valid")
            rospy.loginfo(valid)
            rospy.loginfo(None in valid)
            if not (None in valid.tolist()):
                self.initPos = check
                # Converted desired trajectory point into desired joint angle values
                trajPos = np.transpose(valid)  # Turn desired X,Y,Z position into desired joint angles

                # Process current joint angles
                curAng = np.transpose(self.jointAngles)
                rospy.loginfo("curAng")
                rospy.loginfo(curAng)
                rospy.loginfo("trajPos")
                rospy.loginfo(trajPos)

                # Calculate error between desired joint angles and current joint angles. Updates error integral term
                errorAng = trajPos - curAng
                self.eInt += errorAng

                # Calculated derivative error term. Updates the previous error value
                Derror = (errorAng - self.eprev) / self.dt
                self.eprev = errorAng

                # PID Parameters
                Kp = 400
                Ki = 10
                Kd = 1

                # Calculate the control velocity
                control_vel = ((Kp * errorAng + Ki * self.eInt + Kd * Derror) * (np.pi / 180))

                # Create a JointState message that will be published with control velocity values
                controlInfo = JointState()
                controlInfo.name = self.jointNames
                controlInfo.velocity = [control_vel[0], control_vel[1], control_vel[2]]

                # Publish JointState message. Sleeps by publishing rate to simulate 100 Hz control system
                self._jVelPub.publish(controlInfo)
                self.pubRate.sleep()

def main():
    rospy.init_node('delta_control')
    joy = joyStick()
    rospy.spin()

if __name__=="__main__":
    main()


