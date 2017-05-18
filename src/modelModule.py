#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class modelParams(object):
    def __init__(self):
        # Subscribers to model paramters
        self.modelParams = dict.fromkeys(('rBase', 'rEE', 'lowLinkLength', 'upLinkLength'))
        # self.modelUpdate = 0

        self._baseParamSub = rospy.Subscriber("/Delta_base/base_radius", Float64, self.setBase) # Sub to base radius
        self._eeParamSub = rospy.Subscriber("/Delta_base/ee_radius", Float64, self.setEE) # Sub to ee radiues
        self._lowLinkParamSub = rospy.Subscriber("/Delta_base/lower_link", Float64, self.setLowerLink) # Sub to lower link length
        self._upLinkParamSub = rospy.Subscriber("/Delta_base/upper_link", Float64, self.setUpperLink) # Sub to upper link length
        self.num_param = 4 # Number of params to load when the model is set up in V-REP
        self.updateCount = 0 # Keeps track of how many params are loaded

    def loadIK(self):
        if self.updateCount == self.num_param:
            rospy.set_param('model_param',self.modelParams)
            # rospy.set_param('model_update', self.modelUpdate)
            rospy.logerr("%d parameters loaded!", self.updateCount)
            self.updateCount = 0
            rospy.logerr(getModelParams())
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

def getModelParams():
    if rospy.has_param('model_param'):
        return rospy.get_param('model_param')
    else:
        rospy.logerr("Not all model parameters are loaded!")

def main():
    rospy.init_node("modelParamListener")
    listener = modelParams()
    rospy.spin()

if __name__=="__main__":
    main()