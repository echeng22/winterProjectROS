## What's in this Repo
ROS python code used to control the delta robot model in V-REP.

#### Files of Note
Src folder contains two python files:

+ control.py: Simple ROS publisher/subscriber node that sends JointStates to V-REP to move the active joints. Used to set up and test to make sure ROS communication works between node and V-REP.

+ traj_control.py: ROS trajectory control python script that will have the end-effector of the delta robot move in desired trajectory. Work is in progress for this file and does not currently work as intended.
