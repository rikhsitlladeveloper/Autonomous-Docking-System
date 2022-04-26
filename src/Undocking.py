#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from rospy.core import rospyerr, rospywarn
import tf
from tf.transformations import euler_from_quaternion
import math
import time
from Robot_Control import RobotControl
from my_docking.srv import Undocking
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal,DockingFeedback
class UndockingServer:

    def __init__(self):
        self._as = rospy.Service('Docking_servers',Undocking , handler=self.on_goal)
        self.rc = RobotControl()
        self.rate = rospy.Rate(10)
        self.result = DockingResult()    
 
    
    def on_goal(self, goal):
        rospy.loginfo("A goal has been received!")
        rospy.loginfo(goal)
        self.result.result = 'Done'
        return self.result.result
if __name__ == '__main__':
    rospy.init_node('Docking_servers')

    server = UndockingServer()

    rospy.spin()