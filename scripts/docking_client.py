#!/usr/bin/env python3

from os import wait
import rospy
import actionlib
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal
from rospy import client
class DockingClient:
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/docking', DockingAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send goal!")
    def send_goal_and_get_result(self):
        goal = DockingGoal(fiducial_id = 'fiducial_1' )
        self._ac.send_goal(goal, done_cb= self.done_callback, feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal has been send")
        self._ac.wait_for_result()
        rospy.loginfo(self._ac.get_result())

    def done_callback(self, status, result):
        rospy.loginfo('Status is :' + str(status))
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)
    
if __name__ == '__main__':
    rospy.init_node('Docking_client')

    client = DockingClient()
    
    client.send_goal_and_get_result()
    #rospy.spin()