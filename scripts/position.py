#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion
import math
import time
from Robot_Control import RobotControl
from my_docking.srv import docking
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal,DockingFeedback
class DockingServer:

    def __init__(self):
        self._as = rospy.Service('Docking_servers',docking , handler=self.on_goal)
        self.rc = RobotControl()
        self.rate = rospy.Rate(10)    
        self.result = DockingResult()
        self.feedback = DockingFeedback()

    def on_goal(self, goal):
        rospy.loginfo("A goal has been received!")
        rospy.loginfo(goal)
        parking_mission = False
        self.fiducial_id = goal.fiducial_id
        self.feedback = 'Get fiducia_id'
        while parking_mission == False:
            self.feedback = 'Start_Docking'
            distance = self.rc.get_laser(0)
            if distance >= 1.1:
                tolerance_x = 0.1
                self.result.result = 'Pitch_Correction'
                self.rc.pitch_correction(self.fiducial_id)
                position_check = self.rc.check_accuracy_of_position(self.fiducial_id , tolerance_x)
                print('Position check in range distance > 1.1m ', position_check)
                if position_check == False:
                    self.rc.position_correction(self.fiducial_id)
                    self.result.result = 'Centering'
                else:
                    self.rc.parking_check_1_point(self.fiducial_id)

            if distance < 1.1:
                tolerance_x = 0.02
                self.rc.pitch_correction(self.fiducial_id)
                self.result.result = 'Checking'      
                position_check = self.rc.check_accuracy_of_position(self.fiducial_id , tolerance_x)
                print('Position check in range distance < 1.1m ', position_check)
                if position_check == False:
                    self.rc.position_correction(self.fiducial_id)
                else:
                    self.rc.pitch_correction(self.fiducial_id)
                    print('Parking is starting!!!')
                    self.result.result = 'Parking'
                    self.rc.parking_forward(self.fiducial_id)
                    print('AMR has been succesfully docked!!!')
                    self.result.result = 'Parked Succesfully'
                    
                    parking_mission = True
        return self.result.result
        
if __name__ == '__main__':
    rospy.init_node('Docking_servers')

    server = DockingServer()

    rospy.spin()