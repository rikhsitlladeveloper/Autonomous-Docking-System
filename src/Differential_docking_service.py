#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from rospy.core import rospyerr, rospywarn
import tf
from tf.transformations import euler_from_quaternion
import math
import time
from Robot_Control import RobotControl
from my_docking.srv import Docking_call
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal,DockingFeedback
class DockingServer:

    def __init__(self):
        self._as = rospy.Service('Docking_servers',Docking_call , handler=self.on_goal)
        self.rc = RobotControl()
        self.rate = rospy.Rate(10)    
        self.result = DockingResult()

    def on_goal(self, goal):
        self.fiducial_id = goal.fiducial_id
        if(self.fiducial_id == "undock"):
            print("Undocking is starting")
            self.rc.undock()
            self.result.result = "Undocked"
        else:    
            rospy.loginfo("A goal has been received!")
            rospy.loginfo(goal)
            parking_mission = False
            self.feedback = 'Get fiducia_id'
            while parking_mission == False:
                self.feedback = 'Start_Docking'
                distance = self.rc.get_laser(541)
                if self.rc.obstacle_avoidance() < 0.55:
                    self.rc.stop_robot()
                    print('Obstacle_detected')
                    rospywarn("OBSTACLE_DETECTED")    
                elif distance >= 1.1:
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

                elif distance < 1.1:
                    tolerance_x = rospy.get_param('tolerance_x_for_centering')
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
                        self.result.result = 'AMR has been succesfully docked!!!'
                        parking_mission = True
        
        return self.result.result
        
if __name__ == '__main__':
    rospy.init_node('Docking_servers')

    server = DockingServer()

    rospy.spin()
