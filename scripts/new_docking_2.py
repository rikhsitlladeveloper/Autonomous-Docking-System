#!/usr/bin/env python3

import rospy
import actionlib
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal,DockingFeedback
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist ,Transform,TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
import math
import time
from my_docking.srv import docking
from sensor_msgs.msg import LaserScan

class DockingServer:

    def __init__(self):
        self._as = rospy.Service('Docking_server',docking , handler=self.on_goal)


        self.neo_subscriber_scan_1 = rospy.Subscriber("/scan_1", LaserScan, self.scan_callback)
        #self.neo_subscriber_fiducials = rospy.Subscriber("/tf",TFMessage, self.tf_callback)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(20.0)
        self.centering = False
        rospy.loginfo("Simple Action Server for Docking has been started!")
    def scan_callback(self, msg):
        self.distance_y = (msg.ranges[0])

    def print_distance(self,distance_x):
        while True:
            print(distance_x)

    def on_goal(self, goal):
        rospy.loginfo("A goal has been received!")
        rospy.loginfo(goal)
        result = DockingResult()

        self.fiducial_id = goal.fiducial_id


        self.listener.waitForTransform('base_link','map', rospy.Time(), rospy.Duration(10))
        self.listener.waitForTransform(self.fiducial_id,'base_link', rospy.Time(), rospy.Duration(10))
        self.listener.waitForTransform(self.fiducial_id,'zed2_left_camera_optical_frame', rospy.Time(), rospy.Duration(10))
        self.neo_velocity = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        rospy.loginfo("Aruco marker has been found!")
        self.mission = False
        summ_of_x_from_aruco = 0
        summ_of_y_from_aruco = 0
        summ_of_pitch_from_aruco = 0
        i = 0
        result.result = 'Process'
        # Getting initial pose relative to map
        for x in range (0,100):
            try:
                (self.initial_translation_map_to_baselink, self.initial_quatern_map_to_baselink) = self.listener.lookupTransform('base_link','map', rospy.Time(0))
                (self.initial_translation_baselink_to_aruco, self.initial_quatern_baselink_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'base_link', rospy.Time(0))
                (self.initial_translation_camera_to_aruco, self.initial_quatern_camera_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'zed2_left_camera_optical_frame', rospy.Time(0))         
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.WARN("Cannot find TF!!")
                result.result = 'Failed'

            self.initial_distance_y_from_map_to_baselink = self.initial_translation_map_to_baselink[0]
            self.initial_distance_x_from_map_to_baselink = self.initial_translation_map_to_baselink[1]
            self.initial_roll_from_map_to_baselink ,self.initial_pitch_from_map_to_baselink, self.initial_yaw_from_map_to_baselink = self.degree_from_radian(euler_from_quaternion(self.initial_quatern_map_to_baselink))
            summ_of_x_from_aruco = summ_of_x_from_aruco + self.initial_translation_baselink_to_aruco[0]
            summ_of_y_from_aruco = summ_of_y_from_aruco +self.initial_translation_baselink_to_aruco[2]
            self.initial_euler_camera_to_aruco = euler_from_quaternion(self.initial_quatern_camera_to_aruco)
            self.initial_roll_baselink_to_aruco , self.initial_pitch_baselink_to_aruco, self.initial_yaw_baselink_to_aruco  = self.degree_from_radian(self.initial_euler_camera_to_aruco) 
            summ_of_pitch_from_aruco = summ_of_pitch_from_aruco - self.initial_pitch_baselink_to_aruco
            i = i + 1
            self.rate.sleep()
        self.initial_avg_distance_x_from_base_link_to_aruco = summ_of_x_from_aruco / 100
        self.initial_avg_distance_y_from_base_link_to_aruco = summ_of_y_from_aruco / 100
        self.initial_avg_pitch_baselink_to_aruco = summ_of_pitch_from_aruco / 100
        print("x:", self.initial_avg_distance_x_from_base_link_to_aruco , "y:", self.initial_avg_distance_y_from_base_link_to_aruco , "pitch:", self.initial_avg_pitch_baselink_to_aruco)
        while self.centering == False:
            try:
                (self.translation_map_to_baselink, self.quatern_map_to_baselink) = self.listener.lookupTransform('base_link','map', rospy.Time(0))
                (self.translation_baselink_to_aruco, self.quatern_baselink_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'base_link', rospy.Time(0))
                (self.translation_camera_to_aruco, self.quatern_camera_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'zed2_left_camera_optical_frame', rospy.Time(0))
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.WARN("Cannot find TF!!")
            feedback = DockingFeedback()

            self.euler_map_to_baselink = euler_from_quaternion(self.quatern_map_to_baselink)
            self.euler_camera_to_aruco = euler_from_quaternion(self.quatern_camera_to_aruco)
            self.roll_map_to_baselink , self.pitch_map_to_baselink, self.yaw_map_to_baselink  = self.degree_from_radian(self.euler_map_to_baselink)
            self.roll_baselink_to_aruco , self.pitch_baselink_to_aruco, self.yaw_baselink_to_aruco  = self.degree_from_radian(self.euler_camera_to_aruco)
            # parameters from base_link to aruco
            self.distance_x_from_baselink_to_aruco = self.translation_baselink_to_aruco[0]  # right +
            self.distance_y_from_baselink_to_aruco = self.translation_baselink_to_aruco[2]   # y min from aruco to base_link =0.7 m
            self.angle_from_baselink_to_aruco = math.degrees(math.atan(self.distance_x_from_baselink_to_aruco / self.distance_y_from_baselink_to_aruco)) # Angle Alfa between base_link to aruco in degrees - if base_link in left + if in right
            self.pitch_baselink_to_aruco = -self.pitch_baselink_to_aruco #pitch from camera to aruco clockwise + average 3 degree threshold

            # parameters from map to base_link
            self.relative_y_distance_from_map_to_baselink = self.initial_distance_y_from_map_to_baselink - self.translation_map_to_baselink[0]
            self.relative_x_distance_from_map_to_baselink = self.translation_map_to_baselink[1] - self.initial_distance_x_from_map_to_baselink
            self.relative_yaw_from_map_to_baselink = self.yaw_map_to_baselink - self.initial_yaw_from_map_to_baselink # clockwise + in degree
            self.rotation_goal = 90 - self.initial_avg_pitch_baselink_to_aruco
            if abs(self.initial_avg_distance_x_from_base_link_to_aruco) > 0.1:
                self.centering_mission(self.initial_avg_distance_x_from_base_link_to_aruco , self.initial_avg_distance_y_from_base_link_to_aruco ,self.rotation_goal)

    def centering_mission(self,distance_x, distance_y,rotation_goal):
        relative_distance_x = distance_x - self.relative_x_distance_from_map_to_baselink
        relative_angle = rotation_goal - self.relative_yaw_from_map_to_baselink
        relative_distance_y = distance_y - self.relative_y_distance_from_map_to_baselink
        if abs(relative_angle) > 5:
            angular = relative_angle * 3.14 / 180
            linear = 0
        elif abs(relative_distance_x) > 0.1:
            angular = 0
            linear = relative_distance_x
        else:
            print("Mission has been done")
            self.centering = True
        self.cmd_publisher(linear,angular,self.neo_velocity,self.rate)

    def perpendicular_motion(self,distance_x,perpendicular_mission,listener,range):
        self.listener.waitForTransform(self.fiducial_id,'base_link', rospy.Time(), rospy.Duration(10))
        while perpendicular_mission == False:
            (self.translation_baselink_to_aruco, self.quatern_baselink_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'base_link', rospy.Time(0))
            print('perpendicular_motion')
            distance_x = self.translation_baselink_to_aruco[0]
            if abs(distance_x) > range:
                perpendicular = 0.5 * distance_x
                linear = 0
                angular = 0
            else:
                linear = 0
                perpendicular = 0
                angular = 0

                perpendicular_mission = True
            self.cmd_publisher(linear,perpendicular,angular,self.neo_velocity,self.rate)

    def angular_motion(self,angular_mission,listener,range):
        self.listener.waitForTransform(self.fiducial_id,'zed2_left_camera_optical_frame', rospy.Time(), rospy.Duration(10))
        while angular_mission == False:
            (self.translation_camera_to_aruco, self.quatern_camera_to_aruco) = self.listener.lookupTransform(self.fiducial_id, 'zed2_left_camera_optical_frame', rospy.Time(0))
            print('angular_motion')
            self.euler_camera_to_aruco = euler_from_quaternion(self.quatern_camera_to_aruco)
            pitch = self.euler_camera_to_aruco[1] * 180 / math.pi
            if abs(pitch) > range:
                perpendicular = 0
                linear = 0
                angular = -0.1 * pitch * math.pi / 180
            else:
                linear = 0
                perpendicular = 0
                angular = 0

                angular_mission = True
            self.cmd_publisher(linear,perpendicular,angular,self.neo_velocity,self.rate)



    def degree_from_radian(self,euler): #From radians to degree
        r, p ,y = euler
        roll = r * 180 / math.pi
        pitch = p * 180 / math.pi
        yaw = y * 180 / math.pi
        #print('Euler from degree done')
        return roll , pitch , yaw

    def cmd_publisher(self, linear, angular ,neo_velocity,rate):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        neo_velocity.publish(cmd)
        rate.sleep()
if __name__ == '__main__':
    rospy.init_node('Docking_server')

    server = DockingServer()

    rospy.spin()