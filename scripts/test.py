#!/usr/bin/env python3
import rospy
import actionlib
from docking_msgs.msg import DockingAction,DockingActionFeedback,DockingActionGoal,DockingResult,DockingGoal,DockingFeedback
import tf
from geometry_msgs.msg import Twist ,Transform ,Quaternion
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
import math
import time
from my_docking.srv import docking
from sensor_msgs.msg import LaserScan

class DockingServer:

    def __init__(self):
        self._as = rospy.Service('Docking_server',docking , handler=self.on_goal)
        
        #self.tf = tf.TransformListener()
        #self.rate = rospy.Rate(20.0) 
        #self.tf.waitForTransform('base_link','map', rospy.Time(), rospy.Duration(10))
        self.neo_subscriber_scan_1 = rospy.Subscriber("/scan_2", LaserScan, self.scan_callback)
        self.neo_subscriber_fiducial = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray,self.aruco_detect_cb)
        rospy.loginfo("Simple Action Server for Docking has been started!")
    def scan_callback(self, msg):
        self.distance_y = (msg.ranges[0])
    
    def aruco_detect_cb(self, fid_tf_array):
        fid_tf = fid_tf_array.transforms[0]
        self.dock_aruco_tf = fid_tf

    def fid2pos(self, fid_tf):
        q_now = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        euler_angles = euler_from_quaternion(q_now)
        self.x_trans = fid_tf.transform.translation.x
        self.z_trans = fid_tf.transform.translation.z
        
        #if abs(theta)<APPROACH_ANGLE:
        #rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        #rospy.loginfo("Theta: %3.3f, r: %3.3f, x_trans: %3.3f, z_trans: %3.3f, x: %3.3f, y: %3.3f, z: %3.3f", theta, r, x_trans, z_trans, euler_angles[0], euler_angles[1], euler_angles[2])
        #rospy.loginfo("Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds)
        
    

    def tf_callback(self):
        listener = tf.TransformListener()
        self.rate = rospy.Rate(20.0) 

        listener.waitForTransform('base_link','map', rospy.Time(), rospy.Duration(10))
        #listener.waitForTransform(self.fiducial_id,'base_link', rospy.Time(), rospy.Duration(10))
        #listener.waitForTransform(self.fiducial_id,'zed2_left_camera_optical_frame', rospy.Time(), rospy.Duration(10))
        self.neo_velocity = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        rospy.loginfo("Aruco marker has been found!")
        i = 0
        try:
            (self.initial_translation_map_to_baselink, self.initial_quatern_map_to_baselink) = listener.lookupTransform('base_link','map', rospy.Time(0))
            #(self.initial_translation_baselink_to_aruco, self.initial_quatern_baselink_to_aruco) = listener.lookupTransform(self.fiducial_id, 'base_link', rospy.Time(0))
            #(self.initial_translation_camera_to_aruco, self.initial_quatern_camera_to_aruco) = listener.lookupTransform(self.fiducial_id, 'zed2_left_camera_optical_frame', rospy.Time(0))         
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.WARN("Cannot find TF!!")
        
    def on_goal(self, goal):
        rospy.loginfo("A goal has been received!")
        rospy.loginfo(goal)
        result = DockingResult()
        
        while True:
            print(self.dock_aruco_tf)
          


if __name__ == '__main__':
    rospy.init_node('Docking_server')

    server = DockingServer()

    rospy.spin()