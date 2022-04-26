#!/usr/bin/env python

from re import S
from turtle import backward

from defer import return_value
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from rospy.client import get_param, has_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import radians, copysign, sqrt, pow, pi , degrees
import PyKDL


class RobotControl():

    def __init__(self):
        #rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.summit_vel_publisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber('/scan_1', LaserScan, self.summit_laser_callback)
        self.odom_sub = rospy.Subscriber('/zed2/zed_node/odom', Odometry, self.odom_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.base_frame = '/base_link'
        self.angular_tolerance = radians(1)
        self.linear_tolerance = rospy.get_param('linear_tolerance')
        self.pitch_tolerance = rospy.get_param('pitch_tolerance')
        self.tolerance_x = rospy.get_param('tolerance_x_for_centering')
        rospy.on_shutdown(self.shutdownhook)
        self.parking_stop_distance = rospy.get_param('parking_stop_distance')
        self.check_1_point_stop_distance = rospy.get_param('check_1_point_stop_distance')
        self.obstacle_safety_distance = rospy.get_param('safety_distance')
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            summit_connections = self.summit_vel_publisher.get_num_connections()
            if connections > 0 or summit_connections > 0:
                self.vel_publisher.publish(self.cmd)
                self.summit_vel_publisher.publish(self.cmd)
                # rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
       
        self.laser_msg = msg

        

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def odom_callback(self, msg):
        self.position_q = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        return self.position_q , self.roll, self.pitch, self.yaw

    def get_laser(self, pos):
        time.sleep(0.1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_back_laser(self):
        time.sleep(0.4)
        return self.laser_msg.ranges[0]
    
    def obstacle_avoidance(self):
        #time.sleep(0.01)
        self.regions = {
            'full':  min(min(self.laser_msg.ranges[0:1083]), 10),
        }
        return self.regions['full']
    
    def get_laser_full(self):
        time.sleep(0.005)
        return self.laser_msg.ranges
    
    def parking_forward(self, fiducial_id):
        "*** STARTING FORWARD DOCKING UNTILL CHARGING STATION WITHOUT OBSTACLE AVOIDANCE ***"
        while self.get_laser(541) > self.parking_stop_distance:
            trans_cam ,rotation_cam = self.get_aruco_to_robot_angle(fiducial_id)
            if abs(rotation_cam) > 0.5:
                self.cmd.angular.z = 2 * (-radians(rotation_cam))
            else:
                self.cmd.angular.z = 0
            self.cmd.linear.x = 0.3
            self.publish_once_in_cmd_vel()
        self.stop_robot()
    
    def parking_check_1_point(self, fiducial_id):
        while self.get_laser(541) > self.check_1_point_stop_distance:
            trans_cam ,rotation_cam = self.get_aruco_to_robot_angle(fiducial_id)
            if self.obstacle_avoidance() < self.obstacle_safety_distance :
                self.stop_robot()
            elif abs(rotation_cam) > 0.5:
                self.cmd.angular.z = -radians(rotation_cam)
                self.cmd.linear.x = 0.3
            else:
                self.cmd.angular.z = 0
                self.cmd.linear.x = 0.3
            self.publish_once_in_cmd_vel()
        self.stop_robot()
    
    def check_accuracy_of_position(self, fiducial_id, tolerance):
        sum_of_x = 0
        sum_of_angle = 0
        for x in range(0,100):
            trans_cam ,rotation_cam = self.get_aruco_to_robot_angle(fiducial_id)
            trans_base , rotation_base = self.get_aruco_pos(fiducial_id)
            sum_of_x = trans_base.x + sum_of_x
            sum_of_angle = rotation_cam + sum_of_angle
            time.sleep(0.01)  
        distance_x_avg = sum_of_x / 100
        angle_avg = sum_of_angle / 100
        if abs(distance_x_avg) < tolerance:
            checking = True
        else:
            checking = False     
        return checking
    
    def get_avg_of_aruco_pos(self, fiducial_id): 
        sum_of_x = 0
        sum_of_angle = 0
        for x in range(0,100):
            trans_cam ,rotation_cam = self.get_aruco_to_robot_angle(fiducial_id)
            trans_base , rotation_base = self.get_aruco_pos(fiducial_id)
            sum_of_x = trans_base.x + sum_of_x
            sum_of_angle = rotation_cam + sum_of_angle
            time.sleep(0.01)
        distance_x_avg = sum_of_x / 100
        angle_avg = sum_of_angle / 100
        return distance_x_avg , angle_avg
    
    def stop_robot(self):
        # rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    
    def undock(self):
        self.move_straight_time("backward", 0.4, 2)
        self.turn("clockwise",2,3.1)
        print("Undocking has been done !")
    
    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s

    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)

        while (i <= time):
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s

    def get_odom(self):

        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def position_correction(self, fiducial_id):
        distance_x_avg , angle_avg = self.get_avg_of_aruco_pos(fiducial_id)
        print (distance_x_avg , angle_avg)
        if abs(distance_x_avg) > self.tolerance_x and distance_x_avg < 0:
            self.rotate(-angle_avg - 90)
            self.move(abs(distance_x_avg))
            self.rotate(90)
            print('Robot has been Centered!!!')
        
        elif abs(distance_x_avg) > self.tolerance_x and distance_x_avg > 0:
            self.rotate(90 - angle_avg)
            self.move(abs(distance_x_avg))
            self.rotate(-90)
            print('Robot has been Centered!!!')
        
        else:
            print('Robot has been Centered already!!!')

    def pitch_correction(self, fiducial_id):
        pitch_correction_mission = False
        while pitch_correction_mission == False:
            trans_cam ,rotation_cam = self.get_aruco_to_robot_angle(fiducial_id)
            if self.obstacle_avoidance() < self.obstacle_safety_distance:
                print('Obstacle_detected')
                self.stop_robot()
            elif abs(rotation_cam) > self.pitch_tolerance:
                self.cmd.angular.z = -radians(rotation_cam)
            else:
                self.cmd.angular.z = 0
                self.vel_publisher.publish(self.cmd)
                pitch_correction_mission = True                
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()
        self.stop_robot()
        print("Pitch correcction has been done!") 

    def get_aruco_pos(self,fiducial_id):
        """ Getting position of aruco relative to base_link for Position  (linear: x y z ,rotational: x y z w"""
        """ Get fiducials_id and send position and yaw between aruco and base_link """
        
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(fiducial_id, '/base_link', rospy.Time(), rospy.Duration(5.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot) = self.tf_listener.lookupTransform(fiducial_id, '/base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def get_aruco_to_robot_angle(self,fiducial_id):
        "Getting position of aruco relative to camera Zed2 for Pitch  (linear: x y z ,rotational: x y z w"
        "Get fiducials_id and send position and pitch between aruco and zed2 camera"
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(fiducial_id, '/zed2_left_camera_optical_frame', rospy.Time(), rospy.Duration(5.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot) = self.tf_listener.lookupTransform('/zed2_left_camera_optical_frame', fiducial_id, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_degree(Quaternion(*rot)))


    def move(self, distance):
        position = Point()

        # Get the current position
        (position, rotation) = self.get_odom()

        # Set the movement command to a forward or backward
        if distance > 0:
            self.cmd.linear.x = 0.2
        else:
            self.cmd.linear.x = -0.2
        # Track the last point measured
        last_point_x = position.x
        last_point_y = position.y

        # Track how far we have turned
        move_distance = 0

        goal_distance = distance
        linear = self.cmd.linear.x
        # Begin the motion
        while abs(move_distance + self.linear_tolerance) < abs(goal_distance) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            if self.obstacle_avoidance() < self.obstacle_safety_distance:
                print('Obstacle_detected')
                self.stop_robot()
            else:
                self.cmd.linear.x = linear
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_distance = sqrt((position.x - last_point_x) ** 2 + (position.y - last_point_y) ** 2)

            move_distance += delta_distance
            last_point_x = position.x
            last_point_y = position.y

        self.stop_robot()





    def rotate(self, degrees):

        position = Point()

        # Get the current position
        (position, rotation) = self.get_odom()

        # Set the movement command to a rotation
        if degrees > 0:
            self.cmd.angular.z = 0.6
        else:
            self.cmd.angular.z = -0.6

        # Track the last angle measured
        last_angle = rotation

        # Track how far we have turned
        turn_angle = 0

        goal_angle = radians(degrees)
        angular = self.cmd.angular.z
        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            
            # Publish the Twist message and sleep 1 cycle
            if self.obstacle_avoidance() < self.obstacle_safety_distance:
                print('Obstacle_detected')
                self.stop_robot()
            else:
                self.cmd.angular.z = angular            
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()
           
            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation

        self.stop_robot()

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def quat_to_degree(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return degrees(rot.GetRPY()[1])

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    # rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass