#!/usr/bin/env python3
from Robot_Control import RobotControl

import rospy

rospy.init_node('robot_control_node', anonymous=True)

rc = RobotControl()

print(rc.obstacle_avoidance())