#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from custom_msgs.msg import CustomOdometry

from time import time
import os
import signal
from math import sin, cos, pi
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BebopOdometryToAdjusted:
    def __init__(self):

        rospy.init_node('bebop_odometry_to_adjusted', anonymous=True)

        rospy.Subscriber('/natnet_ros/Bebop2/pose', PoseStamped, self.get_pose_callback)
        
        self.adjusted_odometry_pub = rospy.Publisher('/adjusted_odometry', CustomOdometry, queue_size=200)
        
        self.adjusted_odometry_msg = CustomOdometry()
    
    def get_pose_callback(self, msg):

        Position     = msg.pose.position
        Orientation  = msg.pose.orientation

        Position = [Position.x,Position.y, Position.z]
        Orientation_quaternion = [Orientation.x, Orientation.y, Orientation.z, Orientation.w]
        Orientation_euler = list(euler_from_quaternion(Orientation_quaternion))
        Orientation_euler_adjusted = [Orientation_euler[0],Orientation_euler[1],Orientation_euler[2]]

        self.adjusted_odometry_msg.x = Position[0]
        self.adjusted_odometry_msg.y = Position[1]
        self.adjusted_odometry_msg.z = Position[2]
        

        self.adjusted_odometry_msg.roll  = Orientation_euler_adjusted[0]
        self.adjusted_odometry_msg.pitch = Orientation_euler_adjusted[1]
        self.adjusted_odometry_msg.yaw   = Orientation_euler_adjusted[2]


        self.adjusted_odometry_pub.publish(self.adjusted_odometry_msg)
        

if __name__ == '__main__':
    try:
        Bebop_Odometry_To_Adjusted = BebopOdometryToAdjusted()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass