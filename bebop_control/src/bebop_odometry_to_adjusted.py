#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from custom_msgs.msg import CustomOdometry

from time import time
import os
import signal
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BebopOdometryToAdjusted:
    def __init__(self):

        rospy.init_node('bebop_odometry_to_adjusted', anonymous=True)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_pose_callback)
        self.adjusted_odometry_pub = rospy.Publisher('/adjusted_odometry', CustomOdometry, queue_size=100)

        self.initial_pose = [1.6, 1.2, -pi/2]
        #self.initial_pose = [6.0, 6.0, pi/4]

        self.acc = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        self.diff_time = [0,0]
        
        self.adjusted_odometry_msg = CustomOdometry()
        
    def get_pose_callback(self, msg):

        self.diff_time[1] = time()

        Position     = msg.pose[-1].position
        Orientation  = msg.pose[-1].orientation
        Vel_linear   = msg.twist[-1].linear
        Vel_angular  = msg.twist[-1].angular

        Position = [(Position.y+self.initial_pose[1]),-(Position.x+self.initial_pose[0]), Position.z]
        Orientation_quaternion = [Orientation.x, Orientation.y, Orientation.z, Orientation.w]
        Orientation_euler = list(euler_from_quaternion(Orientation_quaternion))
        Orientation_euler_adjusted = [Orientation_euler[0],Orientation_euler[1],Orientation_euler[2]+self.initial_pose[2]]

        self.acc[0][1] = Vel_linear.x
        self.acc[1][1] = Vel_linear.y
        self.acc[2][1] = Vel_linear.z
        self.acc[3][1] = Vel_angular.x
        self.acc[4][1] = Vel_angular.y
        self.acc[5][1] = Vel_angular.z

        self.adjusted_odometry_msg.x = Position[0]
        self.adjusted_odometry_msg.y = Position[1]
        self.adjusted_odometry_msg.z = Position[2]

        self.adjusted_odometry_msg.roll  = Orientation_euler_adjusted[0]
        self.adjusted_odometry_msg.pitch = Orientation_euler_adjusted[1]
        self.adjusted_odometry_msg.yaw   = Orientation_euler_adjusted[2]

        self.adjusted_odometry_msg.dx = Vel_linear.y
        self.adjusted_odometry_msg.dy = -Vel_linear.x
        self.adjusted_odometry_msg.dz = Vel_linear.z
        self.adjusted_odometry_msg.droll = Vel_angular.y
        self.adjusted_odometry_msg.dpitch = Vel_angular.x
        self.adjusted_odometry_msg.dyaw = Vel_angular.z

        self.adjusted_odometry_msg.ddx = (self.acc[1][1] - self.acc[1][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddy = -(self.acc[0][1] - self.acc[0][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddz = (self.acc[2][1] - self.acc[2][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddroll = (self.acc[3][1] - self.acc[3][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddpitch = (self.acc[4][1] - self.acc[4][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddyaw = (self.acc[5][1] - self.acc[5][0])/(self.diff_time[1] - self.diff_time[0])

        self.acc[0][0] = self.acc[0][1]
        self.acc[1][0] = self.acc[1][1]
        self.acc[2][0] = self.acc[2][1]
        self.acc[3][0] = self.acc[3][1]
        self.acc[4][0] = self.acc[4][1]
        self.acc[5][0] = self.acc[5][1]

        self.diff_time[0] = time()

        self.adjusted_odometry_pub.publish(self.adjusted_odometry_msg)
        

if __name__ == '__main__':
    try:
        Bebop_Odometry_To_Adjusted = BebopOdometryToAdjusted()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass