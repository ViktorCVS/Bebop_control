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
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BebopOdometryToAdjusted:
    def __init__(self):

        rospy.init_node('bebop_odometry_to_adjusted', anonymous=True)

        rospy.Subscriber('/natnet_ros/Bebop2/pose', PoseStamped, self.get_pose_callback)
        self.adjusted_odometry_pub = rospy.Publisher('/adjusted_odometry', CustomOdometry, queue_size=100)

        self.initial_pose = [1.6, 1.2, -pi/2]
        #self.initial_pose = [6.0, 6.0, pi/4]

        self.vel = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        self.acc = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        self.diff_time = [0,0]
        
        self.adjusted_odometry_msg = CustomOdometry()
        
    def get_pose_callback(self, msg):

        self.diff_time[1] = time()

        Position     = msg.pose.position
        Orientation  = msg.pose.orientation

        Position = [Position.x,Position.y, Position.z]
        Orientation_quaternion = [Orientation.x, Orientation.y, Orientation.z, Orientation.w]
        Orientation_euler = list(euler_from_quaternion(Orientation_quaternion))
        Orientation_euler_adjusted = [Orientation_euler[0],Orientation_euler[1],Orientation_euler[2]]

        self.vel[0][1] = Position[0]
        self.vel[1][1] = Position[1]
        self.vel[2][1] = Position[2]
        self.vel[3][1] = Orientation_euler_adjusted[0]
        self.vel[4][1] = Orientation_euler_adjusted[1]
        self.vel[5][1] = Orientation_euler_adjusted[2]


        self.adjusted_odometry_msg.x = Position[0]
        self.adjusted_odometry_msg.y = Position[1]
        self.adjusted_odometry_msg.z = Position[2]

        self.adjusted_odometry_msg.roll  = Orientation_euler_adjusted[0]
        self.adjusted_odometry_msg.pitch = Orientation_euler_adjusted[1]
        self.adjusted_odometry_msg.yaw   = Orientation_euler_adjusted[2]

        self.adjusted_odometry_msg.dx = (self.vel[0][1] - self.vel[0][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.dy = (self.vel[1][1] - self.vel[1][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.dz = (self.vel[2][1] - self.vel[2][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.droll = (self.vel[3][1] - self.vel[3][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.dpitch = (self.vel[4][1] - self.vel[4][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.dyaw =(self.vel[5][1] - self.vel[5][0])/(self.diff_time[1] - self.diff_time[0])

        self.acc[0][1] = self.adjusted_odometry_msg.dx
        self.acc[1][1] = self.adjusted_odometry_msg.dy
        self.acc[2][1] = self.adjusted_odometry_msg.dz
        self.acc[3][1] = self.adjusted_odometry_msg.droll
        self.acc[4][1] = self.adjusted_odometry_msg.dpitch
        self.acc[5][1] = self.adjusted_odometry_msg.dyaw

        self.adjusted_odometry_msg.ddx = (self.acc[1][1] - self.acc[1][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddy = -(self.acc[0][1] - self.acc[0][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddz = (self.acc[2][1] - self.acc[2][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddroll = (self.acc[3][1] - self.acc[3][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddpitch = (self.acc[4][1] - self.acc[4][0])/(self.diff_time[1] - self.diff_time[0])
        self.adjusted_odometry_msg.ddyaw = (self.acc[5][1] - self.acc[5][0])/(self.diff_time[1] - self.diff_time[0])


        self.vel[0][0] = self.vel[0][1]
        self.vel[1][0] = self.vel[1][1]
        self.vel[2][0] = self.vel[2][1]
        self.vel[3][0] = self.vel[3][1]
        self.vel[4][0] = self.vel[4][1]
        self.vel[5][0] = self.vel[5][1]

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