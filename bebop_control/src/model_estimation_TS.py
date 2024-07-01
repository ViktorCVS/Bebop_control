#!/usr/bin/env python

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from custom_msgs.msg import CustomOdometry

import time
import os
import signal
import numpy as np
from numpy.linalg import inv
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#0,0872665

class ModelEstimation:
    def __init__(self):

        rospy.init_node('model_estimation', anonymous=True)

        odometry_data = Subscriber('/adjusted_odometry', CustomOdometry)
        cmd_vel_data = Subscriber('/bebop/cmd_vel', Twist)

        ts = ApproximateTimeSynchronizer([odometry_data, cmd_vel_data], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.get_data_callback)

        self.GE = None

        print("Estou funcionando!")
                
    def get_data_callback(self, msg, cmsg):

        print("Loop funcionando!")

        g0 = msg.ddx*cos(msg.yaw) + msg.ddy*sin(msg.yaw)
        g1 = msg.dx*cos(msg.yaw) + msg.dy*sin(msg.yaw)
        g2 = -msg.ddx*sin(msg.yaw) + msg.ddy*cos(msg.yaw)
        g3 = -msg.dx*sin(msg.yaw) + msg.dy*cos(msg.yaw)

        G = np.matrix([g0,g1,0,0,0,0,0,0],[g2,g3,0,0,0,0,0,0],[0,0,0,0,msg.ddz,msg.dz,0,0],[0,0,0,0,0,0,msg.ddyaw,msg.dyaw])

        if GE == None:

            GE = G
            
        else:

            GE = np.vstack(GE, G)

        T = inv(np.transpose(GE)*GE)*np.transpose(GE)*[cmsg.linear.x, cmsg.linear.y, cmsg.linear.z, cmsg.angular.z]

        print(T)

if __name__ == '__main__':
    try:
        Model_Estimation = ModelEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass