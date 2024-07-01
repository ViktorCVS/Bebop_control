#!/usr/bin/env python

import rospy

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

        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose_callback)
        rospy.Subscriber('/bebop/cmd_vel', Twist, self.get_cmd_vel_callback)

        self.GE = None
        self.uE = None

        self.limite_dados = 2050

        self.duration = 750
        self.freq = 440

        self.s = time.time()

    def get_pose_callback(self, msg):

        self.msg = msg

    def get_cmd_vel_callback(self, cmsg):

        g0 = self.msg.ddx*cos(self.msg.yaw) + self.msg.ddy*sin(self.msg.yaw)
        g1 = self.msg.dx*cos(self.msg.yaw) + self.msg.dy*sin(self.msg.yaw)
        g2 = -self.msg.ddx*sin(self.msg.yaw) + self.msg.ddy*cos(self.msg.yaw)
        g3 = -self.msg.dx*sin(self.msg.yaw) + self.msg.dy*cos(self.msg.yaw)

        u = np.matrix([[cmsg.linear.x], [cmsg.linear.y], [cmsg.linear.z], [cmsg.angular.z]])

        G = np.matrix([[g0,g1,0,0,0,0,0,0],[0,0,g2,g3,0,0,0,0],[0,0,0,0,self.msg.ddz,self.msg.dz,0,0],[0,0,0,0,0,0,self.msg.ddyaw,self.msg.dyaw]])

        if self.GE is None:

            self.GE = G
            self.uE = u
            
        else:

            self.GE = np.vstack([self.GE, G])
            self.uE = np.vstack([self.uE, u])

        

        GE_1 = np.transpose(self.GE)*self.GE
        try:
            GE_2 = inv(GE_1)
        except np.linalg.LinAlgError:
            return
        GE_3 = GE_2*np.transpose(self.GE)
        T = GE_3*self.uE

        
        print()
        print(T)
        print(len(self.GE)/4)
        print(T[6,0],' ',T[7,0])
        
        
        experimento = 10

        if len(self.GE)/4 == self.limite_dados+1:

            print(time.time()-self.s)

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados7_{experimento}.txt', 'a') as f:
                f.write(str(T[6,0]))

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados8_{experimento}.txt', 'a') as f:
                f.write(str(T[7,0]))

            rospy.signal_shutdown("Code sucessfuly executed.")

        else:

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados7_{experimento}.txt', 'a') as f:
                f.write(str(T[6,0])+',')

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados8_{experimento}.txt', 'a') as f:
                f.write(str(T[7,0])+',')


if __name__ == '__main__':
    try:
        Model_Estimation = ModelEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass