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


class ModelEstimation:
    def __init__(self):

        rospy.init_node('model_estimation', anonymous=True)

        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose_callback)
        rospy.Subscriber('/bebop/cmd_vel', Twist, self.get_cmd_vel_callback)

        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        rospy.loginfo("Iniciando Estimação.")

        self.Axis = rospy.get_param('Axis')

        self.GE = None
        self.uE = None

        self.G1E = None
        self.u1E = None
        self.G2E = None
        self.u2E = None
        self.G3E = None
        self.u3E = None
        self.G4E = None
        self.u4E = None

        self.limite_dados = 2050

        self.en_x   = False
        self.en_y   = False
        self.en_z   = False
        self.en_yaw = False

        if(self.Axis=='x'):
            self.en_x    = True
        elif(self.Axis=='y'):
            self.en_y    = True
        elif(self.Axis=='z'):
             self.en_z   = True
        elif(self.Axis=='yaw'):
            self.en_yaw  = True

        if(self.en_x):
            self.pos_T = 0 
        elif(self.en_y):
            self.pos_T = 2
        elif(self.en_z):
            self.pos_T = 4
        else:
            self.pos_T = 6

    def land(self):
        self.land_pub.publish(Empty())

    def get_pose_callback(self, msg):

        self.msg = msg


    def get_cmd_vel_callback(self, cmsg):

        g0 = self.msg.ddx*cos(self.msg.yaw) + self.msg.ddy*sin(self.msg.yaw)
        g1 = self.msg.dx*cos(self.msg.yaw) + self.msg.dy*sin(self.msg.yaw)
        g2 = -self.msg.ddx*sin(self.msg.yaw) + self.msg.ddy*cos(self.msg.yaw)
        g3 = -self.msg.dx*sin(self.msg.yaw) + self.msg.dy*cos(self.msg.yaw)

        #u = np.matrix([[cmsg.linear.x], [cmsg.linear.y], [cmsg.linear.z], [cmsg.angular.z]])

        #G = np.matrix([[g0,g1,0,0,0,0,0,0],[0,0,g2,g3,0,0,0,0],[0,0,0,0,self.msg.ddz,self.msg.dz,0,0],[0,0,0,0,0,0,self.msg.ddyaw,self.msg.dyaw]])

        G1 = np.matrix([[g0,g1,0,0,0,0,0,0]])
        G2 = np.matrix([[0,0,g2,g3,0,0,0,0]])
        G3 = np.matrix([[0,0,0,0,self.msg.ddz,self.msg.dz,0,0]])
        G4 = np.matrix([[0,0,0,0,0,0,self.msg.ddyaw,self.msg.dyaw]])

        u1 = np.matrix([[cmsg.linear.x]])
        u2 = np.matrix([[cmsg.linear.y]])
        u3 = np.matrix([[cmsg.linear.z]])
        u4 = np.matrix([[cmsg.angular.z]])

        if self.GE is None:

            self.G1E = G1
            self.u1E = u1
            self.G2E = G2
            self.u2E = u2
            self.G3E = G3
            self.u3E = u3
            self.G4E = G4
            self.u4E = u4

            
        else:

            self.G1E = np.vstack([self.G1E, G1])
            self.u1E = np.vstack([self.u1E, u1])
            self.G2E = np.vstack([self.G2E, G2])
            self.u2E = np.vstack([self.u2E, u2])
            self.G3E = np.vstack([self.G3E, G3])
            self.u3E = np.vstack([self.u3E, u3])
            self.G4E = np.vstack([self.G4E, G4])
            self.u4E = np.vstack([self.u4E, u4])

        self.GE = np.vstack([self.G1E, self.G2E, self.G3E, self.G4E])
        self.uE = np.vstack([self.u1E, self.u2E, self.u3E, self.u4E])

        GE_1 = np.transpose(self.GE)*self.GE
        try:
            GE_2 = inv(GE_1)
        except np.linalg.LinAlgError:
            return
        GE_3 = GE_2*np.transpose(self.GE)
        T = GE_3*self.uE

        
        print(len(self.GE)/4)
        print(1/T[self.pos_T,0],' ',T[self.pos_T+1,0]/T[self.pos_T,0])
        
        
        experimento = 1

        if len(self.GE)/4 == self.limite_dados+1:

            # with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados_{self.Axis}_{self.pos_T}_{experimento}.txt', 'a') as f:
            #     f.write(str(T[self.pos_T,0]))

            # with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados_{self.Axis}_{self.pos_T+1}_{experimento}.txt', 'a') as f:
            #     f.write(str(T[self.pos_T+1,0]))

            # self.land()
            # rospy.signal_shutdown("Code sucessfuly executed.")

            pass

        else:

            # with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados_{self.Axis}_{self.pos_T}_{experimento}.txt', 'a') as f:
            #     f.write(str(T[self.pos_T,0])+',')

            # with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados_{self.Axis}_{self.pos_T+1}_{experimento}.txt', 'a') as f:
            #     f.write(str(T[self.pos_T+1,0])+',')

            pass


if __name__ == '__main__':
    try:
        Model_Estimation = ModelEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass