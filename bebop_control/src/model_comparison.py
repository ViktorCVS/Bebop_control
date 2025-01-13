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
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        self.k1 = 2920
        self.k2 = -100.1229
        self.k3 = 22816
        self.k4 = -513.8493
        self.k5 = -617.6297
        self.k6 = -478.6105
        self.k7 = -5010.2
        self.k8 = -1666.1

        self.Axis = rospy.get_param("Axis")

        if(self.Axis=='x'):
            self.control = 0
        elif(self.Axis=='y'):
            self.control = 1
        elif(self.Axis=='z'):
            self.control = 2
        elif(self.Axis=='yaw'):
            self.control = 3
        else:
            self.control = 0

        self.limite_dados = 120

        self.s = time.time()

        self.vel = np.matrix([[0],[0],[0],[0]])

        self.control_loop = 0

    def get_pose_callback(self, msg):

        self.msg = msg
        self.vmsg = [msg.dx, msg.dy, msg.dz, msg.dyaw]
        self.amsg = [msg.ddx, msg.ddy, msg.ddz, msg.ddyaw]


    def get_cmd_vel_callback(self, cmsg):

        self.cmd_control = rospy.get_param('Node')

        
        if self.cmd_control != 'open':
            return

        if self.regulation == 'True':
            return

        rospy.loginfo("Iniciando Validação.")
        
        self.F1 = np.matrix([[self.k1*cos(self.msg.yaw),-self.k3*sin(self.msg.yaw),0,0],
                            [self.k1*sin(self.msg.yaw),self.k3*cos(self.msg.yaw),0,0],
                            [0,0,self.k5,0],
                            [0,0,0,self.k7]])

        self.F2 = np.matrix([[self.k2*cos(self.msg.yaw),-self.k4*sin(self.msg.yaw),0,0],
                            [self.k2*sin(self.msg.yaw),self.k4*cos(self.msg.yaw),0,0],
                            [0,0,self.k6,0],
                            [0,0,0,self.k8]])


        u = np.matrix([[cmsg.linear.x], [cmsg.linear.y], [cmsg.linear.z], [cmsg.angular.z]])

        if self.control_loop > 0:
            delta_t = time.time() - self.begin_loop
            self.vel = np.matrix([[self.acc[0,0]*delta_t+self.vel[0,0]],[self.acc[1,0]*delta_t+self.vel[1,0]],[self.acc[2,0]*delta_t+self.vel[2,0]],[self.acc[3,0]*delta_t+self.vel[3,0]]])

        self.begin_loop = time.time()
        self.acc = self.F1*u - self.F2*self.vel

        experimento = 2

        if self.control_loop < self.limite_dados:

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/acc_modelo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.acc[self.control,0])+',')
            
            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/vel_modelo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.vel[self.control,0])+',')

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/acc_sim_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.amsg[self.control])+',')
            
            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/vel_sim_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.vmsg[self.control])+',')

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/tempo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(rospy.get_time())+',')


        else:

            rospy.loginfo("Coleta de dados finalizada.")

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/acc_modelo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.acc[self.control,0]))
            
            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/vel_modelo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.vel[self.control,0]))

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/acc_sim_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.amsg[self.control]))
            
            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/vel_sim_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(self.vmsg[self.control]))

            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/tempo_{self.Axis}_{experimento}.txt', 'a') as f:
                f.write(str(rospy.get_time()))

            rospy.set_param('End','True')
            self.land()
            rospy.signal_shutdown("Code sucessfuly executed.")


        self.control_loop += 1

    def land(self):
        self.land_pub.publish(Empty())


if __name__ == '__main__':
    try:
        Model_Estimation = ModelEstimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass