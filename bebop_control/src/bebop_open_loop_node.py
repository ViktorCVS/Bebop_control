#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from custom_msgs.msg import CustomOdometry

import time
import os
import signal
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovementController:
    def __init__(self):

        rospy.init_node('bebop_open_loop', anonymous=True)

        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)

        rospy.loginfo("Aguardando o publicador para decolagem.")

        ## ------------------------------------------
        ## ---------- Interface do usuário ----------
        ## ------------------------------------------   

        self.tempo_para_decolar = 5
        self.tempo_para_iniciar = 10

        self.en_x   = False
        self.en_y   = False
        self.en_z   = False
        self.en_yaw = True

        ## ------------------------------------------
        ## ---------- Interface do usuário ----------
        ## ------------------------------------------

        while rospy.get_time()<self.tempo_para_decolar:
            pass

        rospy.loginfo("Decolando.")

        self.takeoff()

        while rospy.get_time()<self.tempo_para_iniciar:
            pass

        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.2), self.timer_callback)  

        self.twist_msg = Twist()
        
    def timer_callback(self, event):

        self.twist_msg.linear.x  = int(self.en_x) * self.ref_trajectory(rospy.get_time())
        self.twist_msg.linear.y  = int(self.en_y) * self.ref_trajectory(rospy.get_time())
        self.twist_msg.linear.z  = int(self.en_z) * self.ref_trajectory(rospy.get_time())
        self.twist_msg.angular.z = int(self.en_yaw) * self.ref_trajectory(rospy.get_time())

        self.cmd_vel_pub.publish(self.twist_msg)

    def ref_trajectory(self,t):

        u = 0.4/4.5*( 3*sin(0.2*pi*t) + sin(0.6*pi*t) + 0.5*sin(pi*t) )

        return u 


    def takeoff(self):
        self.takeoff_pub.publish(Empty())


if __name__ == '__main__':
    try:
        movement_controller = MovementController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass