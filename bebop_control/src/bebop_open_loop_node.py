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
        self.en_z   = True
        self.en_yaw = False

        ## ------------------------------------------
        ## ---------- Interface do usuário ----------
        ## ------------------------------------------

        time.sleep(2)

        rospy.loginfo("Decolando.")

        self.takeoff()

        time.sleep(8)

        rospy.loginfo("Iniciando.")

        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.2), self.timer_callback)  

        self.twist_msg = Twist()

        self.control_variable = 0

        self.init_time = rospy.get_time()
        
    def timer_callback(self, event):

        self.twist_msg.linear.x  = int(self.en_x) * self.ref_trajectory(rospy.get_time()-self.init_time)
        self.twist_msg.linear.y  = int(self.en_y) * self.ref_trajectory(rospy.get_time()-self.init_time)
        self.twist_msg.linear.z  = int(self.en_z) * self.ref_trajectory(rospy.get_time()-self.init_time)
        self.twist_msg.angular.z = int(self.en_yaw) * self.ref_trajectory(rospy.get_time()-self.init_time)

        self.cmd_vel_pub.publish(self.twist_msg)

        self.control_variable += 1

    def ref_trajectory(self,t):

        if self.en_x or self.en_y:
            u = 0.08*( 0.5*sin(0.2*pi*t) + sin(0.6*pi*t) + 0.5*sin(pi*t) )
        else:
            u = 0.2*( 0.5*sin(0.2*pi*t) + sin(0.6*pi*t) + 0.5*sin(pi*t) )

        return u 


    def takeoff(self):
        self.takeoff_pub.publish(Empty())


if __name__ == '__main__':
    try:
        movement_controller = MovementController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
