#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Header
from custom_msgs.msg import CustomOdometry

import time
import os
import signal
import csv
import numpy as np
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovementController:
    def __init__(self):

        rospy.init_node('bebop_open_loop', anonymous=True)

        self.twist_msg = Twist()
        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.path = "/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/roll_1.csv"
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.path_pub = rospy.Publisher('/bebop2/path', Path, queue_size=10)

        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose_callback)

        rospy.loginfo("Aguardando o publicador para decolagem.")

        self.dados = []

        self.signal_x_passado = 0
        self.signal_y_passado = 0
        self.signal_z_passado = 0
        self.signal_yaw_passado = 0

        self.e_pitch_passado = 0
        self.e_roll_passado = 0
        self.e_yaw_passado = 0

        self.Axis = rospy.get_param('Axis')
        self.Estimation = rospy.get_param('Estimation')
        self.cmd_control = rospy.get_param('Node')

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

        time.sleep(2)

        rospy.loginfo("Decolando.")

        self.takeoff()

        time.sleep(10)

        rospy.loginfo("Iniciando.")

        self.time = 0
        self.experimento = 12

        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        
        self.time = 0
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)  

        
        self.path_msg = Path()
        self.path_msg.header = Header()
        self.path_msg.header.frame_id = 'world'
        self.poses = []
        self.init_time = rospy.get_time()

    def get_pose_callback(self,msg):

        self.msg = msg
        try:
            with open(self.path,'a') as file:
                file.write(f",{msg.x},{msg.y},{msg.z},{msg.roll},{msg.pitch},{msg.yaw},{self.twist_msg.linear.x},{self.twist_msg.linear.y},{rospy.get_time()-self.init_time}")
        except:
            pass

    def timer_callback(self, event):

        Kcm_x = 1.87
        Kcm_y = 1.87

        Ki_x = 9.37
        Ki_y = 9.37

        beta_x = 0.3
        beta_y = 0.3

        if self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<3.5:

            vx = 0.05
            vy = 0.05

            e_pitch = beta_x*Kcr_x*vx-msg.pitch
            e_roll  = beta_y*Kcr_y*vy-msg.roll

            signal_x = self.signal_x_passado + Ki_x*Ts*(Kcr_x*vx-msg.pitch)-Kcm_x*(e_pitch-e_pitch_passado)
            signal_y = self.signal_y_passado + Ki_y*Ts*(Kcr_y*vy-msg.roll)-Kcm_y*(e_roll-e_roll_passado)

            e_pitch_passado = e_pitch
            e_roll_passado = e_roll

            vsat_x = 0.5
            vsat_y = 0.5

            if signal_x > vsat_x:
                signal_x = vsat_x
            elif signal_x < -vsat_x:
                signal_x = -vsat_x
            
            if signal_y > vsat_y:
                signal_y = vsat_y
            elif signal_y < -vsat_y:
                signal_y = -vsat_y

            self.signal_x_passado = signal_x
            self.signal_y_passado = signal_y

            self.twist_msg.linear.x  = int(self.en_x) * signal_x
            self.twist_msg.linear.y  = int(self.en_y) * signal_y
            self.twist_msg.linear.z  = int(self.en_z) * 0.1
            self.twist_msg.angular.z = int(self.en_yaw) * 0.1
            self.cmd_vel_pub.publish(self.twist_msg)

        elif self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<7:

            vx = -0.10
            vy = -0.10

            self.twist_msg.linear.x  = int(self.en_x) * signal_x
            self.twist_msg.linear.y  = int(self.en_y) * signal_y
            self.twist_msg.linear.z  = int(self.en_z) * -0.2
            self.twist_msg.angular.z = int(self.en_yaw) * -0.2

            self.cmd_vel_pub.publish(self.twist_msg)

        elif self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<14:

            vx = 0.05
            vy = 0.05

            e_pitch = beta_x*Kcr_x*vx-msg.pitch
            e_roll  = beta_y*Kcr_y*vy-msg.roll

            signal_x = self.signal_x_passado + Ki_x*Ts*(Kcr_x*vx-msg.pitch)-Kcm_x*(e_pitch-e_pitch_passado)
            signal_y = self.signal_y_passado + Ki_y*Ts*(Kcr_y*vy-msg.roll)-Kcm_y*(e_roll-e_roll_passado)

            e_pitch_passado = e_pitch
            e_roll_passado = e_roll

            vsat_x = 0.5
            vsat_y = 0.5

            if signal_x > vsat_x:
                signal_x = vsat_x
            elif signal_x < -vsat_x:
                signal_x = -vsat_x
            
            if signal_y > vsat_y:
                signal_y = vsat_y
            elif signal_y < -vsat_y:
                signal_y = -vsat_y

            self.signal_x_passado = signal_x
            self.signal_y_passado = signal_y

            self.twist_msg.linear.x  = int(self.en_x) * signal_x
            self.twist_msg.linear.y  = int(self.en_y) * signal_y
            self.twist_msg.linear.z  = int(self.en_z) * 0.1
            self.twist_msg.angular.z = int(self.en_yaw) * 0.1

            self.cmd_vel_pub.publish(self.twist_msg)

        else:
            rospy.loginfo("Fim de execução.")

            self.land()




        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.msg.x
        pose.pose.position.y = self.msg.y
        pose.pose.position.z = self.msg.z
        pose.pose.orientation.w = self.msg.yaw
        self.poses.append(pose)

        self.path_msg.poses = self.poses

        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)

        print(rospy.get_time()-self.init_time)


    def takeoff(self):
        self.takeoff_pub.publish(Empty())


    def land(self):
        self.land_pub.publish(Empty())


if __name__ == '__main__':
    try:
        movement_controller = MovementController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass