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
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovementController:
    def __init__(self):

        rospy.init_node('bebop_open_loop', anonymous=True)

        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.path_pub = rospy.Publisher('/bebop2/path', Path, queue_size=10)

        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose_callback)

        rospy.loginfo("Aguardando o publicador para decolagem.")

        self.dados = []

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

        self.init_time = rospy.get_time()
        self.time = 0
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)  

        self.twist_msg = Twist()
        self.path_msg = Path()
        self.path_msg.header = Header()
        self.path_msg.header.frame_id = 'world'

        self.poses = []


    def get_pose_callback(self,msg):

        self.msg = msg
        
    def timer_callback(self, event):

        if self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<2:

            self.twist_msg.linear.x  = int(self.en_x) * 0.1
            self.twist_msg.linear.y  = int(self.en_y) * 0.1
            self.twist_msg.linear.z  = int(self.en_z) * 0.1
            self.twist_msg.angular.z = int(self.en_yaw) * 0.1
            self.cmd_vel_pub.publish(self.twist_msg)

        elif self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<4:

            self.twist_msg.linear.x  = int(self.en_x) * -0.1
            self.twist_msg.linear.y  = int(self.en_y) * -0.1
            self.twist_msg.linear.z  = int(self.en_z) * -0.1
            self.twist_msg.angular.z = int(self.en_yaw) * -0.1

            self.cmd_vel_pub.publish(self.twist_msg)

        elif self.cmd_control == 'open' and (rospy.get_time()-self.init_time)<44:

            self.twist_msg.linear.x  = int(self.en_x) * self.ref_trajectory(self.time)
            self.twist_msg.linear.y  = int(self.en_y) * self.ref_trajectory(self.time)
            self.twist_msg.linear.z  = int(self.en_z) * self.ref_trajectory(self.time)
            self.twist_msg.angular.z = int(self.en_yaw) * self.ref_trajectory(self.time)

            self.time += 0.05

            self.dados.append([rospy.get_time(),self.twist_msg.linear.x,self.twist_msg.linear.y,self.twist_msg.linear.z,self.twist_msg.angular.z,self.msg.x,self.msg.y,self.msg.z,self.msg.roll,self.msg.pitch,self.msg.yaw])

            self.cmd_vel_pub.publish(self.twist_msg)

        else:

            rospy.loginfo("Fim de execução.")
            with open(f'/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/dados_{self.Axis}_{self.experimento}', mode='w') as file:

                writer = csv.writer(file)
                writer.writerow(['Tempo','ux','uy','uz','uw','x','y','z','roll','pitch','yaw'])
                writer.writerows(self.dados)

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
        

    def ref_trajectory(self,t):

        if self.en_x or self.en_y:
            if self.Estimation:
                u = 0.02*( 1*sin(0.3*pi*t) - 1*sin(0.1*pi*t) + 3*sin(0.5*pi*t) )
            else:
                u = 0.1

        else:
            if self.Estimation:
                u = 0.15*( 2*sin(0.9*pi*t) + sin(0.7*pi*t) - 0.5*sin(1*pi*t) + 0.8*sin(0.5*pi*t))

            else:
                u = 0.1

        return u 


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