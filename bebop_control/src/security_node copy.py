#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from custom_msgs.msg import CustomOdometry

from math import sin, cos


class SecurityNode:
    def __init__(self):

        rospy.init_node('security_node', anonymous=True)

        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose_callback)
        rospy.Subscriber('/bebop/cmd_vel', Twist, self.get_cmd_vel_callback)

        rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged , self.get_battery_callback)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        self.x_limits = [-2.7,2.7]
        self.y_limits = [-1.8,1.8]
        self.z_limit = 2.4

        self.WARNING = 0.9
        self.CMD_WARNING = 2

        self.CONTADOR_POSE = [0,0,0,0]
        self.CONTADOR_CMD = [0,0,0,0]

        self.TEMPO_CONTADOR_CMD = 600
        self.TEMPO_CONTADOR_POSE = 2000

        self.DELAY_CMD = [True,True,True,True]

        self.DELAY_POSE = [True,True,True]

        rospy.Timer(rospy.Duration(3), self.battery_callback) 

        self.twist_msg = Twist()

    def get_cmd_vel_callback(self,msg):

        # if msg.linear.x != 0:
        #     if self.pose_msg.dx > msg.linear.x*self.CMD_WARNING:
        #         if self.DELAY_CMD[0] == True:
        #             rospy.loginfo("Velocidade em X acima do enviado!")
        #             self.DELAY_CMD[0] = False
        # else:
        #     if self.pose_msg.dx > 0.1:
        #         if self.DELAY_CMD[0] == True:
        #             rospy.loginfo("Velocidade em X acima do enviado!")
        #             self.DELAY_CMD[0] = False

        # if msg.linear.y != 0:
        #     if self.pose_msg.dy > msg.linear.y*self.CMD_WARNING:
        #         if self.DELAY_CMD[1] == True:
        #             rospy.loginfo("Velocidade em Y acima do enviado!")
        #             self.DELAY_CMD[1] = False
        # else:
        #     if self.pose_msg.dy > 0.1:
        #         if self.DELAY_CMD[1] == True:
        #             rospy.loginfo("Velocidade em Y acima do enviado!")
        #             self.DELAY_CMD[1] = False

        # if msg.linear.z != 0:
        #     if self.pose_msg.dz > msg.linear.z*self.CMD_WARNING:
        #         if self.DELAY_CMD[2] == True:
        #             rospy.loginfo("Velocidade em Z acima do enviado!")
        #             self.DELAY_CMD[2] = False
        # else:
        #     if self.pose_msg.dz > 0.1:
        #         if self.DELAY_CMD[2] == True:
        #             rospy.loginfo("Velocidade em Z acima do enviado!")
        #             self.DELAY_CMD[2] = False

        # if msg.angular.z != 0:
        #     if self.pose_msg.dyaw > msg.angular.z*self.CMD_WARNING:
        #         if self.DELAY_CMD[3] == True:
        #             rospy.loginfo("Velocidade em YAW acima do enviado!")
        #             self.DELAY_CMD[3] = False
        # else:
        #     if self.pose_msg.dyaw > 0.1:
        #         if self.DELAY_CMD[3] == True:
        #             rospy.loginfo("Velocidade em YAW acima do enviado!")
        #             self.DELAY_CMD[3] = False

        self.CONTADOR_CMD[0] +=1
        self.CONTADOR_CMD[1] +=1
        self.CONTADOR_CMD[2] +=1
        self.CONTADOR_CMD[3] +=1

        if self.CONTADOR_CMD[0]%self.TEMPO_CONTADOR_CMD==0:
            self.DELAY_CMD[0] = True
            self.DELAY_CMD[1] = True
            self.DELAY_CMD[2] = True
            self.DELAY_CMD[3] = True



    def get_battery_callback(self,msg):

        self.msg = msg

    def battery_callback(self,event):

        if self.msg.percent >= 75: 
            print(f"[LOW PRIORITY] BATTERY: {self.msg.percent}%")
            

        elif self.msg.percent >= 33: 
            print(f"[MEDIUM PRIORITY] BATTERY: {self.msg.percent}%")


        elif self.msg.percent < 33: 
            print(f"[HIGH PRIORITY] BATTERY: {self.msg.percent}%")

    def get_pose_callback(self, msg):

        self.pose_msg = msg

        if msg.x >= self.x_limits[1] or msg.x <= self.x_limits[0]:
            #rospy.loginfo("Limite em X atingido!")
            self.land()
        elif msg.x >= self.x_limits[1]*self.WARNING or msg.x <= self.x_limits[0]*self.WARNING:
            if self.DELAY_POSE[0] == True:
                rospy.loginfo("Limite em X perto de ser atingido!")
                self.DELAY_POSE[0] = False
            
        if msg.y >= self.y_limits[1] or msg.y <= self.y_limits[0]:
            #rospy.loginfo("Limite em Y atingido!")
            self.land()
        elif msg.y >= self.y_limits[1]*self.WARNING or msg.y <= self.y_limits[0]*self.WARNING:
            if self.DELAY_POSE[1] == True:
                rospy.loginfo("Limite em Y perto de ser atingido!")
                self.DELAY_POSE[1] = False
            
        if msg.z >= self.z_limit:
            #rospy.loginfo("Limite em z atingido!")
            self.land()
        elif msg.z >= self.z_limit*self.WARNING:
            if self.DELAY_POSE[2] == True:
                rospy.loginfo("Limite em Z perto de ser atingido!")
                self.DELAY_POSE[2] = False
            
        self.CONTADOR_POSE[0] +=1
        self.CONTADOR_POSE[1] +=1
        self.CONTADOR_POSE[2] +=1

        if self.CONTADOR_POSE[0]%self.TEMPO_CONTADOR_POSE==0:
            self.DELAY_POSE[0] = True
            self.DELAY_POSE[1] = True
            self.DELAY_POSE[2] = True
        

    def land(self):

        rospy.set_param('Node', 'security')
        self.land_pub.publish(Empty())
        rospy.signal_shutdown("Code sucessfuly executed.")


if __name__ == '__main__':
    try:
        Security_Node = SecurityNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass