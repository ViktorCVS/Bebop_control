#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from custom_msgs.msg import CustomOdometry

import time
import os
import signal
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovementController:
    def __init__(self):

        rospy.init_node('bebop_open_linear', anonymous=True)

        self.rate = rospy.Rate(50)
        self.control_info = [True]*15

        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        rospy.loginfo("Aguardando o publicador para decolagem.")

        self.tempo_para_decolar = 10
        self.tempo_para_iniciar = 20
        self.tempo_por_velocidade = 20

        self.en_x   = True
        self.en_y   = False
        self.en_z   = False
        self.en_yaw = False

        self.mapa_de_velocidades = [0.1,0.2,0.5,1.0,2.0,5.0]

        while rospy.get_time()<self.tempo_para_decolar:
            pass

        rospy.loginfo("Decolando.")

        self.takeoff()

        while rospy.get_time()<self.tempo_para_iniciar - 0.02:
            pass

        self.cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  

        rospy.loginfo("Variável de publicação iniciada.")

        self.twist_msg = Twist()
        
    def timer_callback(self, event):

        if rospy.get_time()<self.tempo_para_iniciar+self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[0]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[0]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[0]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[0]

            if(self.control_info[0] == True):

                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[0])
                self.control_info[0] = False

        elif rospy.get_time()<self.tempo_para_iniciar+2*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[1]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[1]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[1]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[1]

            if(self.control_info[1] == True):

                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[1])
                self.control_info[1] = False

        elif rospy.get_time()<self.tempo_para_iniciar+3*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[2]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[2]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[2]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[2]

            if(self.control_info[2] == True):

                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[2])
                self.control_info[2] = False


        elif rospy.get_time()<self.tempo_para_iniciar+4*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[3]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[3]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[3]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[3]

            if(self.control_info[3] == True):
                    
                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[3])
                self.control_info[3] = False


        elif rospy.get_time()<self.tempo_para_iniciar+5*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[4]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[4]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[4]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[4]

            if(self.control_info[4] == True):

                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[4])
                self.control_info[4] = False

        elif rospy.get_time()<self.tempo_para_iniciar+6*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = int(self.en_x) * self.mapa_de_velocidades[5]
            self.twist_msg.linear.y  = int(self.en_y) * self.mapa_de_velocidades[5]
            self.twist_msg.linear.z  = int(self.en_z) * self.mapa_de_velocidades[5]
            self.twist_msg.angular.z = int(self.en_yaw) * self.mapa_de_velocidades[5]

            if(self.control_info[5] == True):
                    
                rospy.loginfo("Trajetória de %s m/s iniciada.",self.mapa_de_velocidades[5])
                self.control_info[5] = False

        elif rospy.get_time()<self.tempo_para_iniciar+7*self.tempo_por_velocidade:

            self.twist_msg.linear.x = 0.0
            self.twist_msg.linear.y = 0.0
            self.twist_msg.linear.z = 0.0
            self.twist_msg.angular.z = 0.0

            if(self.control_info[6] == True):
                    
                rospy.loginfo("Trajetória de ida finalizada. Iniciando a volta.")
                self.control_info[6] = False
        
        elif rospy.get_time()<self.tempo_para_iniciar+8*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[0]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[0]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[0]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[0]

            if(self.control_info[7] == True):

                rospy.loginfo("Trajetória de -0,1 m/s iniciada.")
                self.control_info[7] = False

        elif rospy.get_time()<self.tempo_para_iniciar+9*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[1]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[1]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[1]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[1]

            if(self.control_info[8] == True):

                rospy.loginfo("Trajetória de -0,2 m/s iniciada.")
                self.control_info[8] = False


        elif rospy.get_time()<self.tempo_para_iniciar+10*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[2]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[2]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[2]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[2]

            if(self.control_info[9] == True):
                    
                rospy.loginfo("Trajetória de -0,5 m/s iniciada.")
                self.control_info[9] = False


        elif rospy.get_time()<self.tempo_para_iniciar+11*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[3]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[3]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[3]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[3]

            if(self.control_info[10] == True):

                rospy.loginfo("Trajetória de -1,0 m/s iniciada.")
                self.control_info[10] = False

        elif rospy.get_time()<self.tempo_para_iniciar+12*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[4]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[4]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[4]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[4]

            if(self.control_info[11] == True):
                    
                rospy.loginfo("Trajetória de -2,0 m/s iniciada.")
                self.control_info[11] = False


        elif rospy.get_time()<self.tempo_para_iniciar+13*self.tempo_por_velocidade:

            self.twist_msg.linear.x  = -int(self.en_x) * self.mapa_de_velocidades[5]
            self.twist_msg.linear.y  = -int(self.en_y) * self.mapa_de_velocidades[5]
            self.twist_msg.linear.z  = -int(self.en_z) * self.mapa_de_velocidades[5]
            self.twist_msg.angular.z = -int(self.en_yaw) * self.mapa_de_velocidades[5]

            if(self.control_info[12] == True):
                    
                rospy.loginfo("Trajetória de -5,0 m/s iniciada.")
                self.control_info[12] = False

        elif rospy.get_time()<self.tempo_para_iniciar+14*self.tempo_por_velocidade:

            self.twist_msg.linear.x = 0.0
            self.twist_msg.linear.y = 0.0
            self.twist_msg.linear.z = 0.0
            self.twist_msg.angular.z = 0.0

            if(self.control_info[13] == True):
                    
                rospy.loginfo("Trajetória de volta finalizada.")
                self.control_info[13] = False

        elif rospy.get_time()<self.tempo_para_iniciar+15*self.tempo_por_velocidade:

            if(self.control_info[14] == True):

                rospy.loginfo("Pousando.")
                self.land()
                self.control_info[14] = False

        if rospy.get_time()>=self.tempo_para_iniciar+15*self.tempo_por_velocidade:

            rospy.loginfo("Pronto para encerrar.")
            os.kill(os.getpid(), signal.SIGINT)


        self.cmd_vel_pub.publish(self.twist_msg)

        self.rate.sleep()

        

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