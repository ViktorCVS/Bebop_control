#!/usr/bin/env python3

# Biblioteca base do ROS
import rospy

# Mensagens utilizadas
from geometry_msgs.msg import Twist
from custom_msgs.msg import CustomOdometry
from std_msgs.msg import Empty

# Bibliotecas do python
import numpy as np
from tf.transformations import euler_from_quaternion
from math import pi, atan2, sin, cos
from datetime import datetime
import os


class BebopPDTrajectory:
    def __init__(self):
        
        rospy.init_node('bebop_pd_trajectory_node', anonymous=True)

        # Publicadores para decolagem e pouso.
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        
        # Publicador de velocidade e inscrição para receber o GroundTruth.
        self.velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/adjusted_odometry', CustomOdometry, self.get_pose)

        rospy.loginfo("Inicializando variáveis de publicação e inscrição.")

        # Inicializa a mensagem de velocidade.
        self.velocity_message = Twist()

        ## ------------------------------------------
        ## ---------- Interface do usuário ----------
        ## ------------------------------------------    

        # Tempo para decolar e iniciar o controle.
        self.tempo_para_decolar = 10
        self.tempo_para_iniciar = 20

        # Taxa de amostragem em Hz.
        self.taxa = 10
        self.Ts = 1/self.taxa

        # Variáveis para armazenar a posição passada.
        self.xp = 0
        self.yp = 0
        self.zp = 0

        # Translação para a posição inicial ser 0,0,0,0.
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        # Tempo de execução da trajetória
        self.periodo = 40

        # Frequência dos senos e cossenos da Lemniscata
        self.wx = 2*pi/self.periodo
        self.wy = 4*pi/self.periodo
        self.wz = 2*pi/self.periodo

        # Amplitudes dos senos e cossenos da Lemniscata
        self.Ax = 1
        self.Ay = 0.6
        self.Az = 0.3

        # Ponderação do esforço de controle
        self.rho = 1 

        # Número de voltas para o controle.
        self.Voltas = 1 

        # Horizonte do filtro média móvel.
        self.Horizonte_MM = 5 

        # Cálculo do número de passos necessários para o controle.
        self.Passos = int(self.periodo*self.taxa)

        # alpha_x e alpha_y são os polos discretos do modelo do bebop2. k2 e k4
        self.alpha_x = 0.975
        self.alpha_y = 0.982

        # Ganho em tempo discreto do modelo do bebop2
        self.k_x = 0.1
        self.k_y = 0.1

        # Ganho proporcional e derivativo do controlador PD.
        self.Kp_x = 0.3
        self.Kd_x = 1.05
        
        self.Kp_y = 0.3
        self.Kd_y = 1.07
        
        # Velocidade máxima que o drone pode atingir.
        self.velocity_saturation = 0.5

        # Matrix de dados para armazenas variáveis de interesse
        self.dados = np.matrix([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

        ## ------------------------------------------
        ## ---------- Interface do usuário ----------
        ## ------------------------------------------  

        while rospy.get_time()<self.tempo_para_decolar:
            pass

        self.takeoff()

        while rospy.get_time()<self.tempo_para_iniciar:
            pass

        self.control_variable = 0

        rospy.loginfo("Iniciando controle.")
        self.control_timer = rospy.Timer(rospy.Duration(self.Ts), self.controle_callback)  

    def takeoff(self):

        rospy.loginfo("Decolando.")
        self.takeoff_pub.publish(Empty())

    def land(self, event):

        rospy.loginfo("Pousando.")
        self.land_pub.publish(Empty())

    def get_pose(self, message):
        
        self.current_pose = message

    def kill_node(self, event):

        rospy.loginfo("Finalizando nó.")
        rospy.signal_shutdown("Code sucessfuly executed.")

    def controle_callback(self, event):

        tempo = self.control_variable*self.Ts
        
        if( self.control_variable == self.Horizonte_MM ):
        
            self.x0 = self.current_pose.x
            self.y0 = self.current_pose.y
            self.z0 = self.current_pose.z
            
        elif( self.control_variable < self.Horizonte_MM ):

            self.x0 = 0
            self.y0 = 0
            self.z0 = 1
        
        # Calcula referências	
        x_r   = self.x0 + self.Ax*sin( self.wx*(self.control_variable-self.Horizonte_MM)*self.Ts )
        x_rf  = self.x0 + self.Ax*sin( self.wx*(self.control_variable-self.Horizonte_MM+1)*self.Ts )
        x_rff = self.x0 + self.Ax*sin( self.wx*(self.control_variable-self.Horizonte_MM+2)*self.Ts )
        
        y_r   = self.y0 + self.Ay*sin( self.wy*(self.control_variable-self.Horizonte_MM)*self.Ts )
        y_rf  = self.y0 + self.Ay*sin( self.wy*(self.control_variable-self.Horizonte_MM+1)*self.Ts )
        y_rff = self.y0 + self.Ay*sin( self.wy*(self.control_variable-self.Horizonte_MM+2)*self.Ts )
        
        z_r   = self.z0 + self.Az*sin( self.wz*(self.control_variable-self.Horizonte_MM)*self.Ts )
        z_rf  = self.z0 + self.Az*sin( self.wz*(self.control_variable-self.Horizonte_MM+1)*self.Ts )
        z_rff = self.z0 + self.Az*sin( self.wz*(self.control_variable-self.Horizonte_MM+2)*self.Ts )
        
        # com feedforward 
        vff_x = ( (x_rff-x_rf) - self.alpha_x*(x_rf-x_r) ) / (self.k_x*self.Ts)
        vff_y = ( (y_rff-y_rf) - self.alpha_y*(y_rf-y_r) ) / (self.k_y*self.Ts)
        
        # sem feedforward
        x = self.current_pose.x
        y = self.current_pose.y
        z = self.current_pose.z
        
        # Inicializa valores passados
        if( self.control_variable == 0 ):

            self.xp = x
            self.yp = y
            self.zp = z
            
        # Estima a velocidade (Euler)	
        vx = (x-self.xp) / self.Ts
        vy = (y-self.yp) / self.Ts
        
        # Filtro média móvel
        if( self.control_variable >= self.Horizonte_MM ):
        
            soma_x = vx
            soma_y = vy
        
            Nx = np.array(self.dados[:,13]) 
            Ny = np.array(self.dados[:,14]) 
        
            for control_sum in range(0,self.Horizonte_MM-1):

                soma_x += Nx.item(control_sum)
                soma_y += Ny.item(control_sum)
        
            vx_filt = soma_x / self.Horizonte_MM
            vy_filt = soma_y / self.Horizonte_MM
            
        else:

            vx_filt = vx
            vy_filt = vy
            
        # Armazena o valor passado
        self.xp = x
        self.yp = y
        self.zp = z
        
        # velocidades de referência
        vx_r = (x_rf-x_r) / (self.Ts)
        vy_r = (y_rf-y_r) / (self.Ts)
        vz_r = (z_rf-z_r) / (self.Ts)
        
        # novo controlador de posição
        vc_x = -self.Kp_x*(x-x_r) - self.Kd_x*(vx_filt-vx_r) + vff_x
        vc_y = -self.Kp_y*(y-y_r) - self.Kd_y*(vy_filt-vy_r) + vff_y
        vc_z = self.Kc_z*(z-z_r) + vz_r	
        
        if( self.control_variable >= self.Voltas*self.Passos ):

            vc_x = 0
            vc_y = 0
            vc_z = 0

            rospy.loginfo("Controle finalizado.")
            rospy.Timer(rospy.Duration(10), self.land, oneshot=True)
            rospy.Timer(rospy.Duration(20), self.kill_node, oneshot=True)

            self.control_timer.shutdown()


        theta = self.current_pose.yaw
        
        # Rever a transformação após cálculo dos ganhos
        vrob_x = cos(theta)*vc_x  + sin(theta)*vc_y
        vrob_y = -sin(theta)*vc_x + cos(theta)*vc_y
        vrob_z = vc_z
        
        if( vrob_x > self.velocity_saturation ):
            vrob_x   = self.velocity_saturation
        elif( vrob_x < -self.velocity_saturation ):
            vrob_x   = -self.velocity_saturation
        
        if( vrob_y > self.velocity_saturation ):
            vrob_y = self.velocity_saturation
        elif( vrob_y < -self.velocity_saturation ):
            vrob_y   = -self.velocity_saturation

        if( vrob_z > self.velocity_saturation ):
            vrob_z = self.velocity_saturation
        elif( vrob_z < -self.velocity_saturation ):
            vrob_z   = -self.velocity_saturation
            
        if( self.control_variable < self.Horizonte_MM ):

            vrob_x = 0
            vrob_y = 0
            vrob_z = 0

            vc_x = 0
            vc_y = 0
            vc_z = 0	
        

        self.velocity_message.linear.x = vrob_x
        self.velocity_message.linear.y = vrob_y
        self.velocity_message.linear.z = vrob_z

        self.velocity_pub.publish(self.velocity_message)

        if( self.control_variable == 0):   

            self.dados=np.matrix([[tempo,x,x_r,y,y_r,z,z_r,vc_x,vx_r,vc_y,vy_r,vc_z,vz_r,vx,vy]])
            
                
        else:

            self.dados=np.append(self.dados,[[tempo,x,x_r,y,y_r,z,z_r,vc_x,vx_r,vc_y,vy_r,vc_z,vz_r,vx,vy]],axis=0)
        

        self.control_variable += 1


if __name__ == '__main__':
    try:
        trajectory_controller = BebopPDTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


  
