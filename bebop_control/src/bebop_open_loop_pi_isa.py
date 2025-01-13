#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
import numpy as np
from tf.transformations import euler_from_quaternion
from datetime import datetime
from math import cos, sin

# Variáveis globais
velocity_message = Twist()
current_pose = None
current_euler = None
dados = None
i = 0

def get_euler_orientation(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    return euler_from_quaternion(quaternion)

def get_pose(message):
    global current_pose, current_euler
    current_pose = message.pose
    current_euler = get_euler_orientation(message.pose.orientation)
#       print("get_pose")
#       print(current_euler[2])

def controle(current_pose, current_euler):
    global i, dados, roll_p, picth_p, vx_rob_p, vy_rob_p, e_roll_p, e_pitch_p

    # Get pose
    x = current_pose.position.x
    y = current_pose.position.y
    z = current_pose.position.z
    rotx = current_euler[0]
    roty = current_euler[1]
    rotz = current_euler[2]
    
    roll=-rotx
    pitch=roty
    
    if i==0:
    	roll=-rotx
    	pitch=roty
    	roll_p=roll
    	pitch_p=pitch
    	vx_rob_p=0
    	vy_rob_p=0
    	e_roll_p=0
    	e_pitch_p=0
    else:
    	roll_p=roll
    	pitch_p=pitch
    	roll=-rotx
    	pitch=roty
    		
    	

    #print("Controle")
    #print(rotz)
    
    x_r=0
    y_r=0
    z_r=0
    vx_r=0
    vy_r=0
    vz_r=0
		    

    if i <= Voltas*(1/3)*Passos:
        vx = 0.12
        vy = 0.0
        vz = 0
        v_rot_z = 0.0
        print("v_pos")
  #  elif i > Voltas*(1/3)*Passos and i < Voltas*(2/3)*Passos:
  #      vx = 0.0 
  #      vy = 0.0
  #      vz = 0.0
  #      v_rot_z = 0.0
  #      print("v_rot_z = 0.0")       
    else:
        vx = 0.0
        vy = -0.12
        vz = 0.0
        v_rot_z = 0.0
        print("v_neg")
       
       
       

#    vx_rob = cos(rotz) * vx + sin(rotz) * vy
#    vy_rob = -sin(rotz) * vx + cos(rotz) * vy
#    vz_rob = vz

    #vx_rob = vx
    #vy_rob = vy
    
    # Erro ponderado (PI ISA)
    e_pitch=beta_x*Kcr_x*vx-pitch
    e_roll=beta_y*Kcr_y*vy-roll
        
    vx_rob = vx_rob_p+Ki_x*Ts*(Kcr_x*vx-pitch)-Kcm_x*(e_pitch-e_pitch_p)
    vy_rob = vy_rob_p+Ki_y*Ts*(Kcr_y*vy-roll)-Kcm_y*(e_roll-e_roll_p)
    
    e_pitch_p=e_pitch
    e_roll_p=e_roll
    
    vsat=0.5
    
    if vx_rob > vsat:
    	vx_rob=vsat
    elif vx_rob<-vsat:
    	vx_rob=-vsat	

    if vy_rob > vsat:
    	vy_rob=vsat
    elif vy_rob<-vsat:
    	vy_rob=-vsat	
        
    vz_rob = vz
    
    vx_rob_p=vx_rob
    vy_rob_p=vy_rob
    velocity_message.linear.x = vx_rob
    velocity_message.linear.y = vy_rob
    velocity_message.linear.z = vz_rob
    velocity_message.angular.z = v_rot_z

    for _ in range(1):  # Publicar a mensagem várias vezes
        velocity_position_pub.publish(velocity_message)

    t = i * Ts
    data_row = [t,x,x_r,y,y_r,z,z_r,vx,vx_r,vy,vy_r,vz,vz_r,vx_rob,vy_rob,vz_rob,v_rot_z,rotx,roty,rotz]
    if i == 0:
        dados = np.matrix(data_row)
    else:
        dados = np.append(dados, [data_row], axis=0)



# Inicialização do Nó
rospy.init_node('Bebop_2_identificacao')

# Parâmetros do controle
periodo = 2  # Período da volta 
Taxa = 20  # Frequência de operação [Hz] 
Ts = 1 / Taxa  # Período de amostragem
Voltas = 4  # Número de voltas'
Passos = int(periodo * Taxa)

# Sintonia cascata
global Kcm_x, Kcr_x, Kcm_y, Kcr_y, Ki_x, Ki_y, beta_x, beta_y

# Sintonia I+P (24/10)

Kcm_x=1.87 # ganho proporcional pitch
Kcm_y=1.87  # ganho proporcional roll

Ki_x=9.37  # ganho proporcional pitch
Ki_y=9.37 # ganho proporcional roll

beta_x=0 # pondera referência pitch
beta_y=0 # pondera referência roll

# Sintonia I+P com ponderação de referência (25/10)

# Kcm_x=1.87 # ganho proporcional pitch
# Kcm_y=1.87  # ganho proporcional roll

# Ki_x=9.37  # ganho proporcional pitch
# Ki_y=9.37 # ganho proporcional roll

# beta_x=0.3 # pondera referência pitch
# beta_y=0.3 # pondera referência roll

# Sintonia I+P com ponderação de referência (25/10)

# Kcm_x=1.1 # ganho proporcional pitch
# Kcm_y=1.1  # ganho proporcional roll

# Ki_x=5.5  # ganho proporcional pitch
# Ki_y=5.5  # ganho proporcional roll

# beta_x=0.5 # pondera referência pitch
# beta_y=0.5 # pondera referência roll

# ganhos do modelo em malha aberta (normaliza ganho)
ganho_pitch=0.426
ganho_roll=0.431


# Nomaliza ganho cascata
Kcr_x=ganho_pitch
Kcr_y=ganho_roll


# Inicialização dos tópicos
rospy.Subscriber('/natnet_ros/Bebop1/pose', PoseStamped, get_pose)
velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

rate = rospy.Rate(Taxa)
takeoff_msg = Empty()
land_msg = Empty()

# Decolagem
while takeoff_pub.get_num_connections() < 1:
    pass
takeoff_pub.publish(takeoff_msg)
rospy.sleep(7)
print("Take off ok")

# Controle de trajetória
while not rospy.is_shutdown() and i < Voltas * Passos + 1:
    if current_pose is not None and current_euler is not None:
        controle(current_pose, current_euler)
        i += 1
    rate.sleep()


# Pouso
while land_pub.get_num_connections() < 1:
    pass
land_pub.publish(land_msg)
print("Land ok")

# Salvando os dados
data_hora = datetime.now()
data_hora_texto = data_hora.strftime("dia_%d_%m_%Y_hora_%H_%M")
nome = f"test_x00_10hz_{data_hora_texto}.csv"
path = "/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/"
np.savetxt(path+nome, dados, delimiter=",")
print(f'Dados salvos em {nome}. Tecle Ctrl+C para finalizar o programa.')

rospy.spin()