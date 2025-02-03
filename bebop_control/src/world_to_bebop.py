#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty, Float64MultiArray
import numpy as np
from tf.transformations import euler_from_quaternion
from math import pi, atan2, sin, cos, sqrt, tanh
import os

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


def controle():
	
	x = current_pose.position.x
	y = current_pose.position.y
	z = current_pose.position.z

	rotx = current_euler[0]
	roty = current_euler[1]
	rotz = current_euler[2]
	
	roll = -rotx
	pitch = roty
	yaw = rotz

	if i==0:

		global xp, yp, zp, yawp, vx_k, vy_k, vz_k, vyaw_k,Pk
		
		xp = x
		yp = y
		zp = z
		yawp = yaw
		
		vx_k = np.array([[x],[0]])
		vy_k = np.array([[y],[0]])
		vz_k = np.array([[z],[0]])
		vyaw_k = np.array([[yaw],[0]])
		
		Pk=Pk0
		
	# Estima a velocidade (Euler)	
	vx = (x-xp)/Ts	
	vy = (y-yp)/Ts	
	vz = (z-zp)/Ts	
	vyaw = (yaw-yawp)/Ts

	raw_vel_msg = Float64MultiArray()
	raw_vel_msg.data = [vx,vy, vz, vyaw]
	raw_velocity_pub.publish(raw_vel_msg)
	
	# Filtro de Kalman	
		
	Pk=Ak*Pk*(Ak.T)+Qk
	ck=Pk.item((0,0))
	Kk=Pk*(Hk.T)/(Rk+ck)
	
	vx_k=Ak*vx_k
	vy_k=Ak*vy_k
	vz_k=Ak*vz_k
	vyaw_k=Ak*vyaw_k
	
	vx_k=vx_k+Kk*(x-Hk*vx_k)
	vy_k=vy_k+Kk*(y-Hk*vy_k)
	vz_k=vz_k+Kk*(z-Hk*vz_k)
	vyaw_k=vyaw_k+Kk*(yaw-Hk*vyaw_k)
	
	Pk=Pk-Kk*Hk*Pk

	vx_est=vx_k.item(1)
	vy_est=vy_k.item(1)
	vz_est=vz_k.item(1)
	vyaw_est=vyaw_k.item(1)

	rotation_matrix_world_to_robot = np.matrix([[cos(yaw), sin(yaw)],[-sin(yaw), cos(yaw)]])

	VXY_est = np.array([[vx_est],[vy_est]])

	VXY_est_rob = rotation_matrix_world_to_robot*VXY_est

	vx_est_robo = VXY_est_rob.item(0)
	vy_est_robo = VXY_est_rob.item(1)

	est_vel_msg = Float64MultiArray()
	est_vel_msg.data = [vx_est_robo,vy_est_robo, vz_est, vyaw_est]
	estimated_velocity_pub.publish(est_vel_msg)

	xp = x
	yp = y
	zp = z
	yawp = yaw
		

# Inicialização do Nó
rospy.init_node('Bebop_2_identificacao')		 


## --- NODE --- ##

global Ts, i
global Ak, Hk, Pk0, Qk, Rk


# Variáveis globais
current_pose = None
current_euler = None
i = 0


# Parâmetros do controlador

Taxa = 20 # Opera à Taxa [Hz] 

rate = rospy.Rate(Taxa)

Ts = 1/Taxa # Período de amostragem


# Parâmetros do filtro de Kalman

Ak=np.matrix([[1, Ts],[0, 1]])
Hk=np.matrix([[1 , 0]])
Pk0=np.matrix([[1, 0],[0, 1]])
Qk=np.matrix([[(Ts**4)/4, 0],[0, Ts**2]])
Rk=Ts**3

# Inicialização dos tópicos
rospy.Subscriber('/natnet_ros/Bebop2/pose', PoseStamped, get_pose)
raw_velocity_pub = rospy.Publisher('/bebop/vel_raw', Float64MultiArray, queue_size=10)
estimated_velocity_pub = rospy.Publisher('/bebop/vel_est', Float64MultiArray, queue_size=10)

tecla = input("Aperte a tecla  'Enter' para visualizar dados")

while not rospy.is_shutdown():           

	controle()

	i=i+1 
	rate.sleep()

rospy.spin()    	
		
		