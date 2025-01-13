#!/usr/bin/env python3

import rospy
import time ## controlling the time 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Vector3
import os
#from std_msgs.msg import Empty
from datetime import datetime
from math import pi, atan2, sin, cos, sqrt
from custom_msgs.msg import CustomOdometry
#from sensor_msgs.msg import Joy


velocity_message = Twist()


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
	current_pose = [message.x,message.y,message.z]
	current_euler = [message.roll,message.pitch,message.yaw]

def controle(xp,yp,zp,thetap):

	'''
	Input:
	(x,y,z) = vector of position

	Task:
	publish velocity messages during the time linked to position
	'''
	t=i*Ts
	
	global dados
	
	if  i==M:
		global x0,y0,z0
	
		x0=current_pose[0]
		y0=current_pose[1]
		z0=current_pose[2]
		
	elif i<M:
		x0=0
		y0=0
		z0=1
	
	
	
	# Calcula referências	
	x_r=x0+Ax*sin(wx*(i-M)*Ts)
	x_rf=x0+Ax*sin(wx*(i-M+1)*Ts)
	x_rff=x0+Ax*sin(wx*(i-M+2)*Ts)
	
	dx_rf=(x_rff-x_rf)/Ts
	dx_r=(x_rf-x_r)/Ts
	
	ax_r=(dx_rf-dx_r)/Ts
	
	y_r=y0+Ay*sin(wy*(i-M)*Ts)
	y_rf=y0+Ay*sin(wy*(i-M+1)*Ts)
	y_rff=y0+Ay*sin(wy*(i-M+2)*Ts)
	
	dy_rf=(y_rff-y_rf)/Ts
	dy_r=(y_rf-y_r)/Ts
	
	ay_r=(dy_rf-dy_r)/Ts
	
	z_r=z0+Az*sin(wz*(i-M)*Ts)
	z_rf=z0+Az*sin(wz*(i-M+1)*Ts)
	z_rff=z0+Az*sin(wz*(i-M+2)*Ts)
	
	dz_rf=(z_rff-z_rf)/Ts
	dz_r=(z_rf-z_r)/Ts
	
	az_r=(dz_rf-dz_r)/Ts
	
	theta_r=At*sin(wt*(i-M)*Ts)
	theta_rf=At*sin(wt*(i-M+1)*Ts)
	theta_rff=At*sin(wt*(i-M+2)*Ts)

	
	dtheta_rf=(theta_rff-theta_rf)/Ts
	dtheta_r=(theta_rf-theta_r)/Ts
	
	atheta_r=(dtheta_rf-dtheta_r)/Ts
	
		
	
	# feedforward de controle 
	vff_x=(ax_r+f2x*dx_r)/f1x
	vff_y=(ay_r+f2y*dy_r)/f1y
	vff_z=(az_r+f2z*dz_r)/f1z
	vff_theta=(atheta_r+f2theta*dtheta_r)/f1theta
	
	
		
	x=current_pose[0]
	y=current_pose[1]
	z=current_pose[2]
	theta=current_euler[2]
	
	# Inicializa valores passados
	if i==0:
		xp=x
		yp=y
		zp=z
		thetap=theta
		
	
	# Estima a velocidade (Euler)	
	vx=(x-xp)/Ts
	
	vy=(y-yp)/Ts
	
	vz=(z-zp)/Ts
	
	vtheta=(theta-thetap)/Ts
	
	# Filtro média móvel
	
	if i>=M:
	
		soma_x=vx
		soma_y=vy
		soma_z=vz
		soma_theta=vtheta
	
		Nx=np.array(dados[:,17]) 
		Ny=np.array(dados[:,18]) 
		Nz=np.array(dados[:,19]) 
		Ntheta=np.array(dados[:,20]) 
	
		for c in range(0,M-1):
			soma_x=soma_x+Nx.item(c)
			soma_y=soma_y+Ny.item(c)
			soma_z=soma_z+Nz.item(c)
			soma_theta=soma_theta+Ntheta.item(c)
	
		vx_filt=soma_x/M
		vy_filt=soma_y/M
		vz_filt=soma_z/M
		vtheta_filt=soma_theta/M
		
	else:
		vx_filt=vx
		vy_filt=vy
		vz_filt=vz
		vtheta_filt=vtheta
		
	
	
	# Armazena o valor passado
	xp=x
	yp=y
	zp=z
	thetap=theta
	
	
	
	Ri=np.matrix([[cos(theta), sin(theta)],[-sin(theta), cos(theta)]])
	
	# velocidades de referência
	vx_r=(x_rf-x_r)/(Ts)
	vy_r=(y_rf-y_r)/(Ts)
	vz_r=(z_rf-z_r)/(Ts)
	vtheta_r=(theta_rf-theta_r)/(Ts)
	
	Kp=np.array([[-Kp_x, 0],[0, -Kp_y]])
	Kd=np.array([[-Kd_x, 0],[0, -Kd_y]])
	
	
	XY=np.array([[x-x_r],[y-y_r]])
	
	VXY=np.array([[vx_filt-vx_r],[vy_filt-vy_r]])
	
	Vff=np.array([[vff_x],[vff_y]])
	
	Vc=Kp*Ri*XY+Kd*Ri*VXY+Ri*Vff
	
		
	vc_x=Vc.item(0)
	
	vc_y=Vc.item(1)
	
	#vff_z=0
	#vff_theta=0
	
		
	vc_z=-Kp_z*(z-z_r)-Kd_z*(vz-vz_r)+vff_z
	
	
	vc_theta=-Kp_theta*(theta-theta_r)-Kd_theta*(vtheta-vtheta_r)+vff_theta
	
	
	if i>= Voltas*Passos:
		vc_x=0
		vc_y=0
		vc_z=0
	
	vsat=0.5
	
	

	
	# Rever a transformação após cálculo dos ganhos
	vrob_x=vc_x
	vrob_y=vc_y
	vrob_z=vc_z
	vrob_theta=vc_theta
	
	if vrob_x>vsat:
		vc_x=vsat
	elif vrob_x<-vsat:
		vrob_x=-vsat
	   
	if vrob_y>vsat:
		vrob_y=vsat
	elif vrob_y<-vsat:
		vrob_y=-vsat

	if vrob_z>vsat:
		vrob_z=vsat
	elif vrob_z<-vsat:
		vrob_z=-vsat
		
	if i<M:
		vrob_x=0
		vrob_y=0
		vrob_z=0
		vc_x=0
		vc_y=0
		vc_z=0	
	
	velocity_message.linear.x = vrob_x
	velocity_message.linear.y = vrob_y	
	velocity_message.linear.z = vrob_z
	velocity_message.angular.z = vrob_theta
	

	
	velocity_position_pub.publish(velocity_message)
	
	
	
	if i==0:   
		# guarda dados
		#dados=np.matrix([[t,x,x_r,y,y_r,z,z_r,gui,gui_r,vx,vx_r,vy,vy_r,vz,vz_r,vgui,vgui_r]])
		dados=np.matrix([[t,x,x_r,y,y_r,z,z_r,theta,theta_r,vc_x,vx_r,vc_y,vy_r,vc_z,vz_r,vc_theta,vtheta_r,vx,vy,vz,vtheta]])
		np.savetxt(path+nome, dados, delimiter = ",")
		
            
	else:
		#dados=np.append(dados,[[t,x,x_r,y,y_r,z,z_r,gui,gui_r,vx,vx_r,vy,vy_r,vz,vz_r,vgui,vgui_r]],axis=0)
		dados=np.append(dados,[[t,x,x_r,y,y_r,z,z_r,theta,theta_r,vc_x,vx_r,vc_y,vy_r,vc_z,vz_r,vc_theta,vtheta_r,vx,vy,vz,vtheta]],axis=0)
		np.savetxt(path+nome, dados, delimiter = ",")
		
	return xp, yp, zp, thetap
		
		 


## --- NODE --- ##

global periodo, Ts, i, Voltas, Passos
global Ax, Ay, Az
global Kp_x, Kd_x, Kp_y, Kd_y
global Kp_z, Kd_z, Kp_theta, Kd_theta
global wx, wy, wz
global M
global f1x, f2x, f1y, f2y, f1z, f2z, f1theta, f2theta
global nome, path



# f1x=2.78
# f2x=0.125

f1x=8.0
f2x=0.1

f1y=12.0
f2y=0.11

f1z=4.88
f2z=4.74


f1theta=3.31
#f2theta=5.89
f2theta=4


#f1y=9
#f2y=0.12
#f2y=0.178

#f1x=8
#f2x=0.10
#f2x=0.125

#f1z=6
#f2z=3.5


lamb=0.2
rho=20

# Q=diag(lamb,1)
# R =rho

Kd_x=-(2*f2x-sqrt(4*f2x**2*rho**2+4*(lamb+2*sqrt(rho)/f1x)*(f1x**2/rho)))/(2*f1x)
Kp_x=1/sqrt(rho)


Kd_y=-(2*f2y-sqrt(4*f2y**2*rho**2+4*(lamb+2*sqrt(rho)/f1y)*(f1y**2/rho)))/(2*f1y)
Kp_y=1/sqrt(rho)

lamb=0.1
rho=1

Kd_z=-(2*f2z-sqrt(4*f2z**2*rho**2+4*(lamb+2*sqrt(rho)/f1z)*(f1z**2/rho)))/(2*f1z)
Kp_z=1/sqrt(rho)

lamb=0.1
rho=1


Kd_theta=-(2*f2theta-sqrt(4*f2theta**2*rho**2+4*(lamb+2*sqrt(rho)/f1theta)*(f1theta**2/rho)))/(2*f1theta)
Kp_theta=1/sqrt(rho)



periodo=40 # período da volta 

wx=2*pi/periodo
wy=4*pi/periodo
wz=2*pi/periodo
wt=wx

# Ax=0.5
# Ay=0.3
# Az=0.3
# At=0.3

Ax=0.6
Ay=0.4
Az=0.4
At=1.0

Taxa=20 # Opera à Taxa [Hz] 

Ts=1/Taxa # Período de amostragem

Voltas=2 # Número de voltas

i=0

M=3 # Horizonte do filtro média movel

Passos=int(periodo*Taxa)


rospy.init_node('Bebop_2_Trajectory_Tracking')


rate = rospy.Rate(Taxa) 

velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
 # subscriber = rospy.Subscriber('/natnet_ros/corpo1/pose', Odometry, get_pose)
subscriber = rospy.Subscriber('/adjusted_odometry', CustomOdometry, get_pose)


os.system("rostopic pub --once /bebop/takeoff std_msgs/Empty")


tecla = input("Aperte a tecla  'Enter' para controle de trajetória ou '0+Enter' para pousar")

if tecla == "":
    print("Modo de controle ativado")
elif tecla==0:
    os.system("rostopic pub --once /bebop/takeoff std_msgs/Empty")
    i=Voltas*Passos+1
else:
    print("Tecla não reconhecida. Modo de pouso ativado.")
    os.system("rostopic pub --once /bebop/takeoff std_msgs/Empty")
    i=Voltas*Passos+1


xp=0
yp=0
zp=0
thetap=0

data_hora = datetime.now()
data_hora_texto = data_hora.strftime("dia_%d_%m_%Y_hora_%H_%M")

nome ="bebop_"+data_hora_texto+".csv"
path = "/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/"

while not rospy.is_shutdown() and i < Voltas*Passos+1+M:           
	#inicio = time.time()
	xp,yp,zp,thetap=controle(xp,yp,zp,thetap)
	#fim = time.time()
	#print(fim-inicio)
	i=i+1 
	rate.sleep()

velocity_message.linear.x = 0
velocity_message.linear.y = 0	
velocity_message.linear.z = 0
velocity_message.angular.z = 0	
velocity_position_pub.publish(velocity_message)

os.system("rostopic pub --once /bebop/land std_msgs/Empty")

print('Tecle Ctrl+C para finalizar o programa.')
rospy.spin()    
