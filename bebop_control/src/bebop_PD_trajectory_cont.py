#!/usr/bin/env python3

import rospy
import time ## controlling the time 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_msgs.msg import CustomOdometry
import numpy as np
from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Vector3
import os
#from std_msgs.msg import Empty
from datetime import datetime
from math import pi, atan2, sin, cos, sqrt
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

def controle(xp,yp,zp,thetap,vc_xp,vc_yp,vc_zp,vc_thetap):

	'''
	Input:
	(x,y,z) = vector of position

	Task:
	publish velocity messages during the time linked to position
	'''
	t=i*Ts
	
	global dados
	
	if  i==0:
		global x0,y0,z0
	
		x0=current_pose[0]
		y0=current_pose[1]
		z0=current_pose[2]
		
	#elif i<M:
	#	x0=0
	#	y0=0
	#	z0=1
	
	
	
	# Calcula referências	
	x_r=x0+Ax*sin(wx*(i)*Ts)
	x_rf=x0+Ax*sin(wx*(i+1)*Ts)
	x_rff=x0+Ax*sin(wx*(i+2)*Ts)
	
	dx_rf=(x_rff-x_rf)/Ts
	dx_r=(x_rf-x_r)/Ts
	
	ax_r=(dx_rf-dx_r)/Ts
	
	y_r=y0+Ay*sin(wy*(i)*Ts)
	y_rf=y0+Ay*sin(wy*(i+1)*Ts)
	y_rff=y0+Ay*sin(wy*(i+2)*Ts)
	
	dy_rf=(y_rff-y_rf)/Ts
	dy_r=(y_rf-y_r)/Ts
	
	ay_r=(dy_rf-dy_r)/Ts
	
	z_r=z0+Az*sin(wz*(i)*Ts)
	z_rf=z0+Az*sin(wz*(i+1)*Ts)
	z_rff=z0+Az*sin(wz*(i+2)*Ts)
	
	dz_rf=(z_rff-z_rf)/Ts
	dz_r=(z_rf-z_r)/Ts
	
	az_r=(dz_rf-dz_r)/Ts
	
	theta_r=At*sin(wt*(i)*Ts)
	theta_rf=At*sin(wt*(i+1)*Ts)
	theta_rff=At*sin(wt*(i+2)*Ts)
	
	dtheta_rf=(theta_rff-theta_rf)/Ts
	dtheta_r=(theta_rf-theta_r)/Ts
	
	atheta_r=(dtheta_rf-dtheta_r)/Ts
	
		
	
	# feedforward de controle 
	vff_x=(ax_r+f2x*dx_r)/f1x
	vff_y=(ay_r+f2y*dy_r)/f1y
	vff_z=(az_r+f2z*dz_r)/f1z
	vff_theta=(atheta_r+f2theta*dtheta_r)/f1theta
	
	# Referência passada para observador
	x_rp=x0+Ax*sin(wx*(i-1)*Ts)
	y_rp=y0+Ay*sin(wy*(i-1)*Ts)
	z_rp=z0+Az*sin(wz*(i-1)*Ts)
	theta_rp=At*sin(wt*(i-1)*Ts)
	
	x=current_pose[0]
	y=current_pose[1]
	z=current_pose[2]
	theta=current_euler[2]
	
	# Inicializa valores passados
	if i==0:
	
		global zx_ob, zy_ob, zz_ob, ztheta_ob, wx_ob, wy_ob, wz_ob, wtheta_ob, vff_xp, vff_yp, vff_zp, vff_thetap
		
		xp=x
		yp=y
		zp=z
		thetap=theta
		
		zx_ob=-L*x		
		zy_ob=-L*y		
		zz_ob=-L*z		
		ztheta_ob=-L*theta
		
		wx_ob=-L*(x-x_r)		
		wy_ob=-L*(y-y_r)		
		wz_ob=-L*(z-z_r)
		wtheta_ob=-L*(theta-theta_r)
		
	
		
	else:
	
		zx_ob=(1+alpha_x*Ts)*zx_ob+Ts*f1x*vc_xp+Ts*alpha_x*L*xp
		zy_ob=(1+alpha_y*Ts)*zy_ob+Ts*f1y*vc_yp+Ts*alpha_y*L*yp
		zz_ob=(1+alpha_z*Ts)*zz_ob+Ts*f1z*vc_zp+Ts*alpha_z*L*zp
		ztheta_ob=(1+alpha_theta*Ts)*ztheta_ob+Ts*f1theta*vc_thetap+Ts*alpha_theta*L*thetap
	
		
		
		
		wx_ob=(1+alpha_x*Ts)*wx_ob+Ts*f1x*(vc_xp-vff_xp)+Ts*alpha_x*L*(xp-x_rp)
		wy_ob=(1+alpha_y*Ts)*wy_ob+Ts*f1y*(vc_yp-vff_yp)+Ts*alpha_y*L*(yp-y_rp)
		wz_ob=(1+alpha_z*Ts)*wz_ob+Ts*f1z*(vc_zp-vff_zp)+Ts*alpha_z*L*(zp-z_rp)
		wtheta_ob=(1+alpha_theta*Ts)*wtheta_ob+Ts*f1theta*(vc_thetap-vff_thetap)+Ts*alpha_theta*L*(thetap-theta_rp)
		
	vff_xp=vff_x
	vff_yp=vff_y
	vff_zp=vff_z
	vff_thetap=vff_theta
	# Estima velocidade (estimador de ordem reduzida)
	
	vx_ob=zx_ob+L*xp		
	vy_ob=zy_ob+L*yp
	vz_ob=zz_ob+L*zp
	vtheta_ob=ztheta_ob+L*thetap
	
	# Estima erro velocidade (estimador de ordem reduzida)
	
	
	
	evx_ob=wx_ob+L*(xp-x_rp)
	evy_ob=wy_ob+L*(yp-y_rp)
	evz_ob=wz_ob+L*(zp-z_rp)
	evtheta_ob=wtheta_ob+L*(thetap-theta_rp)
		
	
	# Estima a velocidade (Euler)	
	vx=(x-xp)/Ts	
	vy=(y-yp)/Ts	
	vz=(z-zp)/Ts	
	vtheta=(theta-thetap)/Ts

	# velocidades de referência
	vx_r=(x_rf-x_r)/(Ts)
	vy_r=(y_rf-y_r)/(Ts)
	vz_r=(z_rf-z_r)/(Ts)
	vtheta_r=(theta_rf-theta_r)/(Ts)

	e_vx = vx - vx_r
	e_vy = vy - vy_r
	e_vz = vz - vz_r
	e_vtheta = vtheta - vtheta_r

	e_x = x-x_r
	e_y = y-y_r
	e_z = z-z_r
	e_theta = theta - theta_r
	
	# Filtro média móvel
	
	if i>=1:
	
		Mn=min(i+1,M) 
		soma_x=e_vx
		soma_y=e_vy
		soma_z=e_vz
		soma_theta=e_vtheta

		soma_px=e_x
		soma_py=e_y
		soma_pz=e_z
		soma_ptheta=e_theta
	
		Nx=np.array(dados[:,25]) 
		Ny=np.array(dados[:,26]) 
		Nz=np.array(dados[:,27]) 
		Ntheta=np.array(dados[:,28]) 

		Npx=np.array(dados[:,29]) 
		Npy=np.array(dados[:,30]) 
		Npz=np.array(dados[:,31]) 
		Nptheta=np.array(dados[:,32]) 


		
		num_size=Nx.size-1
		
		
	
		for c in range(0,Mn-1):
			soma_x=soma_x+Nx.item(num_size-1-c)
			soma_y=soma_y+Ny.item(num_size-1-c)
			soma_z=soma_z+Nz.item(num_size-1-c)
			soma_theta=soma_theta+Ntheta.item(num_size-1-c)

			soma_px = soma_px+Npx.item(num_size-1-c)
			soma_py = soma_py+Npy.item(num_size-1-c)
			soma_pz = soma_pz+Npz.item(num_size-1-c)
			soma_ptheta = soma_ptheta+Nptheta.item(num_size-1-c)
	
			
			
		ex_filt=soma_x/Mn
		ey_filt=soma_y/Mn
		ez_filt=soma_z/Mn
		etheta_filt=soma_theta/Mn

		e_x_filt = soma_px/Mn
		e_y_filt = soma_py/Mn
		e_z_filt = soma_pz/Mn
		e_theta_filt = soma_ptheta/Mn
		
		# vx_filt=vx_ob
		# vy_filt=vy_ob
		# vz_filt=vz_ob
		# vtheta_filt=vtheta_ob
		
	       
		
	else:
		ex_filt=0
		ey_filt=0
		ez_filt=0
		etheta_filt=0

		e_x_filt = 0
		e_y_filt = 0
		e_z_filt = 0
		e_theta_filt = 0
		
	
	
	# Armazena o valor passado
	xp=x
	yp=y
	zp=z
	thetap=theta
	
	
	Ri=np.matrix([[cos(theta), sin(theta)],[-sin(theta), cos(theta)]])
	

	
	Kp=np.array([[-Kp_x, 0],[0, -Kp_y]])
	Kd=np.array([[-Kd_x, 0],[0, -Kd_y]])
	
	
	
	
  
	# vc_z=-Kp_z*(z-z_r)-Kd_z*evz_ob+vff_z
	# vc_theta=-Kp_theta*(theta-theta_r)-Kd_theta*evtheta_ob+vff_theta	
	# VXY=np.array([[evx_ob],[evy_ob]])
	
	
	#vc_z=-Kp_z*(e_z_filt)-Kd_z*(ez_filt)+vff_z
	vc_z=-Kp_z*(e_z_filt)+vff_z
	vc_theta=-Kp_theta*(e_theta_filt)-Kd_theta*(etheta_filt)+vff_theta	

	XY=np.array([[e_x_filt],[e_y_filt]])
	VXY=np.array([[ex_filt],[ey_filt]])

		
	Vff=np.array([[vff_x],[vff_y]])
	
	Vc=Kp*Ri*XY+Kd*Ri*VXY+Ri*Vff	
	
	vc_x=Vc.item(0)
	
	vc_y=Vc.item(1)
	
	
	vc_xp=vc_x
	vc_yp=vc_y
	vc_zp=vc_z
	vc_thetap=vc_theta
    
	
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
		vrob_x=vsat
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
	
	M_dados=[[t,x,x_r,y,y_r,z,z_r,theta,theta_r,
			vc_x,vx_r,vc_y,vy_r,vc_z,vz_r,vc_theta,
			vtheta_r,vx,vy,vz,vtheta,vx_ob,vy_ob,vz_ob,
			vtheta_ob,e_vx,e_vy,e_vz,e_vtheta,e_x,e_y,e_z,e_theta]]

	if i==0:   

		dados=np.matrix(M_dados)

		
            
	else:
	
		dados=np.append(dados,M_dados,axis=0)
		np.savetxt(path+nome+"_"+str(f1x)+"_"+str(f2x)+"_"+str(f1y)+"_"+str(f2y)+"_"+str(f1z)+"_"+str(f2z)+
		"_"+str(f1theta)+"_"+str(f2theta)+"_"+str(lamb_x)+"_"+str(rho_x)+"_"+str(lamb_y)+"_"+str(rho_y)+
		"_"+str(lamb_z)+"_"+str(rho_z)+"_"+str(lamb_w)+"_"+str(rho_w)+"_"+str(Ts)+"_"+str(M)+".csv",dados, delimiter = ",")

		
	return xp, yp, zp, thetap, vc_xp, vc_yp, vc_zp, vc_thetap
		
		 

print("CÓDIGO 115")

## --- NODE --- ##

global periodo, Ts, i, Voltas, Passos
global Ax, Ay, Az
global Kp_x, Kd_x, Kp_y, Kd_y
global Kp_z, Kd_z, Kp_theta, Kd_theta
global wx, wy, wz
global M, L, alpha_x, alpha_y
global f1x, f2x, f1y, f2y, f1z, f2z, f1theta, f2theta
global nome, path, Axis
global f1x,f2x,f1y,f2y,f1z,f2z,f1theta,f2theta
global lamb_x,rho_x,lamb_y,rho_y,lamb_z,rho_z,lamb_w,rho_w

## Modelo ##

f1x=2.78
f2x=0.125


f1y=2.85
f2y=0.178

f1z=5.5
f2z=4.74

#f1theta=3.31
f1theta=3.0
f2theta=2

## Trajetória ##

Voltas=2
periodo=15

M=17
L=5


Ax=0.4
Ay=0.4
Az=0.3
At=0.4

## Frequência ##

wx=2*pi/periodo
wy=2*pi/periodo
wz=2*pi/periodo
wt=2*pi/periodo

## Controlador ##

Taxa=40 # Opera à Taxa [Hz] 

lamb_x=0.2
rho_x=30

lamb_y=2
rho_y=30

lamb_z=0.07
rho_z=10

lamb_w=0.02
rho_w=7

#################

Kd_x=-(2*f2x*rho_x-sqrt(4*f2x**2*rho_x**2+4*(lamb_x+(2*sqrt(rho_x)/f1x))*(f1x**2*rho_x)))/(2*f1x*rho_x)
Kp_x=1/sqrt(rho_x)

Kd_y=-(2*f2y*rho_y-sqrt(4*f2y**2*rho_y**2+4*(lamb_y+(2*sqrt(rho_y)/f1y))*(f1y**2*rho_y)))/(2*f1y*rho_y)
Kp_y=1/sqrt(rho_y)


#Kd_z=-(2*f2z*rho_z-sqrt(4*f2z**2*rho_z**2+4*(lamb_z+(2*sqrt(rho_z)/f1z))*(f1z**2*rho_z)))/(2*f1z*rho_z)
Kd_z=0
#Kp_z=1/sqrt(rho_z)
Kp_z=0.8


Kd_theta=-(2*f2theta*rho_w-sqrt(4*f2theta**2*rho_w**2+4*(lamb_w+(2*sqrt(rho_w)/f1theta))*(f1theta**2*rho_w)))/(2*f1theta*rho_w)
Kp_theta=1/sqrt(rho_w)


# print(Kp_x)
# print(Kp_y)
# print(Kp_z)
# print(Kp_theta)

# print(Kd_x)
# print(Kd_y)
# print(Kd_z)
# print(Kd_theta)

# Parâmetros do observador

alpha_x=-f2x-L
alpha_y=-f2y-L
alpha_z=-f2z-L
alpha_theta=-f2theta-L

Ts=1/Taxa # Período de amostragem

i=0

Passos=int(periodo*Taxa)


rospy.init_node('Bebop_2_Trajectory_Tracking')


rate = rospy.Rate(Taxa) 

velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

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
vc_xp=0
vc_yp=0
vc_zp=0
vc_thetap=0

Axis = rospy.get_param('Axis')

data_hora = datetime.now()
data_hora_texto = data_hora.strftime("dia_%d_%m_%Y_hora_%H_%M")
nome="bebop_"+data_hora_texto+'_'+Axis+'_'
path = "/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/"



while not rospy.is_shutdown() and i < Voltas*Passos+1+M:           

	xp,yp,zp,thetap,vc_xp,vc_yp,vc_zp,vc_thetap=controle(xp,yp,zp,thetap,vc_xp,vc_yp,vc_zp,vc_thetap)

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