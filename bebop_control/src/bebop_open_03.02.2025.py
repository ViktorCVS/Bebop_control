#!/usr/bin/env python3

import rospy
import time ## controlling the time 
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty, Float64MultiArray
import numpy as np
from tf.transformations import euler_from_quaternion
from datetime import datetime
from math import pi, atan2, sin, cos, sqrt, tanh
from pynput import keyboard
import os


# Variáveis globais
velocity_message = Twist()
current_pose = None
current_euler = None
dados = None



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
    
def on_key_press(key):

	
	global flag_abort	
	flag_abort=0
		

	if key==keyboard.Key.space:
		print("Missão abortada pela barra de espaço.")
		flag_abort=1

# Create a listener and setup our call backs
keyboard_listener = keyboard.Listener(on_press=on_key_press)        


def controle():

	'''
	Input:
	(x,y,z) = vector of position

	Task:
	publish velocity messages during the time linked to position
	'''

	t=i*Ts
	
	x = current_pose.position.x
	y = current_pose.position.y
	z = current_pose.position.z
	rotx = current_euler[0]
	roty = current_euler[1]

	rotz = current_euler[2]
	
	roll=-rotx
	pitch=roty
	yaw=rotz
	
	global dados, Kp_x, Kp_y
	
	if  i==0:
		global x0,y0,z0,Xxp,Yyp,Zzp,Yyawp,usmcp,up, Wx, Wy, Wz, Wyaw, flag_opt
	
		x0=x
		y0=y
		z0=z
		
		Xxp=np.array([[0],[0]])
		Yyp=np.array([[0],[0]])
		Zzp=np.array([[0],[0]])
		Yyawp=np.array([[0],[0]])
		up=np.array([[0],[0],[0],[0]])
		usmcp=np.array([[0],[0],[0],[0]])
		
		Wx=np.array([[0],[0]])
		Wy=np.array([[0],[0]])
		Wz=np.array([[0],[0]])
		Wyaw=np.array([[0],[0]])
		
		flag_opt=0
	
	
	
	# Calcula referências	
	x_r=x0+Ax*sin(wx*(i)*Ts)+xf
	x_rf=x0+Ax*sin(wx*(i+1)*Ts)+xf
	x_rff=x0+Ax*sin(wx*(i+2)*Ts)+xf
	
	#x_r=x0+Ax*cos(wx*(i)*Ts)
	#x_rf=x0+Ax*cos(wx*(i+1)*Ts)
	#x_rff=x0+Ax*cos(wx*(i+2)*Ts)
	
	dx_rf=(x_rff-x_rf)/Ts
	dx_r=(x_rf-x_r)/Ts
	
	ax_r=(dx_rf-dx_r)/Ts
	
	y_r=y0+Ay*sin(wy*(i)*Ts)+yf
	y_rf=y0+Ay*sin(wy*(i+1)*Ts)+yf
	y_rff=y0+Ay*sin(wy*(i+2)*Ts)+yf
	
	#y_r=y0+Ax*sin(wx*(i)*Ts)
	#y_rf=y0+Ax*sin(wx*(i+1)*Ts)
	#y_rff=y0+Ax*sin(wx*(i+2)*Ts)
	
	
	
	dy_rf=(y_rff-y_rf)/Ts
	dy_r=(y_rf-y_r)/Ts
	
	ay_r=(dy_rf-dy_r)/Ts
	
	z_r=z0+Az*sin(wz*(i)*Ts)+zf
	z_rf=z0+Az*sin(wz*(i+1)*Ts)+zf
	z_rff=z0+Az*sin(wz*(i+2)*Ts)+zf
	
	#z_r=z0+Ax*0.5*sin(wz*(i)*Ts)
	#z_rf=z0+Ax*0.5*sin(wz*(i+1)*Ts)
	#z_rff=z0+Ax*0.5*sin(wz*(i+2)*Ts)
	
	dz_rf=(z_rff-z_rf)/Ts
	dz_r=(z_rf-z_r)/Ts
	
	az_r=(dz_rf-dz_r)/Ts
	
	yaw_r=At*sin(wt*(i)*Ts)+yawf
	yaw_rf=At*sin(wt*(i+1)*Ts)+yawf
	yaw_rff=At*sin(wt*(i+2)*Ts)+yawf
	
	
	dyaw_rf=(yaw_rff-yaw_rf)/Ts
	dyaw_r=(yaw_rf-yaw_r)/Ts
	
	ayaw_r=(dyaw_rf-dyaw_r)/Ts
	
	
	# velocidades de referência
	vx_r=(x_rf-x_r)/(Ts)
	vy_r=(y_rf-y_r)/(Ts)
	vz_r=(z_rf-z_r)/(Ts)
	vyaw_r=(yaw_rf-yaw_r)/(Ts)
		
	
	# feedforward de controle 
	vff_x=(ax_r+f2x*dx_r)/f1x
	vff_y=(ay_r+f2y*dy_r)/f1y
	vff_z=(az_r+f2z*dz_r)/f1z
	vff_yaw=(ayaw_r+f2yaw*dyaw_r)/f1yaw
	
	
	
	# Inicializa dados - cascata
    
	if i==0:
		global roll_p, picth_p, vx_rob_p, vy_rob_p, e_roll_p, e_pitch_p, vbx_p, vby_p, evbx_p, evby_p, evx_est_p, evy_est_p
		roll_p=roll
		pitch_p=pitch
		vx_rob_p=0
		vy_rob_p=0
		vbx_p=0
		vby_p=0
		evbx_p=0
		evby_p=0
		evx_est_p=0
		evy_est_p=0
	
	else:
		roll_p=roll
		pitch_p=pitch
	
	# Inicializa dados - controle
	if i==0:
	
		global xp,yp,zp,yawp,dxp,dyp,dzp,dyawp
		
		xp=x
		yp=y
		zp=z
		yawp=yaw
		
		dxp=0
		dyp=0
		dzp=0
		dyawp=0
		
					
	
	# Estima a velocidade (Euler)	
	vx=(x-xp)/Ts	
	vy=(y-yp)/Ts	
	vz=(z-zp)/Ts	
	vyaw=(yaw-yawp)/Ts
	
	# Avalia travamento dos dados do Optitrack
	var=abs(vx)+abs(vy)+abs(vz)+abs(vyaw)
	
	if var<1e-15 and i>1:
		flag_opt=1
	
	# Calcula erro
	
	ex=x-x_r
	ey=y-y_r
	ez=z-z_r
	eyaw=yaw-yaw_r
	
	
	# Inicializa dados - Kalman
	
	if i==0:	
		
		
		global vx_k, vy_k, vz_k, vyaw_k,Pk
	
		
		vx_k=np.array([[x],[0]])
		vy_k=np.array([[y],[0]])
		vz_k=np.array([[z],[0]])
		vyaw_k=np.array([[yaw],[0]])
		
		
		Pk=Pk0
	
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
	
	est_vel_msg = Float64MultiArray()

	Ri=np.matrix([[cos(yaw), sin(yaw)],[-sin(yaw), cos(yaw)]])

	vx_est=vx_k.item(1)
	vy_est=vy_k.item(1)
	vz_est=vz_k.item(1)
	vyaw_est=vyaw_k.item(1)

	est_vel_msg.data = [vx_est,vy_est, vz_est, vyaw_est]
	estimated_velocity_pub.publish(est_vel_msg)
		
	evx_est=vx_est-vx_r
	evy_est=vy_est-vy_r
	evz_est=vz_est-vz_r
	evyaw_est=vyaw_est-vyaw_r
	
	# Lei de controle (x e y)
	
	Ri=np.matrix([[cos(yaw), sin(yaw)],[-sin(yaw), cos(yaw)]])
	
	# EXY=np.array([[x-x_r],[y-y_r]])
	# EVXY=np.array([[evx_est],[evy_est]])
	
	# Kp=np.matrix([[-Kp_x, 0],[0, -Kp_y]])
	# Kd=np.matrix([[-Kd_x, 0],[0, -Kd_y]])
	
	# Vff=np.array([[vff_x],[vff_y]])

	#####################################################################
	################ BLOCO 03.02.2025 ###################################
	#####################################################################

	gx1 = 0.17081
	gx2 = 0.169187305

	gy1 = 0.18867
	gy2 = 0.186330492

	vx_r = 0.0
	vy_r = 1.0

	if i==0:
		print(vx_r)
		print(vy_r)

	evx_est=vx_r-vx_est
	evy_est=vy_r-vy_est


	vx_rob = vx_rob_p + gx1*evx_est - gx2*evx_est_p
	vy_rob = vy_rob_p + gy1*evy_est - gy2*evy_est_p


	vx_rob_p = vx_rob
	vy_rob_p = vy_rob

	evx_est_p = evx_est
	evy_est_p = evy_est

	# VXY_ROB = np.array([[vx_rob],[vy_rob]])

	# V_transformado = Ri*VXY_ROB

	# vx_robx = V_transformado.item(0)
	# vy_roby = V_transformado.item(1)

	vx_robx = vx_rob
	vy_roby = vy_rob

	#####################################################################
	#####################################################################
	#####################################################################

	#Vw=Kp*EXY+Kd*EVXY+Vff	

	#Vc=Ri*Vw
		
	#vc_x=Vc.item(0)
	
	#vc_y=Vc.item(1)
	
	# Lei de Controle (z e yaw)
		
	vc_z=-Kp_z*(z-z_r)-Kd_z*(evz_est)+vff_z
		
	
	vc_yaw=-Kp_yaw*(yaw-yaw_r)-Kd_yaw*(evyaw_est)+vff_yaw
	
		
	
	# Armazena valores passados
	xp=x
	yp=y
	zp=z
	yawp=yaw
		
 	
	# Erro ponderado (PI ISA)
	# e_pitch=beta_x*vc_x-pitch
	# e_roll=beta_y*vc_y-roll
	
	# if i==0: # Complenta inicialização do cascata
	
	# 	e_pitch_p=e_pitch
	# 	e_roll_p=e_roll
	
	# VXY=np.array([[vx_est],[vy_est]])
	
	# Vb=Ri*VXY
	
	# vbx=Vb.item(0)
	# vby=Vb.item(1)	
        
	# Ki_x=10
	
	# Ki_y=10
	
	# Kp_x=1
	
	# Kp_y=1
	
	
	# c_x=0
	
	# c_y=0
	
	
	# evbx=c_x*vc_x-vbx
	
	# evby=c_x*vc_y-vby
        
	#vx_rob = vx_rob_p+Ki_x*Ts*(vc_x-vbx)+Kp_x*(evbx-evbx_p)
	
    #vy_rob = vy_rob_p+Ki_y*Ts*(vc_y-vby)+Kp_y*(evby-evby_p)

	
	# e_pitch_p=e_pitch
	# e_roll_p=e_roll
	roll_p=roll
	pitch_p=pitch
	# vx_rob_p=vx_rob
	# vy_rob_p=vy_rob
	
	# vbx_p=vbx
	# vby_p=vby
	
	# evbx_p=evbx
	# evby_p=evby
	
	vrob_x=vx_robx 
	vrob_y=vy_roby
	vrob_z=vc_z
	vrob_yaw=vc_yaw
	
	
	
	# Finaliza com controle nulo
	if i>= Voltas*Passos:
		vrob_x=0
		vrob_y=0
		vrob_z=0
		vrob_yaw=0
	
	
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
		
	if flag_abort==1 or flag_opt==1:
	 
		vrob_x=0
		vrob_y=0
		vrob_z=0
		vrob_yaw=0
				
  
	
	# velocity_message.linear.x = vrob_x
	# velocity_message.linear.y = vrob_y	
	# velocity_message.linear.z = vrob_z
	# velocity_message.angular.z = vrob_yaw

	velocity_message.linear.x = 0
	velocity_message.linear.y = 0.05
	velocity_message.linear.z = 0
	velocity_message.angular.z = 0

	
	velocity_position_pub.publish(velocity_message)
	
	
	
	
	data_row=[t,x,x_r,y,y_r,z,z_r,vx,vx_r,vy,vy_r,vz,vz_r,vrob_x,vrob_y,vrob_z,vrob_yaw,rotx,roty,rotz,vx_est,vy_est,vz_est,vyaw_est,yaw_r,vyaw_r]	
	
	if i==0:   
		# guarda dados
		dados=np.matrix(data_row)
		
            
	else:
	
		dados=np.append(dados,[data_row],axis=0)
		
	
		
# Inicialização do Nó
rospy.init_node('Bebop_2_identificacao')		 


## --- NODE --- ##

global periodo, Ts, i, Voltas, Passos, vsat, M
global Ax, Ay, Az
global Kp_x, Kd_x, Kp_y, Kd_y
global Kp_z, Kd_z, Kp_yaw, Kd_yaw
global wx, wy, wz
global xf, yf, zf, yawf
global f1x, f2x, f1y, f2y, f1z, f2z, f1yaw, f2yaw
global Ak, Hk, Pk0, Qk, Rk

data_hora = datetime.now()
data_hora_texto = data_hora.strftime("dia_%d_%m_%Y_hora_%H_%M")

nome="bebop_PID_"+data_hora_texto+"_y.csv"


# Ativa / desativa cascata

#flag_cascata=0 # sem cascata
flag_cascata=1 # com cascata




# Parâmetros do controlador

Taxa=20 # Opera à Taxa [Hz] 

Ts=1/Taxa # Período de amostragem


vsat=0.5

# Parâmetros do filtro de Kalman

Ak=np.matrix([[1, Ts],[0, 1]])
Hk=np.matrix([[1 , 0]])
Pk0=np.matrix([[1, 0],[0, 1]])
Qk=np.matrix([[(Ts**4)/4, 0],[0, Ts**2]])
Rk=Ts**3



# Sintonia cascata
global Kcm_x, Kcr_x, Kcm_y, Kcr_y, Ki_x, Ki_y, beta_x, beta_y

# Sintonia I+P (24/10)
#antigo
#Kcm_x=1.87 # ganho proporcional pitch
#Kcm_y=1.87  # ganho proporcional roll

#Ki_x=9.37  # ganho proporcional pitch
#Ki_y=9.37 # ganho proporcional roll

Kcm_x=1 # ganho proporcional pitch
Kcm_y=1  # ganho proporcional roll

Ki_x=3  # ganho proporcional pitch  0.2
Ki_y=3 # ganho proporcional roll

beta_x=0.1 # pondera referência pitch
beta_y=0.1 # pondera referência roll


# ganhos do modelo em malha aberta (normaliza ganho)
ganho_pitch=0.426
ganho_roll=0.431


# Nomaliza ganho cascata
Kcr_x=ganho_pitch
Kcr_y=ganho_roll


#cx=11.2
cx=10
f2x=0.44
f1x=cx*f2x

#cy=7.5
cy=10
f2y=0.65
f1y=cy*f2y


cz=1.13
f2z=3.7
f1z=cz*f2z

cyaw=1.88
f2yaw=3.8
f1yaw=cyaw*f2yaw



# LQR 
# Q=diag(lamb,1)
# R =rho

#lamb=0.01
#rho=1

#Kd_x=-(2*f2x*rho-sqrt(4*f2x**2*rho**2+4*(lamb+(2*sqrt(rho)/f1x))*(f1x**2*rho)))/(2*f1x*rho)
#Kp_x=1/sqrt(rho)

#Kd_y=-(2*f2y*rho-sqrt(4*f2y**2*rho**2+4*(lamb+(2*sqrt(rho)/f1y))*(f1y**2*rho)))/(2*f1y*rho)
#Kp_y=1/sqrt(rho)

#lamb=0.1
#rho=1

#Kd_z=-(2*f2z*rho-sqrt(4*f2z**2*rho**2+4*(lamb+(2*sqrt(rho)/f1z))*(f1z**2*rho)))/(2*f1z*rho)
#Kp_z=1/sqrt(rho)

#lamb=0.01
#rho=0.1

#Kd_yaw=-(2*f2yaw*rho-sqrt(4*f2yaw**2*rho**2+4*(lamb+(2*sqrt(rho)/f1yaw))*(f1yaw**2*rho)))/(2*f1yaw*rho)
#Kp_yaw=1/sqrt(rho)


# Aloca polos

fator_robusto=1.5  # margem para evitar polos complexos

# pd=0.9 # módulo do polo desejado (x e y)

# Kd_x=(2*pd-f2x)/f1x*fator_robusto
# Kp_x=(pd*pd)/f1x*fator_robusto


# Kd_y=(2*pd-f2y)/f1y*fator_robusto
# Kp_y=(pd*pd)/f1y*fator_robusto

pd=1.8 # módulo do polo desejado (z)

Kd_z=(2*pd-f2x)/f1z*fator_robusto
Kp_z=(pd*pd)/f1z*fator_robusto

pd=1.5 # módulo do polo de sejado (yaw)

Kd_yaw=(2*pd-f2yaw)/f1yaw*fator_robusto
Kp_yaw=(pd*pd)/f1yaw*fator_robusto


# Parâmetros da trajetória


periodo=10 # período da volta 


Voltas=2 # Número de voltas


wx=2*pi/periodo
wy=4*pi/periodo
wz=2*pi/periodo
wt=wx

Ax=0.3
Ay=0.0
Az=0
At=0

# Altera referência (set-point constante)

flag_constante=1 # P/ referência constante (flag_constante=1)

if flag_constante==1:
	# Referência constante
	wx=0 
	wy=0
	wz=0
	wt=0
	
	xf=0    # set-point x
	yf=0    # set-point y	
	zf=0    # set-point z
	yawf=0  # set-point yaw

else:
	# Para gerar leminscata
	xf=0
	yf=0
	zf=0
	yawf=0





# Incializa lei de controle

i=0


Passos=int(periodo*Taxa)


# Inicialização dos tópicos
rospy.Subscriber('/natnet_ros/Bebop2/pose', PoseStamped, get_pose)
velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
estimated_velocity_pub = rospy.Publisher('/bebop/vel_est', Float64MultiArray, queue_size=10)
takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

rate = rospy.Rate(Taxa)
takeoff_msg = Empty()
land_msg = Empty()

# Decolagem
while takeoff_pub.get_num_connections() < 1:
    pass
takeoff_pub.publish(takeoff_msg)
print("Take off ok")



tecla = input("Aperte a tecla  'Enter' para controle de trajetória ou '0+Enter' para pousar")

if tecla == "":
    print("Modo de controle ativado")
elif tecla==0:
    
    # Pouso
	while land_pub.get_num_connections() < 1:
	    pass
	land_pub.publish(land_msg)
	print("Land ok")

	i=Voltas*Passos+1
else:
    
    # Pouso
	while land_pub.get_num_connections() < 1:
	    pass
	land_pub.publish(land_msg)
	print("Tecla não-reconhecida")
	i=Voltas*Passos+1


flag_abort=0
flag_opt=0
#keyboard_listener.start()


while not rospy.is_shutdown() and i < Voltas*Passos+1:           
	#inicio = time.time()
	controle()
	np.savetxt("/home/ubuntu/bebop_ws/src/Bebop_control/bebop_control/Bags/"+nome, dados, delimiter = ",")
	#fim = time.time()
	#print(fim-inicio)
	if flag_opt==1:
		i=Voltas*Passos
		print('Valor de medidas repetido')
	i=i+1 
	rate.sleep()

#keyboard_listener.stop()
#keyboard_listener.join()
velocity_message.linear.x = 0
velocity_message.linear.y = 0	
velocity_message.linear.z = 0
velocity_message.angular.z = 0	
velocity_position_pub.publish(velocity_message)

# Pouso
while land_pub.get_num_connections() < 1:
    pass
land_pub.publish(land_msg)
print("Land ok")

print('Tecle Ctrl+C para finalizar o programa.')
rospy.spin()    	
		
		