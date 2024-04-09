#!/usr/bin/env python3
import rospy
import math 
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x=0;y=0;theta=0
flag = 0                      #Bandera de Lectura y Control       
e_p = np.zeros([3, 1])        #Errores Globales
e_i = np.zeros([3, 1])
e_d = np.zeros([3, 1])
e_anterior = np.zeros([3, 1])
e_prima = np.zeros([3, 1])    

i = 5
time_plt = []
init_time = 0

def control():
    global x,y,theta
    global xd,yd
    global xp,yp
    global flag
    global e_p
    global e_i
    global e_d
    global e_anterior
    global e_prima

    P_1 = 1*np.eye(3); P_1 = np.asmatrix(P_1)
    Q_1 = 0.1*np.eye(3); Q_1 = np.asmatrix(Q_1)
    R_1 = 0.0001*np.eye(1); R_1 = np.asmatrix(R_1)
    W_1 = np.random.rand(3,1); W_1 = np.asmatrix(W_1)

    P_2 = 1*np.eye(3); P_2 = np.asmatrix(P_2)
    Q_2 = 0.1*np.eye(3); Q_2 = np.asmatrix(Q_2)
    R_2 = 0.0001*np.eye(1); R_2 = np.asmatrix(R_2)
    W_2 = np.random.rand(3,1); W_2 = np.asmatrix(W_2)

    P_3 = 1*np.eye(3); P_3 = np.asmatrix(P_3)
    Q_3 = 0.1*np.eye(3); Q_3 = np.asmatrix(Q_3)
    R_3 = 0.0001*np.eye(1); R_3 = np.asmatrix(R_3)
    W_3 = np.random.rand(3,1); W_3 = np.asmatrix(W_3)

    alpha_1 = 0.5
    alpha_3 = 0.5
    eta_1 = 0.01
    eta_3 = 0.01
    beta = 0.001
    D = 0.5
    #Remove Later
    X_2 = np.zeros([3,1]); X_2 = np.asmatrix(X_2)

    while True:
        if flag == 1:
            # Calculate Errors

            e_p[0] = xd - xp
            e_p[2] = yd - yp
            
            e_i = e_i + e_prima
            e_d = e_p - e_anterior
            e_anterior = e_p

            # Define SNPID Inputs

            X_1 = np.mat([np.float(e_p[0]),np.float(e_i[0]),np.float(e_d[0])]); X_1 = X_1.T
           #X_2 = np.mat([np.float(e_p[0]),np.float(e_i[0]),np.float(e_d[0])]); X_2 = X_2.T
            X_3 = np.mat([np.float(e_p[2]),np.float(e_i[2]),np.float(e_d[2])]); X_3 = X_3.T

            # Compute Weighted Sum of SNPID

            V_1 = W_1.T*X_1
            V_3 = W_3.T*X_3 

            # Anti-WindUp

            antiWindup(V_1,alpha_1,0)
            antiWindup(V_3,alpha_3,2)

            # Compute SNPID Output 

            U_1 = alpha_1*math.tanh(V_1)
            U_3 = alpha_3*math.tanh(V_3)
            
            # U_1 = float(V_1)
            # U_3 = float(V_3)

            # Saturate Output (If Necessary)

            U_1 = np.clip(U_1,-0.5,0.5)
            U_3 = np.clip(U_3,-0.5,0.5)

            # Compute Robot Velocities

            J = np.array([[np.cos(theta),-D*np.sin(theta)],
                          [np.sin(theta), D*np.cos(theta)]])
            vw = np.dot(np.linalg.inv(J),np.array([[U_1],[U_3]]))

            # Publish Velocities

            msg_out = Twist()
            msg_out.linear.x = vw[0]
            msg_out.linear.y = 0.0
            msg_out.linear.z = 0.0
            
            msg_out.angular.x = 0.0
            msg_out.angular.y = 0.0
            msg_out.angular.z = vw[1]

            pub.publish(msg_out)
            flag = 0

            # Display SNPID Data
            
            pose = [x,y,theta]
            control = [U_1,0,U_3]
            print(" ")
            print("{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}".format("Errors (x)","Errors (y)","Errors (θ)","Gains (x)","Gains (y)","Gains (θ)","Pose","Control"))
            for i in range(3):
                print("{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}".format(X_1[i,0],X_2[i,0],X_3[i,0],W_1[i,0],W_2[i,0],W_3[i,0],pose[i],control[i]))

            # EKF Training Algorithm
                
            H_1 = alpha_1*(1-float(np.tanh(V_1)**2))*X_1
            H_3 = alpha_3*(1-float(np.tanh(V_3)**2))*X_3

            # H_1 = X_1
            # H_3 = X_3

            K_1 = P_1*H_1/(R_1 + H_1.T*P_1*H_1)
            K_3 = P_3*H_3/(R_3 + H_3.T*P_3*H_3)

            W_1 = W_1 + eta_1*(K_1*e_p[0] - beta*float(e_p[0])*W_1)
            W_3 = W_3 + eta_3*(K_3*e_p[2] - beta*float(e_p[2])*W_3)

            P_1 = P_1 - K_1*H_1.T*P_1 + Q_1
            P_3 = P_3 - K_3*H_3.T*P_3 + Q_3
            

def antiWindup(V, alpha,j):
    global e_prima, e_p
    if abs(np.float(V)) > alpha:
        e_prima[j] = 0
    else:
        e_prima[j] = e_p[j]

def callbackPose(msg):
    global x,y,theta
    global xp,yp
    global xd,yd
    global flag,i
    D = 0.5

    if flag == 0:
        # Get Trajectory

        x_t = np.linspace(0,3*math.pi,60)
        y_t = np.sin(x_t)
        
        xd = x_t[i];yd = y_t[i]
        xd = 10
        yd = 10
        
        # Get Pose

        x = msg.x
        y = msg.y
        theta = msg.theta

        xp = x + D*math.cos(theta)
        yp = y + D*math.sin(theta)

        # if e_p[0] < 0.01:
        #     i+=1
        #     print("Posición alcanzada")
        # if i > (len(x_t)-1):
        #     i=0
        
        flag = 1

if __name__ == '__main__':
    threading.Thread(target=control,daemon=True).start()

    rospy.init_node("Control_Turtlebot")
    rospy.loginfo("Nodo Iniciado")

    init_time = time.time()
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose',Pose,callback=callbackPose)

    rospy.spin()