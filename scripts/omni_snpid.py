#!/usr/bin/env python
import time
import math
import rospy 
import threading
import numpy as np
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Joy

class ControlNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriber= rospy.Subscriber('posesim/pose', PoseStamped, self.callback_pose)
        self.subscriber= rospy.Subscriber('joy', Joy, self.callback_joy)

        self.pd_list = [np.array([1, 0, 0]), np.array([1, 1, 0]), np.array([0, 1, 0]), np.array([0, 0, 0])]
        self.i = 0

        self.joyIsWorking = 0

        # Initializing Pose and Velocities

        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

        # Initializing Errors

        self.e_p = np.array([0, 0, 0])
        self.e_i = np.array([0, 0, 0])
        self.e_d = np.array([0, 0, 0])
        self.e_prev = np.array([0, 0, 0])
        self.e_prime = np.array([0, 0, 0])

        # Initializing SNPID Matrices

        self.P_1 = np.matrix(np.eye(3))
        self.Q_1 = 0.1 * np.matrix(np.eye(3))
        self.R_1 = 0.0001 * np.matrix(np.eye(1))
        self.W_1 = np.matrix(np.random.rand(3,1))

        self.P_2 = np.matrix(np.eye(3))
        self.Q_2 = 0.1 * np.matrix(np.eye(3))
        self.R_2 = 0.0001 * np.matrix(np.eye(1))
        self.W_2 = np.matrix(np.random.rand(3,1))

        self.P_3 = np.matrix(np.eye(3))
        self.Q_3 = 0.1 * np.matrix(np.eye(3))
        self.R_3 = 0.0001 * np.matrix(np.eye(1))
        self.W_3 = np.matrix(np.random.rand(3,1))

        # Initializing SNPID learning rates

        self.alpha_1 = 0.2
        self.alpha_2 = 0.2
        self.alpha_3 = 0.2
        self.eta_1 = 0.02
        self.eta_2 = 0.02
        self.eta_3 = 0.02
        self.beta = 0.01

    def control(self):      
          
        # Error Computation

        self.e_p = self.pd_list[self.i] - self.p
        self.e_i = self.e_i + self.e_prime
        self.e_d = self.e_p - self.e_prev
        self.e_prev = self.e_p

        # Define SNPIDs Inputs

        X_1 = np.matrix([float(self.e_p[0]), float(self.e_i[0]), float(self.e_d[0])]).T
        X_2 = np.matrix([float(self.e_p[1]), float(self.e_i[1]), float(self.e_d[1])]).T
        X_3 = np.matrix([float(self.e_p[2]), float(self.e_i[2]), float(self.e_d[2])]).T

        # Check if Joystick is Sending Data

        if self.joyIsWorking == 0:
        
            # Compute Weighted Sums of SNPIDs

            V_1 = self.W_1.T*X_1
            V_2 = self.W_2.T*X_2
            V_3 = self.W_3.T*X_3

            # Compute SNPIDs Outputs

            U_1 = self.alpha_1*np.tanh(V_1)
            U_2 = self.alpha_2*np.tanh(V_2)
            U_3 = self.alpha_3*np.tanh(V_3)
            
            # U_1 = float(V_1)
            # U_2 = float(V_2)
            # U_3 = float(V_3)

            # Saturate Output (If Necessary)

            U_1 = float(np.clip(U_1,-.2,.2))
            U_2 = float(np.clip(U_2,-.2,.2))
            U_3 = float(np.clip(U_3,-.2,.2))

            # Robot Inverse Kinematics (Control Velocities)

            pi = np.pi
            alpha = self.p[2] + pi/4
            L = 0.425/2
            l = 0.44/2
            T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                        [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                        [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                        [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
            v = np.array([[U_1],[U_2],[U_3]])
            self.u = np.dot(T, v)/0.05

            # Publish Wheel Velocities

            self.publish_velocity()

            # Display SNPIDs Data

            print(" ")
            print("{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}".format("Pose","Pose_d","Errors (x)","Errors (y)","Errors (θ)","Gains (x)","Gains (y)","Gains (θ)","Control"))
            for i in range(3):
                print("{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}".format(self.p[i],self.pd_list[self.i][i],X_1[i,0],X_2[i,0],X_3[i,0],self.W_1[i,0],self.W_2[i,0],self.W_3[i,0],v[i,0]))
            
            # EKF Training Algorithm
                
            H_1 = self.alpha_1*(1-float(np.tanh(V_1)**2))*X_1
            H_2 = self.alpha_2*(1-float(np.tanh(V_2)**2))*X_2
            H_3 = self.alpha_3*(1-float(np.tanh(V_3)**2))*X_3

            # H_1 = X_1
            # H_2 = X_2
            # H_3 = X_3

            K_1 = self.P_1*H_1*np.linalg.inv(self.R_1 + H_1.T*self.P_1*H_1)
            K_2 = self.P_2*H_2*np.linalg.inv(self.R_2 + H_2.T*self.P_2*H_2)
            K_3 = self.P_3*H_3*np.linalg.inv(self.R_3 + H_3.T*self.P_3*H_3)

            self.W_1 = self.W_1 + self.eta_1*(K_1*self.e_p[0] - self.beta*float(self.e_p[0])*self.W_1)
            self.W_2 = self.W_2 + self.eta_2*(K_2*self.e_p[1] - self.beta*float(self.e_p[1])*self.W_2)
            self.W_3 = self.W_3 + self.eta_3*(K_3*self.e_p[2] - self.beta*float(self.e_p[2])*self.W_3)

            self.W_1 = abs(self.W_1)
            self.W_2 = abs(self.W_2)
            self.W_3 = abs(self.W_3)

            self.P_1 = self.P_1 - K_1*H_1.T*self.P_1 + self.Q_1
            self.P_2 = self.P_2 - K_2*H_2.T*self.P_2 + self.Q_2
            self.P_3 = self.P_3 - K_3*H_3.T*self.P_3 + self.Q_3

            # Anti-WindUp

            if abs(float(V_1)) > self.alpha_1:
                prime_1 = 0
            else:
                prime_1 = self.e_p[0]
            if abs(float(V_2)) > self.alpha_2:
                prime_2 = 0
            else:
                prime_2 = self.e_p[1]
            if abs(float(V_3)) > self.alpha_3:
                prime_3 = 0
            else:
                prime_3 = self.e_p[2]

            self.e_prime = np.array([prime_1,prime_2,prime_3])

            print(self.e_prime)

        else:
            print('JoyStick is Working!')

            # Display SNPIDs Data

            print(" ")
            print("{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}".format("Pose","Pose_d","Errors (x)","Errors (y)","Errors (θ)","Gains (x)","Gains (y)","Gains (θ)"))
            for i in range(3):
                print("{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}".format(self.p[i],self.pd_list[self.i][i],X_1[i,0],X_2[i,0],X_3[i,0],self.W_1[i,0],self.W_2[i,0],self.W_3[i,0]))
            

    def callback_pose(self, data):

        # Simulated Pose

        q = (data.pose.orientation.x,
             data.pose.orientation.y,
             data.pose.orientation.z,
             data.pose.orientation.w)
        
        (_,_,yaw) = euler_from_quaternion(q)

        self.p = np.array([data.pose.position.x,data.pose.position.y,yaw]) 

    def callback_joy(self, data):

        # Get Enable Button Status (11)

        self.joyIsWorking = data.buttons[4]
        
    def publish_velocity(self):
        wheelVel = self.u
        
        omniVel_msg = Quaternion()
        omniVel_msg.x = int(10*wheelVel[0])
        omniVel_msg.y = int(10*wheelVel[1])
        omniVel_msg.z = int(10*wheelVel[2])
        omniVel_msg.w = int(10*wheelVel[3])

        #Publishing Wheel Velocities        
        self.publisher.publish(omniVel_msg)

    def update_target_position(self):
        # Check if reached the current target position
        if np.linalg.norm(self.pd_list[self.i] - self.p) < 0.1:
            # Move to the next target position
            self.i = (self.i + 1)
            if self.i == len(self.pd_list):
                self.i = len(self.pd_list)-1

if __name__ == '__main__':
    
    rospy.init_node('Control_SNPID', anonymous=True)
    control = ControlNode()

    rate = rospy.Rate(10)  # Adjust the rate as needed (e.g., 10 Hz)
    while not rospy.is_shutdown():
        control.control() 
        control.update_target_position()
        rate.sleep()

