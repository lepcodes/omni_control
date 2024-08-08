#!/usr/bin/env python
import tf
import csv
import time
import math
import rospy 
import pickle
import rospkg
import threading
import numpy as np
import tf.transformations as tf_trans
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion, euler_from_matrix
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Joy

class ControlNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriber= rospy.Subscriber('posestamped', PoseStamped, self.callback_pose)
        self.subscriber= rospy.Subscriber('joy', Joy, self.callback_joy)
        
        self.loadPose = True
        self.start_time = time.time()
        self.prevTime = time.time()
        self.pd_list, self.quaternion_list = self.load_pose_list()  
        self.pose_list = []
        self.posed_list = []
        self.i = 0
        rospy.on_shutdown(self.save_pose_list)
        
        self.joyIsWorking = 0
        self.joyScaleX = 0.2
        self.joyScaleY = 0.2
        self.joyScaleTheta = 0.2 
        self.calibInProgress = 1
        self.transformer = tf.TransformerROS()

        # Initializing Pose
        
        self.quaternion = np.array([0, 0, 0, 0])
        self.p = np.array([0, 0, 0])
        self.pd = np.array([0, 0, 0])

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

    def load_pose_list(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('omni_control')
        file_path = path + '/pose_list.csv'
        pose_list = []
        
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                pose = np.array([float(element) for element in row])
                pose_list.append(pose)
       
        pickle_file_path = path + '/np_array.pkl'
        with open(pickle_file_path, 'rb') as f:
            quaternion_list = pickle.load(f)

        return pose_list,quaternion_list
    
    def control(self):
        
        # Desired Pose Computation
        
        if self.loadPose == False:
            t = time.time() - self.start_time
            r = 1
            omega = 2 * np.pi / 200
            phi = 0
            x = r * np.cos(omega * t + phi)
            y = r * np.sin(omega * t + phi)
            self.pd = np.array([x, y, 0])
        else:
            self.pd  = self.pd_list[self.i]
            
        # Error Computation   
        
        self.e_p = self.pd - self.p #Proportional Error for x, y

        # Quaternion Error Computation

        q_current_conj = tf_trans.quaternion_conjugate(self.quaternion)
        q_error = tf_trans.quaternion_multiply(self.quaternion_list[self.i], q_current_conj)
        theta_error = 2*math.atan2(np.linalg.norm(q_error[:2]), q_error[3])
        
        rotation_matrix = tf_trans.quaternion_matrix(q_error)[:3, :3]

        # Ensure the trace of the rotation matrix is within the valid range [-1, 3]
        trace = np.clip((np.trace(rotation_matrix) - 1) / 2, -1, 1)

        # Calculate the angle of rotation
        angle = np.arccos(trace)

        # Calculate the axis of rotation
        axis = np.array([
            rotation_matrix[2, 1] - rotation_matrix[1, 2],
            rotation_matrix[0, 2] - rotation_matrix[2, 0],
            rotation_matrix[1, 0] - rotation_matrix[0, 1]
        ])
        if np.linalg.norm(axis) != 0:
            axis = axis / np.linalg.norm(axis)
            
        if axis[1] > 0:
            # Axis points in the positive Y direction, the error is positive
            angle = -angle
        else:
            # Axis points in the negative Y direction, the error is negative
            angle = angle
        self.e_p[2] = angle # Proportional error of theta based on quaternion
        
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

            # Control Velocities
            
            self.v_x = U_1 
            self.v_y = U_2 
            self.v_theta = U_3 

            # Publish Wheel Velocities

            self.publish_velocity()

            # Display SNPIDs Data
            
            v = [self.v_x,self.v_y,self.v_theta]
            print(" ")
            print("{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}".format("Pose","Pose_d","Errors (x)","Errors (y)","Errors (t)","Gains (x)","Gains (y)","Gains (t)","Control"))
            for i in range(3):
                print("{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}".format(self.p[i],self.pd[i],X_1[i,0],X_2[i,0],X_3[i,0],self.W_1[i,0],self.W_2[i,0],self.W_3[i,0],v[i]))
            
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


        else:

            # Display SNPIDs Data

            print('JoyStick is Working!')
            print("{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}|{:<10}".format("Pose","Pose_d","Errors (x)","Errors (y)","Errors (tht)","Gains (x)","Gains (y)","Gains (tht)"))
            for i in range(3):
                print("{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}|{:<10.4f}".format(self.p[i],self.pd_list[self.i][i],X_1[i,0],X_2[i,0],X_3[i,0],self.W_1[i,0],self.W_2[i,0],self.W_3[i,0]))
        
    def callback_pose(self, data):

        # Get Pose from Message
        
        pose = (data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z)
        quaternion = (data.pose.orientation.x,
                      data.pose.orientation.y,
                      data.pose.orientation.z,
                      data.pose.orientation.w)
        
        # Set Pose of Center of Omni-Robot
        
        (_,pitch,_) = euler_from_quaternion(quaternion)
        self.p = np.array([data.pose.position.z,-data.pose.position.x, -pitch])
        self.quaternion = np.array(quaternion)
        
        self.pose_list.append(self.p)
        self.posed_list.append(self.pd)
    def callback_joy(self, data):
        
        # Get Enable Button Status (11)

        self.joyIsWorking = data.buttons[4]
        
        if self.joyIsWorking == 1:
        
            # Joystick Velocities

            self.v_x = data.axes[1]*self.joyScaleX
            self.v_y = data.axes[0]*self.joyScaleY
            self.v_theta = data.axes[3]*self.joyScaleTheta
            
            # Publish Wheel Velocities
        
            self.publish_velocity()
            
    def publish_velocity(self):
        # Robot Orientation

        theta = self.p[2]
        #theta = 0

        # Robot Inverse Kinematics

        pi = np.pi
        alpha = theta + pi/4
        L = 0.425/2
        l = 0.44/2
        T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                      [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
        
        v = np.array([self.v_x, self.v_y, self.v_theta])
        self.u = np.dot(T,v)/0.05
        
        #Publishing Wheel Velocities

        omniVel_msg = Quaternion()
        omniVel_msg.x = int(10*self.u[0])
        omniVel_msg.y = int(10*self.u[1])
        omniVel_msg.z = int(10*self.u[2])
        omniVel_msg.w = int(10*self.u[3])

        self.publisher.publish(omniVel_msg)

    def update_target_position(self):
        if self.loadPose == True:
            # Check if reached the current target position
            #if np.linalg.norm(self.e_p) < 0.05:
            elapsedTime = time.time() - self.prevTime
            print(elapsedTime)
            if  elapsedTime > 0.1:
                self.prevTime = time.time()
                # Move to the next target position
                self.i = (self.i + 1)
                #print('Posicion Alcanzada!')
                # If reached the last target position
                if self.i == len(self.pd_list):
                    # Stay in position
                    self.i = len(self.pd_list)-1
                    
    def save_pose_list(self):
        rospack = rospkg.RosPack()
        # get the file path for your_package
        path = rospack.get_path('omni_control')
        with open(path + '/pose_list_control.csv', 'w') as f:
            writer = csv.writer(f)
            for pose in self.pose_list:
                writer.writerow(pose)
        with open(path + '/pose_d_list_control.csv', 'w') as f:
            writer = csv.writer(f)
            for pose in self.posed_list:
                writer.writerow(pose) 
        print('Trajectory saved in:')
        print(path)        

if __name__ == '__main__':
    
    rospy.init_node('Control_SNPID', anonymous=True)
    control = ControlNode()
    rate = rospy.Rate(10)  # Adjust the rate as needed (e.g., 10 Hz)
    try:
        while not rospy.is_shutdown():
            control.control() 
            control.update_target_position()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
