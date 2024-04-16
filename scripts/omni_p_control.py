#!/usr/bin/env python3
import time
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, PoseStamped

class ControlNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriber= rospy.Subscriber('posesim/pose', PoseStamped, self.callback_pose)

        self.pd_list = [np.array([1, 0, 0]), np.array([1, 1, 0]), np.array([0, 1, 0]), np.array([0, 0, 0])]
        self.i = 0
        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

        self.K = 0.6*np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.e = np.array([0, 0, 0])

    def control(self):      
          
        # Error Computation

        self.e = self.pd_list[self.i] - self.p

        # Robot Inverse Kinematics

        pi = np.pi
        alpha = self.p[2] + pi/4
        L = 0.425/2
        l = 0.44/2
        T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                      [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
        
        v = np.dot(self.K,self.e)
        v = np.clip(v, -1, 1)
        self.u = np.dot(T,v)/0.05

        # Publish Wheel Velocities

        self.publish_velocity()

        # Print Variables
        print(" ")
        print("{:<10}|{:<10}|{:<10}".format("Pose","Desired","Errors"))
        for i in range(3):
            print("{:<10.4f}|{:<10.4f}|{:<10.4f}".format(self.p[i],self.pd_list[self.i][i],self.e[i]))


    def callback_pose(self, data):

        # Simulated Pose

        q = (data.pose.orientation.x,
             data.pose.orientation.y,
             data.pose.orientation.z,
             data.pose.orientation.w)
        
        (_,_,yaw) = euler_from_quaternion(q)

        self.p = np.array([data.pose.position.x,data.pose.position.y,yaw]) 
        
    def publish_velocity(self):
        wheelVel = self.u
        
        omniVel_msg = Quaternion()
        omniVel_msg.x = float(wheelVel[0])
        omniVel_msg.y = float(wheelVel[1])
        omniVel_msg.z = float(wheelVel[2])
        omniVel_msg.w = float(wheelVel[3])

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
    rospy.init_node('Control', anonymous=True)
    control = ControlNode()

    rate = rospy.Rate(5)  # Adjust the rate as needed (e.g., 10 Hz)
    while not rospy.is_shutdown():
        control.control() 
        control.update_target_position()
        rate.sleep()

