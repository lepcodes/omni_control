#!/usr/bin/env python
import time
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion

class ControlNode:
    def __init__(self):
        self.publisher_ = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.pd = np.array([2, 2, 0])
        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

        self.K = 0.6*np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.e = np.array([0, 0, 0])
        self.eprev = np.array([0, 0, 0])
        self.prevTime = time.time()
        self.elapsedTime = 0
        self.t = 0

    def control(self):
        self.t = time.time()
        self.elapsedTime = self.t - self.prevTime
        
        #  Error Calculation
        self.e = self.pd - self.p

        # Omnidirectional Cinematics
        pi = np.pi
        alpha = self.p[2] + pi/4
        L = 0.425/2
        l = 0.44/2
        T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                      [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
        
        v = np.dot(self.K,self.e)
        v = np.clip(v, -0.2, 0.2)
        self.u = np.dot(T,v)/0.05

    def publish_velocity(self):
        wheelVel = self.u
        
        omniVel_msg = Quaternion()
        omniVel_msg.x = float(wheelVel[0])
        omniVel_msg.y = float(wheelVel[1])
        omniVel_msg.z = float(wheelVel[2])
        omniVel_msg.w = float(wheelVel[3])

        #Publishing Wheel Velocities        
        self.publisher_.publish(omniVel_msg)

    def simulated_pose(self):
        L = 0.425/2
        l = 0.44/2
        R = np.array([[np.cos(self.p[2]), -np.sin(self.p[2]), 0],
                      [np.sin(self.p[2]),  np.cos(self.p[2]), 0],
                      [                0,                  0, 1]])
        T = np.array([[1, -1, -(L+l)],
                      [1,  1,  (L+l)],
                      [1,  1, -(L+l)],
                      [1, -1,  (L+l)]])
        T = np.linalg.pinv(T)
        self.pp = np.dot(np.dot(R,T),self.u)*0.05
        self.p = self.p + self.pp*self.elapsedTime
        
        print("Pose: "+str(self.p))
        print("Vel:  "+str(self.pp))
        self.prevTime = self.t

if __name__ == '__main__':
    rospy.init_node('Control', anonymous=True)
    control = ControlNode()

    rate = rospy.Rate(5)  # Adjust the rate as needed (e.g., 10 Hz)
    while not rospy.is_shutdown():
        control.control() 
        control.publish_velocity()
        control.simulated_pose()
        rate.sleep()
