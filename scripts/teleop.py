#!/usr/bin/env python3
import time
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Twist, PoseStamped

class ControlNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriber = rospy.Subscriber('cmd_vel', Twist, self.callback_vel)
        # self.subscriber2 = rospy.Subscriber('pose', PoseStamped,self.callback_pose)

        self.v_x = 0
        self.v_y = 0
        self.v_theta = 0

        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

        self.prevTime = time.time()
        self.elapsedTime = 0
        self.t = 0

    def control(self):
        # Time for Pose Estimation

        self.t = time.time()
        self.elapsedTime = self.t - self.prevTime

        # Robot Pose

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

        self.publish_velocity(self.u)
        self.simulated_pose()

    def callback_vel(self, data):
        
        # Control Velocities

        self.v_x = data.linear.x
        self.v_y = data.linear.y
        self.v_theta = data.angular.z

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
        self.pp = np.dot(np.dot(R,T),self.u)*0.005
        self.p = self.p + self.pp*self.elapsedTime
        self.prevTime = self.t

        self.x = self.p[0]
        self.y = self.p[1]
        self.theta = self.p[2]
        print(self.p)

    def publish_velocity(self,wheelVel):
        
        omniVel_msg = Quaternion()
        omniVel_msg.x = float(wheelVel[0])
        omniVel_msg.y = float(wheelVel[1])
        omniVel_msg.z = float(wheelVel[2])
        omniVel_msg.w = float(wheelVel[3])

        #Publishing Wheel Velocities        
        self.publisher.publish(omniVel_msg)

if __name__ == '__main__':
    rospy.init_node('Omni_Teleop', anonymous=True)
    control = ControlNode()

    rate = rospy.Rate(10)  # Adjust the rate as needed (e.g., 10 Hz)
    while not rospy.is_shutdown():
        control.control() 
        rate.sleep()

