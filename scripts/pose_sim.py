#!/usr/bin/env python
import rospy
import numpy as np
from pynput import keyboard
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

class KeyPublisher:
    def __init__(self):
        self.publisher_ = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.pos_x = 0
        self.pos_y = 0
        self.pos_thet = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_theta = 0

    def control():
        pass

    def publish_velocity(self):

        # Omnidirectional Cinematics
        pi = np.pi
        alpha = self.vel_theta + pi/4
        L = 0.425/2
        l = 0.44/2

        A = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                      [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
        linearVel = np.array([self.vel_x, self.vel_y, self.vel_theta])
        wheelVel = np.dot(A, linearVel )
        
        omniVel_msg = Quaternion()
        omniVel_msg.x = float(wheelVel[0])
        omniVel_msg.y = float(wheelVel[1])
        omniVel_msg.z = float(wheelVel[2])
        omniVel_msg.w = float(wheelVel[3])

        #Publishing Wheel Velocities        
        self.publisher_.publish(omniVel_msg)

if __name__ == '__main__':
    rospy.init_node('key_publisher', anonymous=True)
    key_publisher = KeyPublisher()

    rate = rospy.Rate(10)  # Adjust the rate as needed (e.g., 10 Hz)
    while not rospy.is_shutdown():
        key_publisher.control_logic() 
        key_publisher.publish_velocity()
        rate.sleep()
