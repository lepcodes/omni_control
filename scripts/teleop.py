#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Twist

class ControlNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriber = rospy.Subscriber('cmd_vel', Twist, self.callback)

    def callback(self, data):
        # Process the data here
        x = data.linear.x
        y = data.linear.y
        theta = data.angular.z

        pi = np.pi
        alpha = theta + pi/4
        L = 0.425/2
        l = 0.44/2
        T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                      [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                      [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
        
        v = np.array([x,y,theta])
        u = np.dot(T,v)/0.05

        self.publish_velocity(u)

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
        rate.sleep()
