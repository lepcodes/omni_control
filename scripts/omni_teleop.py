#!/usr/bin/env python3
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, Twist, PoseStamped

class TeleOpNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriberJoy  = rospy.Subscriber('cmd_vel', Twist, self.callback_vel)
        self.subscriberPose = rospy.Subscriber('posesim/pose', PoseStamped, self.callback_pose)

        self.v_x = 0
        self.v_y = 0
        self.v_theta = 0

        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

    def control(self):
        
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

        # Publish Wheel Velocities

        self.publish_velocity(self.u)

    def callback_vel(self, data):
        
        # Joystick Velocities

        self.v_x = data.linear.x
        self.v_y = data.linear.y
        self.v_theta = data.angular.z

    def callback_pose(self, data):
        
        # Simulated Pose

        q = (data.pose.orientation.x,
             data.pose.orientation.y,
             data.pose.orientation.z,
             data.pose.orientation.w)
        
        (_,_,yaw) = euler_from_quaternion(q)

        self.p = np.array([data.pose.position.x,data.pose.position.y,yaw]) 

    def publish_velocity(self,wheelVel):
        
        #Publishing Wheel Velocities

        omniVel_msg = Quaternion()
        omniVel_msg.x = float(wheelVel[0])
        omniVel_msg.y = float(wheelVel[1])
        omniVel_msg.z = float(wheelVel[2])
        omniVel_msg.w = float(wheelVel[3])

        self.publisher.publish(omniVel_msg)

if __name__ == '__main__':
    rospy.init_node('Omni_Teleop', anonymous=True)
    teleop = TeleOpNode()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        teleop.control() 
        rate.sleep()

