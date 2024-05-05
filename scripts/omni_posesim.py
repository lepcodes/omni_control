#!/usr/bin/env python
import time
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, PoseStamped

class PoseSimNode:
    def __init__(self):
        self.subscriber = rospy.Subscriber('omni_vel', Quaternion, self.callback_wheel_vel)
        self.publisher = rospy.Publisher('posesim', PoseStamped, queue_size=10)

        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])

        self.prevTime = time.time()
        self.elapsedTime = 0
        self.t = 0

    def callback_wheel_vel(self, data):
        # Time for Pose Estimation

        self.t = time.time()
        self.elapsedTime = self.t - self.prevTime

        # Velocity of Wheels

        self.u = np.array([data.x,data.y,data.z,data.w])

        # Robot Forward Kinematics

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
        self.pp = np.dot(np.dot(R,T),self.u)*0.002

        # Integrator

        self.p = self.p + self.pp*self.elapsedTime
        self.prevTime = self.t

        # Publish Pose
        
        self.publish_pose()

    def publish_pose(self):
        
        poseMsg = PoseStamped()

        poseMsg.pose.position.x = -self.p[1]
        poseMsg.pose.position.y = 0
        poseMsg.pose.position.z = self.p[0]

        q = quaternion_from_euler(0,-self.p[2],0)
        
        poseMsg.pose.orientation.x = q[0]
        poseMsg.pose.orientation.y = q[1]
        poseMsg.pose.orientation.z = q[2]
        poseMsg.pose.orientation.w = q[3]

        self.publisher.publish(poseMsg)

if __name__ == '__main__':
    rospy.init_node('Pose_Sim', anonymous=True)
    poseSim = PoseSimNode()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
