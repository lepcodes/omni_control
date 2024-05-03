#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, euler_from_matrix
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Joy

class TeleOpNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriberJoy  = rospy.Subscriber('joy', Joy, self.callback_joy)
        self.subscriberPose = rospy.Subscriber('posestamped', PoseStamped, self.callback_pose)
        
        self.joyScaleX = 0.2
        self.joyScaleY = 0.2
        self.joyScaleTheta = 0.2 
        self.transformer = tf.TransformerROS()        

        self.v_x = 0
        self.v_y = 0
        self.v_theta = 0

        self.p = np.array([0, 0, 0])
        self.pp = np.array([0, 0, 0])
        
    def callback_joy(self, data):
        
        # Joystick Velocities

        self.v_x = data.axes[1]*self.joyScaleX
        self.v_y = data.axes[0]*self.joyScaleY
        self.v_theta = data.axes[3]*self.joyScaleTheta
        
        # Publish Wheel Velocities
        
        self.publish_velocity()

    def callback_pose(self, data):
        
        # Simulated Pose

        q = (data.pose.orientation.x,
             data.pose.orientation.y,
             data.pose.orientation.z,
             data.pose.orientation.w)
        
        (roll,pitch,yaw) = euler_from_quaternion(q)
        print(roll,pitch,yaw)
        # Transformation Matrix

        gTc = self.transformer.fromTranslationRotation((data.pose.position.x,data.pose.position.y,data.pose.position.z),q)
        cTo = np.array([[0,-1, 0, 1],
                        [0, 0,-1, 1],
                        [1, 0, 0, 1],
                        [0, 0, 0, 1]]).astype(float)
        gTo = np.dot(gTc,cTo)
        gTo_rm = gTo[:3,:3]
        (roll,pitch,yaw) = euler_from_matrix(gTo_rm,'sxyz')
        
        self.p = np.array([data.pose.position.x,data.pose.position.y,yaw])}
        print(roll,pitch,yaw)

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

if __name__ == '__main__':
    rospy.init_node('Omni_Teleop', anonymous=True)
    teleop = TeleOpNode()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

