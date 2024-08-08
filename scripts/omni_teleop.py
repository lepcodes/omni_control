#!/usr/bin/env python
import tf
import csv
import rospy
import pickle
import rospkg
import numpy as np
from tf.transformations import euler_from_quaternion, euler_from_matrix
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Joy

class TeleOpNode:
    def __init__(self):
        self.publisher = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.subscriberJoy  = rospy.Subscriber('joy', Joy, self.callback_joy)
        self.subscriberPose = rospy.Subscriber('posestamped', PoseStamped, self.callback_pose)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        self.firstCount = 0
        self.firstPose = False
        self.pose_list = []
        self.quaternion_list = []
        rospy.on_shutdown(self.save_pose_list)
        
        self.joyScaleX = 0.2
        self.joyScaleY = 0.2
        self.joyScaleTheta = 0.2 
        self.transformer = tf.TransformerROS()        

        self.v_x = 0
        self.v_y = 0
        self.v_theta = 0

        self.p = np.array([0, 0, 0])
        self.quaternion = np.array([0, 0, 0, 0])
        
    def callback_joy(self, data):
        
        # Joystick Velocities

        self.v_x = data.axes[1]*self.joyScaleX
        self.v_y = data.axes[0]*self.joyScaleY
        self.v_theta = data.axes[3]*self.joyScaleTheta
        
        # Publish Wheel Velocities
        
        self.publish_velocity()

    def callback_pose(self, data):
    
        # Get Pose from Message
        
        pose = (data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z)
        quaternion = (data.pose.orientation.x,
                      data.pose.orientation.y,
                      data.pose.orientation.z,
                      data.pose.orientation.w)
        
        (roll,pitch,yaw) = euler_from_quaternion(quaternion)
        self.p = np.array([data.pose.position.z,-data.pose.position.x,-pitch])
        self.quaternion = np.array(quaternion)

        if self.firstPose == False:
            self.firstPose = True
            rospy.loginfo("Pose is being Published!!!")
            rospy.loginfo("Waiting to Save the Pose...")

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
        
        # Publishing Wheel Velocities

        omniVel_msg = Quaternion()
        omniVel_msg.x = int(10*self.u[0])
        omniVel_msg.y = int(10*self.u[1])
        omniVel_msg.z = int(10*self.u[2])
        omniVel_msg.w = int(10*self.u[3])

        self.publisher.publish(omniVel_msg)
        
    def timer_callback(self, event):
        
        if self.firstPose == True:
            if self.firstCount > 5:
                self.pose_list.append(self.p)
                self.quaternion_list.append(np.array(self.quaternion))
                print("Pose Saved")
            self.firstCount += 1
        else:
            rospy.loginfo("Waiting for Pose to be Published...")
            
    def save_pose_list(self):
        rospack = rospkg.RosPack()
        # get the file path for your_package
        path = rospack.get_path('omni_control')
        file_path = path + '/pose_list.csv'
        with open(file_path, 'w') as f:
            writer = csv.writer(f)
            for pose in self.pose_list:
                writer.writerow(pose)
        print('Trajectory saved in:')
        print(file_path)
        
	    # Save numpy array using pickle
        pickle_file_path = path + '/np_array.pkl'
        with open(pickle_file_path, 'wb') as f:
            pickle.dump(self.quaternion_list, f)
            print('Numpy array saved in:')
            print(pickle_file_path)
    
if __name__ == '__main__':
    rospy.init_node('Omni_Teleop', anonymous=True)
    teleop = TeleOpNode()

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


