#!/usr/bin/env python

import rospy
import numpy as np
from pynput import keyboard
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

class KeyPublisher:
    def __init__(self):
        self.publisher_ = rospy.Publisher('omni_vel', Quaternion, queue_size=10)
        self.pressed_keys = set()
        self.vel_x = 0
        self.vel_y = 0
        self.vel_theta = 0
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
	
	def timer_callback(self, event):
        if not self.pressed_keys:
            self.vel_x = 0
            self.vel_y = 0
            self.vel_theta = 0

    def on_press(self, key):
	self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback, oneshot=True)
        c = 0.1
        if key in {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}:
            self.pressed_keys.add(key)
            if keyboard.Key.up in self.pressed_keys:
                self.vel_x = self.vel_x + c
            if keyboard.Key.down in self.pressed_keys:
                self.vel_x = self.vel_x - c
            if keyboard.Key.left in self.pressed_keys:
                self.vel_y = self.vel_y - c
            if keyboard.Key.right in self.pressed_keys:
                self.vel_y = self.vel_y + c

            self.vel_x = np.clip(self.vel_x, -5, 5)
            self.vel_y = np.clip(self.vel_y, -5, 5)

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
            omniVel_msg.w = float(wheelVel[1])
            omniVel_msg.y = float(wheelVel[2])
            omniVel_msg.z = float(wheelVel[3])
            
            self.publisher_.publish(omniVel_msg)

    def on_release(self, key):
        try:
            self.pressed_keys.remove(key)
        except KeyError:
            pass  # Key was not in the set, ignore.

if __name__ == '__main__':
    rospy.init_node('key_publisher', anonymous=True)
    key_publisher = KeyPublisher()
    rospy.spin()

