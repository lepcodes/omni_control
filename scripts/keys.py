#!/usr/bin/env python

import rospy
import numpy as np
from pynput import keyboard
from std_msgs.msg import String
from omni_control.msg import OmniVel

class KeyPublisher:
    def __init__(self):
        self.publisher_ = rospy.Publisher('omni_vel', OmniVel, queue_size=10)
        self.pressed_keys = set()
        self.vel_x = 0
        self.vel_y = 0
        self.vel_theta = 0
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        c = 5
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

            self.vel_x = np.clip(self.vel_x, -1000, 1000)
            self.vel_y = np.clip(self.vel_y, -1000, 1000)

            # Omnidirectional Cinematics
            pi = np.pi
            alpha = self.vel_theta + pi/4
            L = 42.5/2
            l = 44/2

            A = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                          [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                          [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                          [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
            B = np.array([self.vel_x, self.vel_y, self.vel_theta])
            result = np.dot(A, B)
            result = np.clip(result, -255, 255)
            result = result.tolist()

            omniVel_msg = OmniVel()
            omniVel_msg.wheelVel = result
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

