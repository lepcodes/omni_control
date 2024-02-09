#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard

class KeyPublisher:
    def __init__(self):
        self.publisher_ = rospy.Publisher('key_topic', String, queue_size=10)
        self.pressed_keys = set()
        self.vel_x = 0
        self.vel_y = 0
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        x = 0
        y = 0
        if key in {keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right}:
            self.pressed_keys.add(key)
            if keyboard.Key.up in self.pressed_keys:
                x = x + 1
            if keyboard.Key.down in self.pressed_keys:
                x = x - 1
            if keyboard.Key.left in self.pressed_keys:
                y = y - 1
            if keyboard.Key.right in self.pressed_keys:
                y = y + 1

            self.vel_x = x
            self.vel_y = y
            
            msg = String()
            msg.data = "Vel_x = {}, Vel_y = {}".format(self.vel_x, self.vel_y)
            self.publisher_.publish(msg)

    def on_release(self, key):
        try:
            self.pressed_keys.remove(key)
        except KeyError:
            pass  # Key was not in the set, ignore.

if __name__ == '__main__':
    rospy.init_node('key_publisher', anonymous=True)
    key_publisher = KeyPublisher()
    rospy.spin()

