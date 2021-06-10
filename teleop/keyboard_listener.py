import pynput
import os
from pynput import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardListener(Node):
    """
    """
    def __init__(self):
        """
        :Attributes:
            :keys_set: (set) the set of keys pressed at the current time
            :listener: (keyboard.Listener) analogue subscription (from ROS)
                    to keyboard events
            :dt: time delta between publications
            :keys_pub: ROS2 publisher, publishes the keys pressed
            :timer: ROS2 timer
            :keyboard_topic: (str) name of the topic to publish
         """
        super().__init__('keyboard_listener')

        self.keys_set = set()
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

        self.init_parameters()
        self.get_parametes()

        self.keys_pub = self.create_publisher(String, self.keyboard_topic, 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                ('update_rate', 20),
                ('keyboard_topic_name', '/keyboard')
            ]
        )

    def get_parametes(self):
        """
        Gets node parameters
        """
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.keyboard_topic = self.get_parameter('keyboard_topic_name').get_parameter_value().string_value
        self.dt = 1 / update_rate

    def on_press(self, pressed_key):
        """
        Calls when keys are pressed
        Adds a key name to the set of pressed keys.
        """
        if pressed_key is not None:
            if isinstance(pressed_key, pynput.keyboard.KeyCode) and pressed_key.char is not None:
                pressed_key = pressed_key.char.lower()
            elif isinstance(pressed_key, pynput.keyboard.Key):
                pressed_key = pressed_key.name
            self.keys_set.add(pressed_key)

    def on_release(self, released_key ):
        """
        Calls when keys are realesed
        Remove a key name from the set of pressed keys.
        """
        if released_key is not None:
            if isinstance(released_key, pynput.keyboard.KeyCode) and released_key.char is not None:
                released_key = released_key.char.lower()
            elif isinstance(released_key, pynput.keyboard.Key):
                released_key = released_key.name
            self.keys_set.discard(released_key)

    def timer_callback(self):
        """
        callback for a timer event
        creates a message and publishes it
        """
        msg = String()
        msg.data = ' '.join(str(k) for k in self.keys_set)
        self.keys_pub.publish(msg)



def main():
    """
    """
    rclpy.init()
    listener = KeyboardListener()
    rclpy.spin(listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
