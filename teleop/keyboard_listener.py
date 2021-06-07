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
            :keys_set:
            :listener:
            :dt:
            :keys_pub:
            :timer:
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

        self.keys_pub = self.create_publisher(String, "/keyboard", 10)
        self.timer = self.create_timer(self.dt, self.timer_keyboard_callback)

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                ('update_rate', 20)
            ]
        )

    def get_parametes(self):
        """
        Gets node parameters
        """
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.dt = 1 / update_rate

    def on_press(self, pressed_key):
        """
        """
        if pressed_key is not None:
            if isinstance(pressed_key, pynput.keyboard.KeyCode) and pressed_key.char is not None:
                pressed_key = pressed_key.char.lower()
            elif isinstance(pressed_key, pynput.keyboard.Key):
                pressed_key = pressed_key.name
            self.keys_set.add(pressed_key)

    def on_release(self, released_key ):
        """
        """
        if released_key is not None:
            if isinstance(released_key, pynput.keyboard.KeyCode) and released_key.char is not None:
                released_key = released_key.char.lower()
            elif isinstance(released_key, pynput.keyboard.Key):
                released_key = released_key.name
            self.keys_set.discard(released_key)

    def timer_keyboard_callback(self):
        """
        """
        # os.system('cls' if os.name == 'nt' else 'clear')
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
