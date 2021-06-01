# from pygame import time
import sys
import time
import numpy as np
import pygame
from launch_ros import descriptions
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor

# https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py


class Teleop(Node):
    """
    """
    def __init__(self):
        """
        """
        super().__init__('teleop')

        self.cmd_msg = Twist()

        self.init_parameters()
        self.get_parametes()
        self.init_subs()
        self.init_pubs()

        self.run()
        
    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                ('robot_type', "rosbot"),
                ('controller_type', "keyboard"),
                ('control_topic', "/cmd_vel"),
                ('v_limit', 0.5),
                ('w_limit', 2.5),
                ('lin_a', 0.25),
                ('ang_a', 0.5),
            ]
        )
        print("INIT PARAMETERS")

    def get_parametes(self):
        """
        Gets node parameters
        """
        import os
        os.popen("ros2 param list")
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.v_limit = self.get_parameter('v_limit').get_parameter_value().double_value
        self.w_limit = self.get_parameter('w_limit').get_parameter_value().double_value
        self.lin_a = self.get_parameter('lin_a').get_parameter_value().double_value
        self.ang_a = self.get_parameter('ang_a').get_parameter_value().double_value
        
        self.dt = 1 / self.update_rate

    def get_parameters_from_config(self):
        """
        """
        pass

    def init_subs(self):
        """
        """
        pass

    def init_pubs(self):
        """
        """
        if self.robot_type in ['rosbot', 'tankbot']:
            self.cmd_pub = self.create_publisher(Twist, self.control_topic, 10)

    def controller_callback(self, msg):
        """
        """
        pass

    def process_keyboard_control_for_rosbot(self, keys, curr_cmd):
        """
        """

        lin_vel = curr_cmd.linear.x
        ang_vel = curr_cmd.angular.z
        k = 1
        if keys[pygame.K_LSHIFT]:
            k = 2

        if keys[pygame.K_w]:        # forward
            lin_vel += self.lin_a * self.dt  * k
        elif keys[pygame.K_s]:      # backward
            lin_vel -= self.lin_a * self.dt  * k
        else:
            lin_vel /= 1.5
            lin_vel = 0.0 if abs(lin_vel) < 0.001 else lin_vel

        if keys[pygame.K_a]:        # left
            ang_vel = 0.0 if ang_vel < 0.0 else ang_vel
            ang_vel += self.ang_a * self.dt  * k
        elif keys[pygame.K_d]:      # right
            ang_vel = 0.0 if ang_vel > 0.0 else ang_vel
            ang_vel -= self.ang_a * self.dt  * k
        else:
            ang_vel /= 1.5
            ang_vel = 0.0 if abs(ang_vel) < 0.001 else ang_vel

        if keys[pygame.K_SPACE]:
            curr_cmd = Twist()

        lin_vel = np.clip(lin_vel, a_min=-self.v_limit, a_max=self.v_limit)
        ang_vel = np.clip(ang_vel, a_min=-self.w_limit, a_max=self.w_limit)
        lin_vel = round(lin_vel, 3)
        ang_vel = round(ang_vel, 3)
        print(lin_vel, ang_vel)
        curr_cmd.linear.x = lin_vel
        curr_cmd.angular.z = ang_vel

        return curr_cmd

    def run(self):
        """
        """
        print("RUN")
        # FPS = 60
        W = 640  # ширина экрана
        H = 320  # высота экрана
        WHITE = (255, 255, 255)
        
        sc = pygame.display.set_mode((W, H))
        # clock = pygame.time.Clock()                          
        while rclpy.ok():
            for event in pygame.event.get():
                # проверка для закрытия окна
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        pygame.quit()
                        sys.exit(0)

            keys = pygame.key.get_pressed()
            self.cmd_msg = self.process_keyboard_control_for_rosbot(keys, self.cmd_msg)
            self.cmd_pub.publish(self.cmd_msg)
            time.sleep(self.dt)


def main():
    """
    """
    rclpy.init()
    teleop_controller = Teleop()
    rclpy.spin(teleop_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
