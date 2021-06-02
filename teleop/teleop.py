import sys
import time
import numpy as np
import pygame
from launch_ros import descriptions
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rosbot_teleop import RosbotTeleop


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
        self.declare_robot_teleop()
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
                ('update_rate', 20),
            ]
        )

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

    def declare_robot_teleop(self):
        """
        """
        if self.robot_type == "rosbot":
            self.robot_teleop = RosbotTeleop(
                dt=self.dt,
                v_limit=self.v_limit,
                w_limit=self.w_limit,
                lin_a=self.lin_a, 
                ang_a=self.ang_a
            )
        elif self.robot_type == "tankbot":
            pass # TODO TankBotTeleop
        elif self.robot_type == "qadrotor":
            pass # TODO qadrotor
        else:
            raise NameError('Unknown Robot type!')


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

    def run(self):
        """
        """
        W = 640  # ширина экрана
        H = 320  # высота экрана
        WHITE = (255, 255, 255)
        sc = pygame.display.set_mode((W, H))   

        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        pygame.quit()
                        sys.exit(0)

            self.cmd_msg =  self.robot_teleop.keyboard_cotrol_callback(
                keys = pygame.key.get_pressed(),
                curr_cmd = self.cmd_msg
            )

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
