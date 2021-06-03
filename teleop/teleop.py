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

# Вопросы
# 0. Обсудить архитектуру телеуправления - c пивом покатит
# 1. Мб сделать управление с клавиатуры по таймеру (потестировать)
# 2. Че делать с вебом, Поднять локальный веб, 
# 3. 



class Teleop(Node):
    """
    Class for universal telecontrol for various robotic platforms
    """
    def __init__(self):
        """
        :Attributes:
            :cmd_msg: (Twist or ..) current control
         """
        super().__init__('teleop')
       
        self.init_parameters()
        self.get_parametes()
        self.declare_robot_teleop()
        self.cmd_msg = self.robot_teleop.get_cmd_msg()
        self.init_subs()
        self.init_pubs()
        self.timer = self.create_timer(self.dt, self.timer_keyboard_callback)
        # self.run()
        self.keyboard_control_recieved = False
        self.joystick_control_recieved = False
        self.web_control_recieved = False

        sc = pygame.display.set_mode((640, 320))   
        
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
                ('joystick_topic', "/joy"),
                ('movable_camera', False),
                ('v_limit', 1.5),
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
        self.joystick_topic = self.get_parameter('joystick_topic').get_parameter_value().string_value
        self.movable_camera = self.get_parameter('movable_camera').get_parameter_value().bool_value
        self.v_limit = self.get_parameter('v_limit').get_parameter_value().double_value
        self.w_limit = self.get_parameter('w_limit').get_parameter_value().double_value
        self.lin_a = self.get_parameter('lin_a').get_parameter_value().double_value
        self.ang_a = self.get_parameter('ang_a').get_parameter_value().double_value
        
        self.dt = 1 / self.update_rate

    def declare_robot_teleop(self):
        """
        Defines the class of the robot_teleop,  depending on the type of robot
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
        
        self.joy_sub = self.create_subscription(
            type(self.robot_teleop.get_joystick_msg()),
            self.joystick_topic,
            self.joystick_callback,
            1
        )

    def init_pubs(self):
        """
        """
        self.cmd_pub = self.create_publisher(type(self.cmd_msg), self.control_topic, 10)
        if self.movable_camera:
            self.head_cmd_pub = self.create_publisher(type(self.head_cmd_msg), self.head_cmd_topic, 10)

    def timer_keyboard_callback(self):
        """
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    pygame.quit()
                    sys.exit(0)

        pressed_keys = pygame.key.get_pressed()
        self.keyboard_control_recieved = True in pygame.key.get_pressed()
        if self.keyboard_control_recieved:          
            self.cmd_msg = self.robot_teleop.process_keyboard_input(
                keys=pressed_keys,
                curr_cmd=self.cmd_msg
            )
            self.cmd_pub.publish(self.cmd_msg)
        
        # if not receive any input
        if (
            not self.keyboard_control_recieved 
            and not self.joystick_control_recieved 
            and not self.web_control_recieved
        ):
            # STOP 
            self.cmd_msg = self.robot_teleop.get_cmd_msg()
            self.cmd_pub.publish(self.cmd_msg)

        self.keyboard_control_recieved = False


    def joystick_callback(self):
        """
        """
        self.joystick_control_recieved = True

    def joystick_callback(self):
        """
        """
        self.web_control_recieved = True


def main():
    """
    """
    rclpy.init()
    teleop_controller = Teleop()
    rclpy.spin(teleop_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
