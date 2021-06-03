import numpy as np
import pygame
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class RosbotTeleop():
    """
    """
    def __init__(self, dt, v_limit, w_limit, lin_a, ang_a):
        """
        """
        self.dt = dt
        self.v_limit = v_limit
        self.w_limit = w_limit
        self.lin_a = lin_a
        self.ang_a = ang_a


    def get_cmd_msg(self):
        return Twist()

    def get_joystick_msg(self):
        return Joy()

    def process_keyboard_input(self, keys, curr_cmd):
        """
        """

        lin_vel = curr_cmd.linear.x
        ang_vel = curr_cmd.angular.z
        k = 1
        if keys[pygame.K_LSHIFT]:
            k = 2

        if keys[pygame.K_w]:    # forward
            lin_vel += self.lin_a * self.dt  * k
        elif keys[pygame.K_s]:  # backward
            lin_vel -= self.lin_a * self.dt  * k
        else:
            lin_vel /= 1.5
            lin_vel = 0.0 if abs(lin_vel) < 0.001 else lin_vel

        if keys[pygame.K_a]:    # left
            ang_vel = 0.0 if ang_vel < 0.0 else ang_vel
            ang_vel += self.ang_a * self.dt  * k
        elif keys[pygame.K_d]:  # right
            ang_vel = 0.0 if ang_vel > 0.0 else ang_vel
            ang_vel -= self.ang_a * self.dt  * k
        else:
            ang_vel /= 1.5
            ang_vel = 0.0 if abs(ang_vel) < 0.001 else ang_vel

        if keys[pygame.K_SPACE]:
            curr_cmd = Twist()

        lin_vel, ang_vel = self.clip_velocities(lin_vel, ang_vel)

        print(lin_vel, ang_vel)
        curr_cmd.linear.x = lin_vel
        curr_cmd.angular.z = ang_vel

        return curr_cmd

    # (c)pizheno https://github.com/FastSense/tankbot-rc/blob/refactoring_to_class/tankbot_joystick.py#L54
    def process_joystick_input(self, msg, curr_cmd):
        """Callback for joystick input"""

        # Reducing sensitivity of angular velocity control by joystick
        JOYSTICK_ANGULAR_SCALER = 0.6
        JOYSTICK_AXIS_LINEAR_VEL = 2     # 1
        JOYSTICK_AXIS_ANGULAR_VEL = 1    # 0
        JOYSTICK_AXIS_HEAD_YAW = 3       # 2
        JOYSTICK_AXIS_HEAD_PITCH = 0     # 3

        # Linear velocity axis
        # lin = msg.axes[1]
        lin_vel = -msg.axes[JOYSTICK_AXIS_LINEAR_VEL]
        # Angular velocity axis
        # ang = msg.axes[0] * JOYSTICK_ANGULAR_SCALER
        ang_vel = msg.axes[JOYSTICK_AXIS_ANGULAR_VEL] * JOYSTICK_ANGULAR_SCALER

        # Exponential scaling for input
        kl = 1
        ka = 1
        if lin_vel < 0:
            lin = abs(lin_vel)
            kl = -1
        if ang_vel < 0:
            ang = abs(ang_vel)
            ka = -1

        # Exponential scaling for inputs
        lin_vel = kl * (np.exp(lin_vel) - 1) / (np.e - 1)
        ang_vel = ka * (np.exp(ang) - 1) / (np.e - 1)
        lin_vel, ang_vel = self.clip_velocities(lin_vel, ang_vel)

        curr_cmd.linear.x = lin_vel
        curr_cmd.angular.z = ang_vel
        return curr_cmd
        # with self.head_cmd_lock:
        #     self.head_cmd.angular.x = (msg.axes[JOYSTICK_AXIS_HEAD_YAW] + 1) / 2
        #     self.head_cmd.angular.y = (-msg.axes[JOYSTICK_AXIS_HEAD_PITCH] + 1) / 2



    def clip_velocities(self, lin_vel, ang_vel):
        """
        """

        lin_vel = np.clip(
            lin_vel,
            a_min=-self.v_limit,
            a_max=self.v_limit
        )
        ang_vel= np.clip(
            ang_vel,
            a_min=-self.w_limit, 
            a_max=self.w_limit
        )
        lin_vel = round(lin_vel, 3)
        ang_vel = round(ang_vel, 3)

        return lin_vel, ang_vel