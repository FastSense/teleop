import numpy as np
import pygame
from geometry_msgs.msg import Twist



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





    def keyboard_cotrol_callback(self, keys, curr_cmd):
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
        print(lin_vel, ang_vel)
        curr_cmd.linear.x = lin_vel
        curr_cmd.angular.z = ang_vel

        return curr_cmd