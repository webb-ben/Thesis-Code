# Arm.py
# Spring 2020
# Ben Webb

import numpy as np

import data
import time
from Motor import *


def inverse_jacobian(a1, a2, a3, a4, x, y, z, Ya):
    return np.linalg.inv(jacobian(a1, a2, a3, a4)) \
           * np.mat([x, y, z, Ya]).T


def jacobian(a1, a2, a3, a4):
    return np.matrix([[-22.3 * np.sin(a1) - 25.3 * np.sin(a1 + a2) - 6.5 * np.sin(a1 + a2) * np.cos(a3) + 17 * np.sin(
        a1 + a2) * np.sin(a3) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6 * np.cos(a1 + a2) * np.sin(a4),
                       -25.3 * np.sin(a1 + a2) - 6.5 * np.sin(a1 + a2) * np.cos(a3) + 17 * np.sin(a1 + a2) * np.sin(
                           a3) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - np.cos(a1 + a2) * np.sin(a4),
                       -6.5 * np.cos(a1 + a2) * np.sin(a3) - 17 * np.cos(a1 + a2) * np.cos(a3) - 6 * np.cos(
                           a1 + a2) * np.sin(a3) * np.cos(a4),
                       -6 * np.cos(a1 + a2) * np.cos(a3) * np.sin(a4) - 6 * np.sin(a1 + a2) * np.cos(a4)],
                      [22.3 * np.cos(a1) + 25.3 * np.cos(a1 + a2) + 6.5 * np.cos(a1 + a2) * np.cos(a3) - 17 * np.cos(
                          a1 + a2) * np.sin(a3) + 6 * np.cos(a1 + a2) * np.cos(a3) * np.cos(a4) - 6 * np.sin(
                          a1 + a2) * np.sin(a4),
                       25.3 * np.cos(a1 + a2) + 6.5 * np.cos(a1 + a2) * np.cos(a3) - 17 * np.cos(a1 + a2) * np.sin(
                           a3) + 6 * np.cos(a1 + a2) * np.cos(a3) * np.cos(a4) - 6 * np.sin(a1 + a2) * np.sin(a4),
                       -6.5 * np.cos(a1 + a2) * np.sin(a3) - 17 * np.cos(a1 + a2) * np.cos(a3) - 6 * np.cos(
                           a1 + a2) * np.cos(a3) * np.cos(a4),
                       -6 * np.cos(a1 + a2) * np.cos(a3) * np.sin(a4) + 6 * np.cos(a1 + a2) * np.cos(a4)],
                      [0, 0, -6.5 * np.cos(a3) + 17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4),
                       6 * np.sin(a3) * np.sin(a4)],
                      [0, 0, np.cos(a1 + a2), np.sin(a1 + a2) * np.sin(a3)]])

class Arm:
    def __init__(self, port_handler, packet_handler):
        self.port_handler = port_handler
        self.motor_list = [Motor(1, port_handler, packet_handler, 255, 767, 80, 12, 128),
                           Motor(2, port_handler, packet_handler, 100, 973, 120, 10, 64, 0),
                           Motor(3, port_handler, packet_handler, 105, 800, 180, 10, 32),
                           Motor(4, port_handler, packet_handler, 50, 793, 210, 10, 32)]
        self.bootup()

    def bootup(self):
        self.set_positions(self.get_positions())
        self.set_angular_range()
        self.enable_torque()
        self.set_speed()
        self.set_angular_compliance()
        self.set_angular_slope()

    def enable_torque(self):
        [m.enable_torque() for m in self.motor_list]

    def disable_torque(self):
        [m.disable_torque() for m in self.motor_list]

    def close_connection(self):
        self.set_positions(self.get_positions())
        self.disable_torque()
        self.port_handler.closePort()
        exit()

    def set_speed(self):
        [m.set_speed() for m in self.motor_list]

    def set_angular_compliance(self):
        [m.set_angular_compliance() for m in self.motor_list]

    def set_angular_range(self):
        [m.set_angular_range() for m in self.motor_list]

    def set_angular_slope(self):
        [m.set_angular_slope() for m in self.motor_list]

    def set_position(self, motor, position=512, wait=False):
        motor.set_position(position, wait)

    def set_positions(self, position=(512, 512, 512, 512), wait=False):
        for i in range(len(self.motor_list)):
            self.set_position(self.motor_list[-1 - i], position[-1 - i], wait)

    def set_theta(self, motor, position=0, wait=False):
        motor.set_theta(position, wait)

    def set_thetas(self, position=(0, 0, 0, 0), wait=False):
        for i in range(len(self.motor_list)):
            self.set_theta(self.motor_list[-1 - i], position[-1 - i], wait)

    def get_position(self, id):
        return self.motor_list[id - 1].get_position()

    def get_positions(self):
        return [m.get_position() for m in self.motor_list]

    def get_theta(self, id):
        return self.motor_list[id - 1].get_theta()

    def get_thetas(self):
        return [m.get_theta() for m in self.motor_list]

    def get_transformation(self):
        t1, t2, t3, t4 = self.get_thetas()
        return np.matrix([[-np.sin(t4) * np.sin(t1 + t2) + np.cos(t3) * np.cos(t4) * np.cos(t1 + t2),
                           -np.sin(t4) * np.cos(t3) * np.cos(t1 + t2) - np.sin(t1 + t2) * np.cos(t4),
                           np.sin(t3) * np.cos(t1 + t2),
                           -18.5 * np.sin(t3) * np.cos(t1 + t2) - 6 * np.sin(t4) * np.sin(t1 + t2) + 22.3 * np.cos(
                               t1) + 6 * np.cos(t3) * np.cos(t4) * np.cos(t1 + t2) + 6.5 * np.cos(t3) * np.cos(
                               t1 + t2) + 25.3 * np.cos(t1 + t2)],
                          [np.sin(t4) * np.cos(t1 + t2) + np.sin(t1 + t2) * np.cos(t3) * np.cos(t4),
                           -np.sin(t4) * np.sin(t1 + t2) * np.cos(t3) + np.cos(t4) * np.cos(t1 + t2),
                           np.sin(t3) * np.sin(t1 + t2),
                           22.3 * np.sin(t1) - 18.5 * np.sin(t3) * np.sin(t1 + t2) + 6 * np.sin(t4) * np.cos(
                               t1 + t2) + 6 * np.sin(t1 + t2) * np.cos(t3) * np.cos(t4) + 6.5 * np.sin(
                               t1 + t2) * np.cos(t3) + 25.3 * np.sin(t1 + t2)],
                          [-np.sin(t3) * np.cos(t4), np.sin(t3) * np.sin(t4), np.cos(t3),
                           -6 * np.sin(t3) * np.cos(t4) - 6.5 * np.sin(t3) - 18.5 * np.cos(t3) + 19],
                          [0, 0, 0, 1]])

    def get_jacobian(self):
        return jacobian(*self.get_thetas())

    def get_inv_jacobian(self, x, y, z, Ya):
        return inverse_jacobian(*self.get_thetas(), x, y, z, Ya)

    def get_ea(self):
        return self.get_transformation()[:-1, -1].T.tolist()[0]

    def get_speed(self, id):
        return self.motor_list[id - 1].get_speed()

    def get_speeds(self):
        return [m.get_speed() for m in self.motor_list]

    def get_load(self, id):
        return self.motor_list[id - 1].get_load()

    def get_loads(self):
        return [m.get_load() for m in self.motor_list]

    def move(self, dx, dy, dz, Ya, steps, sleep_time=0.03):
        for m in self.motor_list:
            m.angular_compliance = 1
            m.set_angular_compliance()
        a1, a2, a3, a4 = self.get_thetas()
        pos = self.get_ea()
        x, y, z = pos[0] + dx, pos[1] + dy, pos[2] + dz
        while abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1:
            print(pos)
            pos = self.get_ea()
            dx, dy, dz = x - pos[0], y - pos[1], z - pos[2]
            p = self.get_inv_jacobian(dx / steps, dy / steps, dz / steps, Ya / steps)
            a1, a2, a3, a4 = (np.matrix((a1, a2, a3, a4)).T + p).T.tolist()[0]
            self.set_thetas((a1, a2, a3, a4))
            # time.sleep(sleep_time)

    def move_x(self, distance, steps=300, sleep=0.03):
        a1, a2, a3, a4 = self.get_thetas()
        x, y, z = self.get_ea()
        step_a2, step_x = ((22.3 * a2) - (distance * 0.5)) / (22.3 * steps), distance / steps
        for i in range(steps):
            a2 += step_a2
            x += step_x
            a1 = 2.13668 - a2 + (0.02779 * x)
            self.set_thetas((a1, a2, a3, a4))
            # time.sleep(sleep)

def main():
    arm = Arm()
    try:
        arm.bootup()
        arm.trial()
        arm.close_connection()
    except KeyboardInterrupt:
        arm.close_connection()


if __name__ == "__main__":
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    main()

'''

[19.06116913267098, 41.24101485525948, 1.0946642699830313]
-1cm y
[18.973489142882265, 40.111575539755435, 1.0266920104840551]
+1cm y
[19.07485550239428, 41.12773860859118, 1.0946642699830313]
+1cm x
[19.819376950905944, 41.292583404101556, 1.0945883834606462]
-1cm x
[19.07485550239428, 41.12773860859118, 1.0946642699830313]
+2cm Y
[18.937279524206478, 43.16856674053385, 1.0946642699830313]

'''


def __init__(port_handler, packet_handler):
    return None