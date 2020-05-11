# Arm.py
# Spring 2020
# Ben Webb

import numpy as np
from Motor import Motor
import time


import datetime
import data
import sys, tty, termios
# fd = sys.stdin.fileno()
# old_settings = termios.tcgetattr(fd)
# def getch():
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

def inverse_jacobian(a1, a2, a3, a4, x, y, z):
    return np.linalg.pinv(jacobian(a1, a2, a3, a4)) \
           @ np.array([x, y, z]).T


def jacobian(a1, a2, a3, a4):
    return np.array([
        [-10*np.sin(a3)*np.sin(a1+a2)-5*np.sin(a4)*np.cos(a1+a2)-22*np.sin(a1)-5*np.cos(a3)*np.cos(a4)*np.sin(a1+a2)-7*np.cos(a3)*np.sin(a1+a2)-24.5*np.sin(a1+a2),
         -10*np.sin(a3)*np.sin(a1+a2)-5*np.sin(a4)*np.cos(a1+a2)-5*np.cos(a3)*np.cos(a4)*np.sin(a1+a2)-7*np.cos(a3)*np.sin(a1+a2)-24.5*np.sin(a1+a2),
         10*np.cos(a3)*np.cos(a1+a2)-5*np.sin(a3)*np.cos(a4)*np.cos(a1+a2)-7*np.sin(a3)*np.cos(a1+a2),
         -5*np.cos(a4)*np.sin(a1+a2)-5*np.cos(a3)*np.sin(a4)*np.cos(a1+a2)],
        [22*np.cos(a1)+10*np.sin(a3)*np.cos(a1+a2)-5*np.sin(a4)*np.sin(a1+a2)+5*np.cos(a1+a2)*np.cos(a3)*np.cos(a4)+7*np.cos(a1+a2)*np.cos(a3)+24.5*np.cos(a1+a2),
         10*np.sin(a3)*np.cos(a1+a2)-5*np.sin(a4)*np.sin(a1+a2)+5*np.cos(a1+a2)*np.cos(a3)*np.cos(a4)+7*np.cos(a1+a2)*np.cos(a3)+24.5*np.cos(a1+a2),
         10*np.cos(a3)*np.sin(a1+a2)+5*np.cos(a3)*np.cos(a1+a2)*np.cos(a4)-7*np.sin(a3)*np.sin(a1+a2),
         5*np.cos(a4)*np.cos(a1+a2)-5*np.sin(a1+a2-5*np.sin(a1+a2)*np.cos(a3)*np.cos(a4))],
        [0,
         0,
         5*np.cos(a3)*np.cos(a4)+7*np.cos(a3)+10*np.sin(a3),
         -5*np.sin(a3)*np.sin(a4)]
    ])


class Arm:
    def __init__(self, port_handler, packet_handler):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.motor_list = [Motor(1, port_handler, packet_handler, 355, 760, 240, 1, 32, 32),
                           Motor(2, port_handler, packet_handler, 206, 660, 240, 1, 32, 32, 0),
                           Motor(3, port_handler, packet_handler, 50, 800, 64, 2, 8, 8),
                           Motor(4, port_handler, packet_handler, 200, 793, 128, 1, 32, 32)]
        self.boot_up()

    def boot_up(self):
        self.set_positions(self.get_positions())
        self.set_angular_range()
        self.set_speed()
        self.set_angular_compliance()
        self.set_angular_slope()
        self.enable_torque()

    def enable_torque(self):
        [m.enable_torque() for m in self.motor_list]

    def disable_torque(self):
        [m.disable_torque() for m in self.motor_list]

    def close_connection(self):
        self.set_positions(self.get_positions())
        self.disable_torque()
        self.port_handler.closePort()
        exit()

    def theta_to_pos(self, theta, theta_shift):
        return (205 + (theta + theta_shift) * 195.442270117)

    def pos_to_theta(self, pos, theta_shift):
        return (pos - 205) * 0.00511660041 - theta_shift

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
        [self.set_position(self.motor_list[-1 - i], position[-1 - i], wait) for i in range(len(self.motor_list))]

    def set_theta(self, motor, position=0, wait=False):
        motor.set_theta(position, wait)

    def set_thetas(self, position=(0, 0, 0, 0), wait=False):
        [self.set_theta(self.motor_list[-1 - i], position[-1 - i], wait) for i in range(len(self.motor_list))]

    def get_position(self, i):
        return self.motor_list[i - 1].get_position()

    def get_positions(self):
        return [m.get_position() for m in self.motor_list]

    def get_theta(self, i):
        return self.motor_list[i - 1].get_theta()

    def get_thetas(self):
        return [m.get_theta() for m in self.motor_list]

    def get_goal_position(self, i):
        return self.motor_list[i - 1].get_goal_pos()

    def get_goal_positions(self):
        return [m.get_goal_pos() for m in self.motor_list]

    def get_transformation(self):
        t1, t2, t3, t4 = self.get_thetas()
        return np.array([[-np.sin(t4) * np.sin(t1 + t2) + np.cos(t3) * np.cos(t4) * np.cos(t1 + t2),
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
                        -6 * np.sin(t3) * np.cos(t4) - 6.5 * np.sin(t3) - 18.5 * np.cos(t3) + 18.5],
                       [0, 0, 0, 1]])


    def get_jacobian(self):
        return jacobian(*self.get_thetas())

    def get_inv_jacobian(self, x, y, z):
        return inverse_jacobian(*self.get_thetas(), x, y, z)

    def get_ea(self):
        return self.get_transformation()[:-1, -1].T.tolist()

    def get_xy(self):
        t1, t2, t3, t4 = self.get_thetas()
        s, c = np.sin, np.cos
        return 10*s(t3)*c(t1+t2)-5*s(t4)*s(t1+t2)+22*c(t1)+5*c(t3)*c(t4)*c(t1+t2)+7*c(t3)*c(t1+t2)+24.5*c(t1 + t2)\
            ,22*s(t1)+10*s(t3)*s(t1+t2)+5*s(t4)*c(t1+t2)+5*s(t1+t2)*c(t3)*c(t4)+7*s(t1+t2)*c(t3)+24.5*s(t1 + t2)
        # return 22.2*np.cos(t1) + 36.5*np.cos(t1 + t2), 22.2*np.sin(t1) + 36.5*np.sin(t1 + t2)

    def get_xyt(self):
        t1, t2, = self.motor_list[0].get_theta(), self.motor_list[1].get_theta()
        return 22.2*np.cos(t1) + 36.5*np.cos(t1 + t2), 22.2*np.sin(t1) + 36.5*np.sin(t1 + t2), t1, t2

    def get_speed(self, i):
        return self.motor_list[i - 1].get_speed()

    def get_speeds(self):
        return [m.get_speed() for m in self.motor_list]

    def get_load(self, i):
        return self.motor_list[i - 1].get_load()

    def get_loads(self):
        return [m.get_load() for m in self.motor_list]

    def move(self, dx, dy):
        a1, a2, a3, a4 = self.get_thetas()
        p = self.get_inv_jacobian(dx, dy)
        a1, a2, a3, a4 = (np.mat((a1, a2, a3, a4)).T + p).T.tolist()[0]
        self.set_thetas((a1, a2, a3, a4))

    # def trial(self, type='Line'):
    #     self.disable_torque()
    #     self.motor_list[2].enable_torque()
    #     self.motor_list[3].enable_torque()
    #     toc = data.Data('TableOfContents.csv')
    #     j = 0
    #     while 1:
    #         c = getch()
    #         if c == chr(0x1b): break
    #         elif c == chr(0x20):
    #             newtrial = data.Data('template.csv')
    #             filename = '%s%.3i.csv' % (type, toc.get_num_points())
    #             print (toc.count_matching('filename', type))
    #             print('Now tracking ' + filename)
    #             i = 0
    #             p0 = self.get_xy()
    #             # for i in range(500):
    #             s1, s2 = self.get_speed(1), self.get_speed(1)
    #             while s1 != 0 or s2 != 0 or i < 200:
    #                 newtrial.addRow((i, self.get_position(1), self.get_position(2), 0, 0, *self.get_xy(), 0, s1, s2, 0, 0,0,0,0,0))
    #                 time.sleep(0.005)
    #                 print (i)
    #                 i += 1
    #                 s1, s2 = self.get_speed(1), self.get_speed(2)
    #             print('Closing ' + filename)
    #             p = self.get_xy()
    #             dist = np.sqrt((p0[0] - p[0])**2 + (p0[1] - p[1])**2)
    #             toc.addRow((filename, '/DataFolder/Data/yPrim/' + filename, datetime.datetime.now(), type, dist, np.arctan2(p0[1]-p[1], p0[0]-p[0]), *p0, 0, *p, 0))
    #             newtrial.write(filename, True, 'yPrim')
    #             toc.write('TableOfContents.csv', False)
    #             print('Done tracking ' + filename)
    #             j += 1