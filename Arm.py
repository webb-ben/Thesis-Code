# Arm.py
# Spring 2020
# Ben Webb

import numpy as np
import time
from Motor import *


def inverse_jacobian(a1, a2, a3, a4, x, y, z, Ya):
    return np.linalg.inv(jacobian(a1, a2, a3, a4)) \
           * np.mat([x, y, z, Ya]).T


def jacobian(a1, a2, a3, a4):
    return np.mat([[-22.3 * np.sin(a1) - 25.3 * np.sin(a1 + a2) - 6.5 * np.sin(a1 + a2) * np.cos(a3) + 17 * np.sin(
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
        self.packet_handler = packet_handler
        self.motor_list = [Motor(1, port_handler, packet_handler, 255, 767, 50, 1, 16, 24),
                           Motor(2, port_handler, packet_handler, 200, 973, 50, 1, 12, 8, 0),
                           Motor(3, port_handler, packet_handler, 50, 800, 50, 1, 64, 64),
                           Motor(4, port_handler, packet_handler, 200, 793, 210, 1, 2, 2)]

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

    def get_transformation(self):
        t1, t2, t3, t4 = self.get_thetas()
        return np.mat([[-np.sin(t4) * np.sin(t1 + t2) + np.cos(t3) * np.cos(t4) * np.cos(t1 + t2),
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

    def get_inv_jacobian(self, x, y, z, Ya):
        return inverse_jacobian(*self.get_thetas(), x, y, z, Ya)

    def get_ea(self):
        return self.get_transformation()[:-1, -1].T.tolist()[0]

    def get_xy(self):
        t1, t2, = self.motor_list[0].get_theta(), self.motor_list[1].get_theta()
        return 22.2*np.cos(t1) + 36.5*np.cos(t1 + t2), 22.2*np.sin(t1) + 36.5*np.sin(t1 + t2)

    def get_speed(self, i):
        return self.motor_list[i - 1].get_speed()

    def get_speeds(self):
        return [m.get_speed() for m in self.motor_list]

    def get_load(self, i):
        return self.motor_list[i - 1].get_load()

    def get_loads(self):
        return [m.get_load() for m in self.motor_list]

    def move(self, dx, dy, dz, Ya, steps, sleep_time=0.0025):
        for m in self.motor_list:
            m.angular_compliance = 1
            m.set_angular_compliance()
        a1, a2, a3, a4 = self.get_thetas()
        pos = self.get_ea()
        x, y, z = pos[0] + dx, pos[1] + dy, pos[2] + dz
        while abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1:
            pos = self.get_ea()
            dx, dy, dz = x - pos[0], y - pos[1], z - pos[2]
            p = self.get_inv_jacobian(dx / steps, dy / steps, dz / steps, Ya / steps)
            a1, a2, a3, a4 = (np.mat((a1, a2, a3, a4)).T + p).T.tolist()[0]
            self.set_thetas((a1, a2, a3, a4))
            time.sleep(sleep_time)

    def move_x(self, distance, steps=300, sleep=0.03):
        a1, a2, a3, a4 = self.get_thetas()
        x, y, z = self.get_ea()
        step_a2, step_x = ((22.3 * a2) - (distance * 0.5)) / (22.3 * steps), distance / steps
        for i in range(steps):
            a2 += step_a2
            x += step_x
            a1 = 2.13668 - a2 + (0.02779 * x)
            self.set_thetas((a1, a2, a3, a4))

    # def pressure(self, force = 512, position = 512):
        # m = self.motor_list[2]
        # m.set_maximum_torque(force)
        # m.set_position(position, False)

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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ArmSimulator(Arm):
    def __init__(self, port_handler, packet_handler):
        self.dhp = np.mat([[0, 0, 18.5, 0.1381482112],
                           [0, 22.3, 0, 1.5],
                           [-np.pi / 2, 25.3, 0, 0],
                           [np.pi / 2, 6.5, -1.5, 0],
                           [0, 6, 0, 0],
                           [0, 0, -15.5, 0]])
        self.init_vis()
        self.set_thetas(self.get_thetas())
        try:
            Arm.__init__(self, port_handler, packet_handler)
        except:
            self.motor_list = []

    def init_vis(self):
        self.t0 = np.empty((self.dhp.shape[0], 4, 4))
        self.z0 = np.empty((self.dhp.shape[0], 4, 4))  # Z-axis matrices for each joint
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(211, projection='3d')
        self.ep = self.fig.add_subplot(212, aspect="equal")
        # ax.view_init(50, azim=90)

        # Label the figure and axes (including units)
        self.ax.set_title("Arm Dynamics")
        self.ax.set_xlabel("X0 (cm)")
        self.ax.set_ylabel("Y0 (cm)")
        self.ax.set_zlabel("Z0 (cm)")

        # Fix axis limits so that they're the same size regardless of
        # the configuration of the arm
        self.ax.set_xlim((0, 50))
        self.ax.set_ylim((-50, 50))
        self.ax.set_zlim((0, 20))
        self.ep.set_xlim((0, 50))
        self.ep.set_ylim((0, 50))
        self.arm_objects = [[], []]
        color_codes = ('#DAF7A6', '#FFC300', '#FF5733', '#C70039', '#900C3F', '#581845')
        for i in range(1, self.t0.shape[0]):
            self.arm_objects[0].extend(
                self.ax.plot([self.t0[i - 1][0][3], self.t0[i][0][3]], [self.t0[i - 1][1][3], self.t0[i][1][3]],
                             [self.t0[i - 1][2][3], self.t0[i][2][3]],
                             'o-', linewidth=5, color=color_codes[-i - 1]))

        for i in range(self.t0.shape[0]):
            self.arm_objects[1].extend(
                self.ax.plot([self.t0[i][0][3], self.z0[i][0][3]], [self.t0[i][1][3], self.z0[i][1][3]],
                             [self.t0[i][2][3], self.z0[i][2][3]], '-',
                             markersize=5, linewidth=1, label="z{0}".format(i + 1), color=color_codes[-i - 1]))
        plt.ion()
        plt.show()


    def visualizeArm(self):
        '''Draw a stick figure of the the arm in its current configuration.

        t0 is a numpy ndarray of shape (N, 4, 4), where N is the number of
        degrees of freedom. t0[i][:][:] is the transformation
        matrix describing the position and orientation of frame i in frame 0.

        Similarly, z0 is a numpy ndarray of shape (N, 4, 4), where N is the
        number of degrees of freedom. z0[i][:][:] is the transformation
        matrix describing the position and orientation of the end of the Zi
        axis in frame 0.

        All angles must be in radians and all distances must be in cm.'''

        # Draw the links of the arm

        for i, line in zip(range(1, self.t0.shape[0]), self.arm_objects[0]):
            line.set_data([self.t0[i - 1][0][3], self.t0[i][0][3]], [self.t0[i - 1][1][3], self.t0[i][1][3]])
            line.set_3d_properties([self.t0[i - 1][2][3], self.t0[i][2][3]])

        for i, line in zip(range(self.t0.shape[0]), self.arm_objects[1]):
            line.set_data([self.t0[i][0][3], self.z0[i][0][3]], [self.t0[i][1][3], self.z0[i][1][3]])
            line.set_3d_properties([self.t0[i][2][3], self.z0[i][2][3]])

        self.ep.scatter(self.t0[-1][0][3], self.t0[-1][1][3], color='#B0B0B0', s=1)
        self.ax.scatter(self.t0[-1][0][3], self.t0[-1][1][3], self.t0[-1][2][3], color='#B0B0B0', s=1)

        plt.pause(0.01)

    def set_thetas(self, position=(0, 0, 0, 0), wait=False):
        a1, a2, a3, a4 = position
        # print (position)
        self.dhp[:, 3] = (np.mat([a1, a2, a3, a4, 0, 0])).T

        for i in range(self.dhp.shape[0]):
            # Create joint transformation from DH Parameters
            self.t0[i] = np.mat([(np.cos(self.dhp[i, 3]), -np.sin(self.dhp[i, 3]), 0, self.dhp[i, 1]),
                                 (np.sin(self.dhp[i, 3]) * np.cos(self.dhp[i, 0]), np.cos(self.dhp[i, 3]) * np.cos(self.dhp[i, 0]),
                                  -np.sin(self.dhp[i, 0]), -self.dhp[i, 2] * np.sin(self.dhp[i, 0])),
                                 (np.sin(self.dhp[i, 3]) * np.sin(self.dhp[i, 0]), np.cos(self.dhp[i, 3]) * np.sin(self.dhp[i, 0]),
                                  np.cos(self.dhp[i, 0]), self.dhp[i, 2] * np.cos(self.dhp[i, 0])),
                                 (0, 0, 0, 1)])

            # Put each transformation in the frame 0
            if i != 0:
                self.t0[i] = np.matmul(self.t0[i - 1], self.t0[i])

            # Compute Z axis in frame 0 for each joint
            self.z0[i] = np.matmul(self.t0[i], np.mat([[1, 0, 0, 0],
                                                       [0, 1, 0, 0],
                                                       [0, 0, 1, 5],
                                                       [0, 0, 0, 1]]))
        # self.visualizeArm()

    def get_thetas(self):
        return self.dhp[0, 3], self.dhp[1, 3], self.dhp[2, 3], self.dhp[3, 3]

    def close_connection(self):
        return

    def get_ea(self):
        return self.t0[-1][0][3], self.t0[-1][1][3], self.t0[-1][1][3]

    def get_xy(self):
        return self.t0[-1][0][3], self.t0[-1][1][3]