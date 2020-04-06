# Arm.py
# Spring 2020
# Ben Webb

import numpy as np
from numpy import cos as c  # for trig functions (cos and sin)
from numpy import sin as s
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting
import Arm

plt.ion()

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


class Arm_Simulator(Arm.Arm):
    def __init__(self, port_handler, packet_handler):
        self.dhp = np.mat([[0, 0, 18.5, 0.1381482112],
                      [0, 22.3, 0, 1.34],
                      [-np.pi / 2, 25.3, 0, -0.1841976149],
                      [np.pi / 2, 6.5, -1.5, 0.08186560661],
                      [0, 6, -15.5, 0]])
        self.t0 = np.empty((self.dhp.shape[0], 4, 4))
        self.init_vis()
        self.set_thetas(self.get_thetas())
        try:
            Arm.Arm.__init__(port_handler, packet_handler)
        except:
            self.motor_list = []

    def init_vis(self):
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
        self.ax.set_xlim((-50, 50))
        self.ax.set_ylim((-50, 50))
        self.ax.set_zlim((-50, 50))
        self.ep.set_xlim((0, 50))
        self.ep.set_ylim((0, 50))
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

        self.ep.scatter(self.t0[-1][0][3], self.t0[-1][1][3], color='#B0B0B0', s=1)

        l = []
        # Draw the links of the arm
        for i in range(1, self.t0.shape[0]):
            l.extend(
                self.ax.plot([self.t0[i - 1][0][3], self.t0[i][0][3]], [self.t0[i - 1][1][3], self.t0[i][1][3]], [self.t0[i - 1][2][3], self.t0[i][2][3]],
                        'o-',
                        color='#B0B0B0', linewidth=5))

        self.fig.canvas.draw()

        plt.pause(.01)
        for i in l:
            i.remove()

    def set_thetas(self, position=(0, 0, 0, 0), wait=False):
        a1, a2, a3,a4 = position
        # print (position)
        self.dhp[:, 3] = (np.mat([a1, a2, a3, a4, 0])).T
        for i in range(self.dhp.shape[0]):
            # Create joint transformation from DH Parameters

            self.t0[i] = np.mat([(c(self.dhp[i, 3]), -s(self.dhp[i, 3]), 0, self.dhp[i, 1]),
                            (s(self.dhp[i, 3]) * c(self.dhp[i, 0]), c(self.dhp[i, 3]) * c(self.dhp[i, 0]), -s(self.dhp[i, 0]),
                             -self.dhp[i, 2] * s(self.dhp[i, 0])),
                            (s(self.dhp[i, 3]) * s(self.dhp[i, 0]), c(self.dhp[i, 3]) * s(self.dhp[i, 0]), c(self.dhp[i, 0]),
                             self.dhp[i, 2] * c(self.dhp[i, 0])),
                            (0, 0, 0, 1)])
            # Put each transformation in the frame 0
            if i != 0: self.t0[i] = np.matmul(self.t0[i - 1], self.t0[i])
        self.visualizeArm()

    def get_thetas(self):
        return self.dhp[0, 3], self.dhp[1, 3], self.dhp[2, 3],self.dhp[3, 3]

    def close_connection(self):
        return

    def get_ea(self):
        return self.t0[-1][0][3],  self.t0[-1][1][3], self.t0[-1][1][3]
