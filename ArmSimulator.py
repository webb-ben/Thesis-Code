import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Arm import *

class ArmSimulator(Arm):
    def __init__(self, port_handler, packet_handler):
        self.dhp = np.array([[0.0, 0.0, 10.5, -0.4502608370048966],
                  [0.0, 22.0, 0.0,  2.0057073607199998],
                  [np.pi / 2, 24.5, 0.0,  0.2558300195751033],
                  [-np.pi / 2, 7, -1.5, -0.020466402564896624],
                  [0.0, 5.0, -8.5, 0.0]])
        self.init_vis()
        self.set_thetas(self.get_thetas())
        try:
            Arm.__init__(self, port_handler, packet_handler)
        except AttributeError:
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
        color_codes = ('#DAF7A6', '#FFC300', '#FF5733', '#C70039', '#900C3F', '#FF1845', '#FF3214')
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
        # plt.show()


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

        self.ep.scatter(*self.get_xy(), color='#B0B0B0', s=1)
        self.ax.scatter(*self.get_xy(), self.t0[-1][2][3], color='#B0B0B0', s=1)

        plt.pause(0.005)

    def set_thetas(self, position=(0, 0, 0, 0), wait=False):
        a1, a2, a3, a4 = position
        # print (position)
        self.dhp[:4, 3] = (np.mat([a1, a2, a3, a4]))
        self.tf()

    def get_thetas(self):
        return self.dhp[0, 3], self.dhp[1, 3], self.dhp[2, 3], self.dhp[3, 3]

    def get_positions(self):
        return [self.get_position(i) for i in range(1,5)]

    def get_position(self, i):
        theta = 0 if i == 2 else np.pi/2
        return self.theta_to_pos(self.dhp[i-1, 3], theta)

    def set_positions(self, position=(512, 512, 512, 512), wait=False):
        [self.set_position(i+1, position[i]) for i in range(len(position))]
        self.tf()

    def set_position(self, i, position=512, wait=False):
        theta = 0 if i == 2 else np.pi/2
        self.dhp[i - 1, 3] = self.pos_to_theta(position, theta)

    def tf(self):
        for i in range(self.dhp.shape[0]):
            # Create joint transformation from DH Parameters
            self.t0[i] = np.mat([(np.cos(self.dhp[i,3]), -np.sin(self.dhp[i,3]), 0, self.dhp[i,1]),
                                 (np.sin(self.dhp[i,3])*np.cos(self.dhp[i, 0]), np.cos(self.dhp[i,3])*np.cos(self.dhp[i,0]),
                                  -np.sin(self.dhp[i,0]), -self.dhp[i, 2] * np.sin(self.dhp[i, 0])),
                                 (np.sin(self.dhp[i,3]) * np.sin(self.dhp[i, 0]), np.cos(self.dhp[i, 3]) * np.sin(self.dhp[i, 0]),
                                  np.cos(self.dhp[i,0]), self.dhp[i, 2] * np.cos(self.dhp[i, 0])),
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

    def get_ea(self):
        return self.t0[-1][0][3], self.t0[-1][1][3], self.t0[-1][1][3]
