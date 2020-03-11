'''
Lab 2: Dynamics
CS442 Spring 2019
Caitrin Eaton
Modified by Ben Webb
'''

import sys  # for command line arguments
from numpy import cos as c # for trig functions (cos and sin)
from numpy import sin as s
import numpy as np  # for matrix math
import matplotlib.pyplot as plt  # for visualization
import time
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting

ALPHA = 0
A = 1
D = 2
THETA = 3
np.set_printoptions(precision=5, suppress=True)
plt.ion()

def visualizeArm(t0, z0=None, ax=None, ep=None, fig=None):
    '''Draw a stick figure of the the arm in its current configuration.

    t0 is a numpy ndarray of shape (N, 4, 4), where N is the number of
    degrees of freedom. t0[i][:][:] is the transformation
    matrix describing the position and orientation of frame i in frame 0.

    Similarly, z0 is a numpy ndarray of shape (N, 4, 4), where N is the
    number of degrees of freedom. z0[i][:][:] is the transformation
    matrix describing the position and orientation of the end of the Zi
    axis in frame 0.

    All angles must be in radians and all distances must be in cm.'''

    # If no existing axis was given as a parameter, then create a new figure
    if ax == None:
        # Create a new figure and configure the axes for 3D plotting
        fig = plt.figure()
        ax = fig.add_subplot(211,  projection='3d')
        ep = fig.add_subplot(212)
        # ax.view_init(50, azim=90)

        # Label the figure and axes (including units)
        ax.set_title("Arm Dynamics")
        ax.set_xlabel("X0 (cm)")
        ax.set_ylabel("Y0 (cm)")
        ax.set_zlabel("Z0 (cm)")

        # Fix axis limits so that they're the same size regardless of
        # the configuration of the arm
        ax.set_xlim((-50, 50))
        ax.set_ylim((-50, 50))
        ax.set_zlim((-50, 50))
        ep.set_xlim((0, 50))
        ep.set_ylim((0, 50))

    else:
        print(t0[-1][0][3], t0[-1][1][3])
        ep.scatter(t0[-1][0][3], t0[-1][1][3], color='#B0B0B0', s=1)


    l = []
    # Draw the links of the arm
    for i in range(1, t0.shape[0]):
        l.extend(ax.plot([t0[i - 1][0][3], t0[i][0][3]], [t0[i - 1][1][3], t0[i][1][3]], [t0[i - 1][2][3], t0[i][2][3]], 'o-',
                color='#B0B0B0', linewidth=5))

    if z0 is not None:
        # Draw the Z axis for each joint, if they were provided
        for i in range(t0.shape[0]):
            l.extend(ax.plot([t0[i][0][3], z0[i][0][3]], [t0[i][1][3], z0[i][1][3]], [t0[i][2][3], z0[i][2][3]], 'o-',
                    markersize=5, linewidth=1, label="z{0}".format(i + 1)))


    fig.canvas.draw()
    plt.pause(.01)
    for i in l:
        i.remove()
    return (fig, ax, ep)

def get_transformation(t1,t2,t3,t4):
        return np.mat([[-np.sin(t4)*np.sin(t1+t2)+np.cos(t3)*np.cos(t4)*np.cos(t1+t2), -np.sin(t4)*np.cos(t3)*np.cos(t1+t2)-np.sin(t1+t2)*np.cos(t4), np.sin(t3)*np.cos(t1+t2), -18.5*np.sin(t3)*np.cos(t1+t2)-6*np.sin(t4)*np.sin(t1+t2)+22.3*np.cos(t1)+6*np.cos(t3)*np.cos(t4)*np.cos(t1+t2)+6.5*np.cos(t3)*np.cos(t1+t2)+25.3*np.cos(t1+t2)],
                          [np.sin(t4)*np.cos(t1+t2)+np.sin(t1+t2)*np.cos(t3)*np.cos(t4), -np.sin(t4)*np.sin(t1+t2)*np.cos(t3)+np.cos(t4)*np.cos(t1+t2), np.sin(t3)*np.sin(t1+t2), 22.3*np.sin(t1)-18.5*np.sin(t3)*np.sin(t1+t2)+6*np.sin(t4)*np.cos(t1+t2)+6*np.sin(t1+t2)*np.cos(t3)*np.cos(t4)+6.5*np.sin(t1+t2)*np.cos(t3)+25.3*np.sin(t1+t2)],
                          [-np.sin(t3)*np.cos(t4), np.sin(t3)*np.sin(t4), np.cos(t3), -6*np.sin(t3)*np.cos(t4)-6.5*np.sin(t3)-18.5*np.cos(t3)+19],
                          [0, 0, 0, 1]])

def inverse_jacobian(a1,a2,a3,a4,x,y,z,Y):
    jac = np.linalg.inv(np.mat([[-22.3 * s(a1) - 25.3 * s(a1 + a2) - 6.5 * s(a1 + a2) * c(a3) + 17 * s(a1 + a2) * s(a3) - 6 * s(
        a1 + a2) * c(a3) * c(a4) - 6 * c(a1 + a2) * s(a4),
                   -25.3 * s(a1 + a2) - 6.5 * s(a1 + a2) * c(a3) + 17 * s(a1 + a2) * s(a3) - 6 * s(a1 + a2) * c(a3) * c(
                       a4) - c(a1 + a2) * s(a4),
                   -6.5 * c(a1 + a2) * s(a3) - 17 * c(a1 + a2) * c(a3) - 6 * c(a1 + a2) * s(a3) * c(a4),
                   -6 * c(a1 + a2) * c(a3) * s(a4) - 6 * s(a1 + a2) * c(a4)],
                  [22.3 * c(a1) + 25.3 * s(a1 + a2) + 6.5 * c(a1 + a2) * c(a3) - 17 * c(a1 + a2) * s(a3) + 6 * c(
                      a1 + a2) * c(a3) * c(a4) - 6 * s(a1 + a2) * s(a4),
                   25.3 * s(a1 + a2) + 6.5 * c(a1 + a2) * c(a3) - 17 * c(a1 + a2) * s(a3) + 6 * c(a1 + a2) * c(a3) * c(
                       a4) - 6 * s(a1 + a2) * s(a4),
                   -6.5 * c(a1 + a2) * s(a3) - 17 * c(a1 + a2) * c(a3) - 6 * c(a1 + a2) * c(a3) * c(a4),
                   -6 * c(a1 + a2) * c(a3) * s(a4) + 6 * c(a1 + a2) * c(a4)],
                  [0, 0, -6.5 * c(a3) + 17 * s(a3) - 6 * c(a3) * c(a4), 6 * s(a3) * s(a4)],
                  [0, 0, c(a1 + a2), s(a1 + a2) * s(a3)]]))
    return (jac * np.mat([x,y,z,Y]).T)

def jacobian(a1,a2,a3,a4,t1,t2,t3,t4):
    return (np.mat([[-22.3 * s(a1) - 25.3 * s(a1 + a2) - 6.5 * s(a1 + a2) * c(a3) + 17 * s(a1 + a2) * s(a3) - 6 * s(
        a1 + a2) * c(a3) * c(a4) - 6 * c(a1 + a2) * s(a4),
                   -25.3 * s(a1 + a2) - 6.5 * s(a1 + a2) * c(a3) + 17 * s(a1 + a2) * s(a3) - 6 * s(a1 + a2) * c(a3) * c(
                       a4) - c(a1 + a2) * s(a4),
                   -6.5 * c(a1 + a2) * s(a3) - 17 * c(a1 + a2) * c(a3) - 6 * c(a1 + a2) * s(a3) * c(a4),
                   -6 * c(a1 + a2) * c(a3) * s(a4) - 6 * s(a1 + a2) * c(a4)],
                  [22.3 * c(a1) + 25.3 * s(a1 + a2) + 6.5 * c(a1 + a2) * c(a3) - 17 * c(a1 + a2) * s(a3) + 6 * c(
                      a1 + a2) * c(a3) * c(a4) - 6 * s(a1 + a2) * s(a4),
                   25.3 * s(a1 + a2) + 6.5 * c(a1 + a2) * c(a3) - 17 * c(a1 + a2) * s(a3) + 6 * c(a1 + a2) * c(a3) * c(
                       a4) - 6 * s(a1 + a2) * s(a4),
                   -6.5 * c(a1 + a2) * s(a3) - 17 * c(a1 + a2) * c(a3) - 6 * c(a1 + a2) * c(a3) * c(a4),
                   -6 * c(a1 + a2) * c(a3) * s(a4) + 6 * c(a1 + a2) * c(a4)],
                  [0, 0, -6.5 * c(a3) + 17 * s(a3) - 6 * c(a3) * c(a4), 6 * s(a3) * s(a4)],
                  [0, 0, c(a1 + a2), s(a1 + a2) * s(a3)]])* np.matrix([t1,t2,t3,t4]).T).T

def main(argv):
    '''Take in 3 joint angles and calculate the end effector position
       for the modified DH parameters through transformation.
       Also find the Jacobian matrix and its determinant for the 6th
       transformation to calculate the angular torque using input forces in XYZ directions.
       Using the torque then find the actual end effector force
    '''
    # Case 1 : DH-Parameters
    dhp = np.mat([[0, 0, 18.5, 0],
                  [0, 22.3, 0, 2],
                  [-np.pi / 2, 25.3, 0, 0],
                  [np.pi / 2, 6.5, -1.5, 0.0460494],
                  [0, 6, -15.5, 0]])

    # Copy joint angles that were provided as command line arguments
    # into the table of DH parameters and forces into a 1x3 Matrix.
    z0 = np.empty((dhp.shape[0], 4, 4))  # Z-axis matrices for each joint
    (fig, ax, ep) = visualizeArm(np.empty((1,1,1)))
    plt.show()
    # for j in range(10):
    #     t0 = np.empty((dhp.shape[0], 4,4))
    #     a1, a2, a3, a4 = inverse_jacobian(dhp[0, -1], dhp[1, -1], dhp[2, -1], dhp[3, -1], 0, -1, 0, 0)
    #     dhp[:, 3] = dhp[:, 3] + (np.matrix([a1, a2, a3, a4, 0])).T
    #     # Compute the forward kinematics of the arm, finding the orientation
    #     # and position of each joint's frame in the base frame (X0, Y0, Z0).
    #     for i in range(dhp.shape[0]):
    #         # Create joint transformation from DH Parameters
    #
    #         t0[i] = np.mat([(c(dhp[i,3]), -s(dhp[i,3]), 0, dhp[i,1]),
    #                         (s(dhp[i,3]) * c(dhp[i,0]), c(dhp[i,3]) * c(dhp[i,0]), -s(dhp[i,0]),-dhp[i,2] * s(dhp[i, 0])),
    #                         (s(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,0]), dhp[i,2] * c(dhp[i, 0])),
    #                         (0, 0, 0, 1)])
    #
    #         # Put each transformation in the frame 0
    #         if i != 0: t0[i] = np.matmul(t0[i-1],t0[i])
    #
    #         # Compute Z axis in frame 0 for each joint
    #         z0[i] = np.matmul(t0[i],np.matrix([[1, 0, 0, 0],
    #                            [0, 1, 0, 0],
    #                            [0, 0, 1, 15],
    #                            [0, 0, 0, 1]]))
    #     np.savetxt(sys.stdout.buffer, np.mat(t0[-1,:-1,-1]), fmt="%.5f")
    #     print(a1, a2, a3, a4)
    #     (fig, ax) = visualizeArm(t0, ax=ax, fig=fig)
    #     # print (jacobian(dhp[0,-1],dhp[1,-1],dhp[2,-1],dhp[3,-1],0,0,0,0))
    steps = 100
    a1, a2, a3, a4 = dhp[:4, 3]

    step_x = 20 / (steps)
    for j in range(steps):
        # a2 += -np.sin(0.00661375661 * np.pi * step_x)
        # a1 += (-0.027 * step_x) + np.sin(0.00661375661 * np.pi * step_x)
        a2 += -2.854 * np.sin(0.0071 * np.pi * step_x)
        a1 += 0.0468 * step_x
        dhp[:, 3] = (np.mat([a1,a2, a3, a4, 0])).T

        # print(a1,a2,a3,a4,x)
        # print(*get_transformation(a1, a2, a3, a4)[:-1, -1].T)

        t0 = np.empty((dhp.shape[0], 4,4))

        for i in range(dhp.shape[0]):
            # Create joint transformation from DH Parameters

            t0[i] = np.mat([(c(dhp[i,3]), -s(dhp[i,3]), 0, dhp[i,1]),
                            (s(dhp[i,3]) * c(dhp[i,0]), c(dhp[i,3]) * c(dhp[i,0]), -s(dhp[i,0]),-dhp[i,2] * s(dhp[i, 0])),
                            (s(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,0]), dhp[i,2] * c(dhp[i, 0])),
                            (0, 0, 0, 1)])

            # Put each transformation in the frame 0
            if i != 0: t0[i] = np.matmul(t0[i-1], t0[i])

            # Compute Z axis in frame 0 for each joint
            z0[i] = np.matmul(t0[i], np.matrix([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 15],
                               [0, 0, 0, 1]]))

        (fig, ax, ep) = visualizeArm(t0, ax=ax, fig=fig, ep=ep)




if __name__ == "__main__":
    main(sys.argv)