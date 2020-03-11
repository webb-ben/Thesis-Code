'''
Lab 2: Dynamics
CS442 Spring 2019
Caitrin Eaton
Modified by Ben Webb
'''

import sys  # for command line arguments
from math import cos as c # for trig functions (cos and sin)
from math import sin as s
import numpy as np  # for matrix math
import matplotlib.pyplot as plt  # for visualization
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting

ALPHA = 0
A = 1
D = 2
THETA = 3
np.set_printoptions(precision=2, suppress=True)

def visualizeArm(t0, z0=None, ax=None):
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
        ax = fig.add_subplot(111, aspect='equal', projection='3d')

        # Label the figure and axes (including units)
        ax.set_title("Arm Dynamics")
        ax.set_xlabel("X0 (cm)")
        ax.set_ylabel("Y0 (cm)")
        ax.set_zlabel("Z0 (cm)")

        # Fix axis limits so that they're the same size regardless of
        # the configuration of the arm
        ax.set_xlim((-25, 75))
        ax.set_ylim((-50, 50))
        ax.set_zlim((-50, 50))

    # Draw the links of the arm
    for i in range(1, t0.shape[0]):
        ax.plot([t0[i - 1][0][3], t0[i][0][3]], [t0[i - 1][1][3], t0[i][1][3]], [t0[i - 1][2][3], t0[i][2][3]], 'o-',
                color='#B0B0B0', linewidth=5)

    if z0 is not None:
        # Draw the Z axis for each joint, if they were provided
        for i in range(t0.shape[0]):
            ax.plot([t0[i][0][3], z0[i][0][3]], [t0[i][1][3], z0[i][1][3]], [t0[i][2][3], z0[i][2][3]], 'o-',
                    markersize=5, linewidth=1, label="z{0}".format(i + 1))

    ax.legend(loc='center left')

    return (fig, ax)

def main(argv):
    '''Take in 3 joint angles and calculate the end effector position
       for the modified DH parameters through transformation.
       Also find the Jacobian matrix and its determinant for the 6th
       transformation to calculate the angular torque using input forces in XYZ directions.
       Using the torque then find the actual end effector force
    '''
    # The WAM arm's modified DH parameters (leaving thetas as zeros)
    dhp = np.matrix([[0, 0, 20, 0],
                  [0, 22.3, 0, 0],
                  [np.pi / 2, 22.3, 0, 0],
                  [-np.pi / 2, 6.5, -1.5, 0],
                  [0, 5.5, -15.5, 0]])

    # Copy joint angles that were provided as command line arguments
    # into the table of DH parameters and forces into a 1x3 Matrix.
    a1, a2, a3, a4, force_in = np.radians(float(argv[1])), np.radians(float(argv[2])), np.radians(float(argv[3])), np.radians(float(argv[4])), np.mat([float(argv[5]), float(argv[6]), float(argv[7])]).T
    dhp[:, 3] = (np.matrix( [a1,a2,a3,a4,0])).T
    # Compute the forward kinematics of the arm, finding the orientation
    # and position of each joint's frame in the base frame (X0, Y0, Z0).
    t0 = np.empty((dhp.shape[0], 4, 4)) # 4 4x4 matrices with each matrix representing the coordinates in baseframe of the joint
    z0 = np.empty((dhp.shape[0], 4, 4)) # Z-axis matrices for each joint
    for i in range(dhp.shape[0]):
        # Create joint transformation from DH Parameters
        t0[i] = np.mat([(c(dhp[i,3]), -s(dhp[i,3]), 0, dhp[i,1]),
                        (s(dhp[i,3]) * c(dhp[i,0]), c(dhp[i,3]) * c(dhp[i,0]), -s(dhp[i,0]),-dhp[i,2] * s(dhp[i, 0])),
                        (s(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,0]), dhp[i,2] * c(dhp[i, 0])),
                        (0, 0, 0, 1)])

        # Put each transformation in the frame 0
        if i != 0: t0[i] = np.matmul(t0[i-1],t0[i])

        # Compute Z axis in frame 0 for each joint
        z0[i] = np.matmul(t0[i],np.matrix([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 15],
                           [0, 0, 0, 1]]))

    (fig, ax) = visualizeArm(t0, z0)
    plt.show()


    # Jacobian Matrix derived from 0-4T
    jac = np.mat([[-s(a1)*(55*s(a2)+30*s(a2+a4)+4.5*c(a2)-4.5*c(a2+a4)), c(a1)*(55*c(a2)+30*c(a2+a4)+4.5*s(a2+a4)-4.5*s(a2)), c(a1)*(30*c(a2+a4)+4.5*s(a2+a4))],
                  [c(a1)*(55*s(a2)+30*s(a2+a4)+4.5*c(a2)-4.5*c(a2+a4)), s(a1)*(55*c(a2)+30*c(a2+a4)+4.5*s(a2+a4)-4.5*s(a2)), s(a1)*(30*c(a2+a4)+4.5*s(a2+a4))],
                  [0, -55*s(a2)-30*s(a2+a4)+4.5*c(a2+a4)-4.5*c(a2), -30*s(a2+a4)+4.5*c(a2+a4)]])

    # Torque of arm (inverse dynamics)
    torque = np.linalg.inv(jac)*force_in
    # Recalculate force (forward dynamics)
    force = jac*torque

    # Print the joint coordinates (in the base frame) to the Terminal
    print("Joint\t X\t  Y\t\tZ\t(all cm)")
    np.savetxt(sys.stdout.buffer, np.hstack((np.matrix((1,2,3,4)).T, t0[:,:-1,3])), fmt="%.2f")

    # Print the Jacobian Matrix
    print("\nJacobian:")
    np.savetxt(sys.stdout.buffer, jac, fmt="% .2f")

    # Print the determinant of the Jacobian Matrix
    print("\nDeterminant: %.2f" % (np.linalg.det(jac)) )

    # Print the Joint torque for each angle calculated from inverse kinematics
    print("\nJoint Torques:")
    for i in range(torque.shape[0]): print (" Tau"+str(i**2+1)+":  % .2f N-cm" %(torque[i,0]))

    # Print the force at the end effector
    print("\nEnd Effector Force:")
    d = ("Fx", "Fy", "Fz")
    for i in range(force.shape[0]): print(" " + d[i] + ":  % .2f N" % (force[i, 0]))

    # Draw the arm in its current configuration
    (fig, ax) = visualizeArm(t0, z0)
    plt.show()


if __name__ == "__main__":
    main(sys.argv)