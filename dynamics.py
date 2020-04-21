'''
dynamics.py
Senior Honors Thesis Fall 2019
Ben Webb
Caitrin Eaton

Derives Transformation and Jacobian Matrices
'''

from sympy import cos as c
from sympy import sin as s
from sympy import symbols, Matrix, pi, simplify, eye
import numpy as np
import time

def main():
    a1, a2, a3, a4 = symbols('a1 a2 a3 a4')
    d1, t1, t2, t3, t4 = symbols('d t1 t2 t3 t4')
    # a1, a2, a3, a4 = 0, 0, 0, 0
    # t1, t2, t3, t4 = 0, 0, 0, 0
    # Case 1 : DH-Parameters
    dhp = Matrix([[0, 0, 18.5, t1],
                  [0, 22.3, 0, t2-np.pi/2],
                  [-pi / 2, 25.3, 0, t3],
                  [pi / 2, 6.5, -1.5, 0],
                  [0, 5, -8.5, 0]])

    # Case 2: DH-Parameters
    # dhp = Matrix([[0, 0, 18.5, t1],
    #               [0, 22.3, 0, t2],
    #               [0, 25.3, 0, t3],
    #               [-pi/2, 9, 0, t4],
    #               [pi/2, 6, -15.5, 0]])
    t0 = eye(4)
    for i in range(dhp.shape[0]):
        t0 = simplify(t0 *
                      Matrix([[c(dhp[i, 3]), -s(dhp[i,3]), 0, dhp[i,1]],
                        [s(dhp[i,3]) * c(dhp[i,0]), c(dhp[i,3]) * c(dhp[i,0]), -s(dhp[i,0]),-dhp[i,2] * s(dhp[i, 0])],
                        [s(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,3]) * s(dhp[i,0]), c(dhp[i,0]), dhp[i,2] * c(dhp[i, 0])],
                        [0, 0, 0, 1]])
                      )
    print(simplify(t0))
    exit()

    # Simplify some derivations
    # print (simplify(
        # -c(a4)*s(a1+a2+a3)*c(a4)-s(a4)*s(a1+a2+a3)*s(a4)
        # -s(a1+a2+a3)*c(a4)*-s(a4)-s(a1+a2+a3)*s(a4)*c(a4)
        # -c(a1+a2+a3)*s(a4)*-s(a4)+c(a1+a2+a3)*c(a4)*c(a4)
        # c(a1+a2+a3)*c(a4)*(c(a1+a2+a3)*c(a4))-s(a1+a2+a3)*-s(a1+a2+a3)+c(a1+a2+a3)*s(a4)*c(a1+a2+a3)*s(a4)
        # -s(a1+a2+a3)*s(a4)*c(a1+a2+a3)*c(a4) + s(a1+a2+a3)*c(a4) * c(a1+a2+a3)*s(a4)
        # -c(a3)*c(a4)*(s(a1+a2)*c(a3)*c(a4)+c(a1+a2)*s(a4))+c(a3)*s(a4)*(-s(a1+a2)*c(a3)*s(a4)+c(a1+a2)*c(a4))-s(a3)*s(a1+a2)*s(a3)
        # -s(a1+a2)*s(a3)*c(a4)*(c(a1+a2)*c(a3)*c(a4)-s(a1+a2)*s(a4))+s(a1+a2)*s(a3)*s(a4)*(-c(a1+a2)*c(a3)*s(a4)-s(a1+a2)*c(a4))+s(a1+a2)*c(a3)*c(a1+a2)*s(a3)
        # c(a3)*(s(a1+a2)*(s(a3)**2)*s(a4)-c(a1+a2)*c(a4)*s(a4)-s(a1+a2)*c(a3))
        # c(a1+a2)*s(a3)*c(a4)*(s(a3)*c(a4))+c(a1+a2)*s(a3)*s(a4)*(s(a3)*s(a4))+c(a1+a2)*c(a3)*c(a3)
    # ))

    # Case 1: Jacobian
    jac = Matrix([[-22.3*s(a1)-25.3*s(a1+a2)-6.5*s(a1+a2)*c(a3)+17*s(a1+a2)*s(a3)-6*s(a1+a2)*c(a3)*c(a4)-6*c(a1+a2)*s(a4),
                   -25.3*s(a1+a2)-6.5*s(a1+a2)*c(a3)+17*s(a1+a2)*s(a3)-6*s(a1+a2)*c(a3)*c(a4)-c(a1+a2)*s(a4),
                   -6.5*c(a1+a2)*s(a3)-17*c(a1+a2)*c(a3)-6*c(a1+a2)*s(a3)*c(a4),
                   -6*c(a1+a2)*c(a3)*s(a4)-6*s(a1+a2)*c(a4)],
                  [22.3*c(a1)+25.3*c(a1+a2)+6.5*c(a1+a2)*c(a3)-17*c(a1+a2)*s(a3)+6*c(a1+a2)*c(a3)*c(a4)-6*s(a1+a2)*s(a4),
                   25.3*c(a1+a2)+6.5*c(a1+a2)*c(a3)-17*c(a1+a2)*s(a3)+6*c(a1+a2)*c(a3)*c(a4)-6*s(a1+a2)*s(a4),
                   -6.5*c(a1+a2)*s(a3)-17*c(a1+a2)*c(a3)-6*c(a1+a2)*c(a3)*c(a4),
                   -6*c(a1+a2)*c(a3)*s(a4)+6*c(a1+a2)*c(a4)],
                  [0,0, -6.5*c(a3)+17*s(a3)-6*c(a3)*c(a4), 6*s(a3)*s(a4)],
                  [0,0,c(a1+a2),s(a1+a2)*s(a3)]])

    # Case 2: Jacobian
    # jac = Matrix([[
    #     -22.3*s(a1)-25.3*s(a1+a2)-9*s(a1+a2+a3)-6*s(a1+a2+a3)*c(a4)+15.5*s(a1+a2+a3)*s(a4),
    #     -25.3*s(a1+a2)-9*s(a1+a2+a3)-6*s(a1+a2+a3)*c(a4)+15.5*s(a1+a2+a3)*s(a4),
    #     -9*s(a1+a2+a3)-6*s(a1+a2+a3)*c(a4)+15.5*s(a1+a2+a3)*s(a4),
    #     -6*c(a1+a2+a3)*s(a4)-15.5*c(a1+a2+a3)*c(a4)],
    #     [22.3*c(a1)+25.3*c(a1+a2)+9*c(a1+a2+a3)+6*c(a1+a2+a3)*c(a4)+15.5*c(a1+a2+a3)*s(a4),
    #      25.3*c(a1+a2)+9*c(a1+a2+a3)+6*c(a1+a2+a3)*c(a4)+15.5*c(a1+a2+a3)*s(a4),
    #      9*c(a1+a2+a3)+6*c(a1+a2+a3)*c(a4)+15.5*c(a1+a2+a3)*s(a4),
    #      -c(a1+a2+a3)*s(a4)+15.5*s(a1+a2+a3)*c(a4)],
    #     [0, 0, 0, 15.5*s(a4)-6*c(a4)]])

    # Case 3: Jacobian
    jac = Matrix([[-22.3*s(a1)-37.8*c(a1+a2),37.8*c(a1+a2)],
        [22.3*c(a1)+37.8*s(a1+a2), 37.8*s(a1+a2)]])
    print ("Inverse:", simplify(jac.inv()))
    print(simplify((0.58994708994709*s(a1)/(37.8*s(2*a1 + 2*a2) + 22.3*c(a2)))))
    t = time.time()
    # Case 1: Inverse Jacobian
    Ta = np.matrix([60, 0, 0, 0]).T
    exit()
    # i_jac = Matrix([[(-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (
    #                               -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(
    #                           a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) - (
    #                               17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(
    #                           a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (
    #                               17 * sin(a3) * cos(a1 + a2) + 6 * sin(a4) * sin(a1 + a2) - 22.3 * cos(a1) - 6 * cos(
    #                           a3) * cos(a4) * cos(a1 + a2) - 6.5 * cos(a3) * cos(a1 + a2) - 25.3 * cos(a1 + a2)) + (
    #                               -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(
    #                           a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (
    #                               -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(
    #                           a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) / ((-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #     a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))), -(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) / (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))), (((-(-(-6 * sin(a4) * cos(a3) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a4)) * (
    #             -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(
    #         a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) + (
    #                                                   -6 * sin(a4) * cos(a3) * cos(a1 + a2) + 6 * cos(a4) * cos(
    #                                               a1 + a2)) * (
    #                                                   -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                               a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                               a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) * (
    #                                                 17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(
    #                                             a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #                                             a3) - 25.3 * sin(a1 + a2)) + (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * (-6 * sin(a4) * cos(a3) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a4))) * (
    #                                                17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) - 6 * (-(
    #             (-6.5 * sin(a3) * cos(a1 + a2) - 6 * cos(a3) * cos(a4) * cos(a1 + a2) - 17 * cos(a3) * cos(a1 + a2)) * (
    #                 -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(
    #             a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) - (
    #                         -6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(a3) * cos(
    #                     a1 + a2)) * (
    #                         -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(
    #                     a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2))) * (17 * sin(
    #     a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2)) + (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (
    #                                               -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                           a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                           a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) * (-6 * sin(a3) * cos(
    #     a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(a3) * cos(a1 + a2))) * sin(a3) * sin(a4)) * cos(
    #     a1 + a2) + (((-6.5 * sin(a3) * cos(a1 + a2) - 6 * cos(a3) * cos(a4) * cos(a1 + a2) - 17 * cos(a3) * cos(
    #     a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(
    #     a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) - (
    #                              -6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(
    #                          a3) * cos(a1 + a2)) * (
    #                              -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(
    #                          a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2))) * (
    #                             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(
    #                         a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) - (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * (-6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(
    #     a3) * cos(a1 + a2))) * ((17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2))) / ((-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * ((17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2)) * (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * (
    #                                                                      -22.3 * sin(a1) + 17 * sin(a3) * sin(
    #                                                                  a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(
    #                                                                  a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                                                  a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))), -((-(
    #             -(-6 * sin(a4) * cos(a3) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a4)) * (
    #                 -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(
    #             a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) + (
    #                         -6 * sin(a4) * cos(a3) * cos(a1 + a2) + 6 * cos(a4) * cos(a1 + a2)) * (
    #                         -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(
    #                     a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) * (17 * sin(
    #     a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2)) + (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (
    #                                               -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                           a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                           a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) * (-6 * sin(a4) * cos(
    #     a3) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a4))) * (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(
    #     a3)) - 6 * (-(
    #             (-6.5 * sin(a3) * cos(a1 + a2) - 6 * cos(a3) * cos(a4) * cos(a1 + a2) - 17 * cos(a3) * cos(a1 + a2)) * (
    #                 -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(
    #             a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) - (
    #                         -6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(a3) * cos(
    #                     a1 + a2)) * (
    #                         -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(
    #                     a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2))) * (
    #                             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(
    #                         a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) + (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * (-6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(a1 + a2) - 17 * cos(
    #     a3) * cos(a1 + a2))) * sin(a3) * sin(a4)) / ((-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * ((17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(
    #     a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(
    #     a1 + a2)))], [(17 * sin(a3) * cos(a1 + a2) + 6 * sin(a4) * sin(a1 + a2) - 22.3 * cos(a1) - 6 * cos(a3) * cos(
    #     a4) * cos(a1 + a2) - 6.5 * cos(a3) * cos(a1 + a2) - 25.3 * cos(a1 + a2)) / (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))), (
    #                               -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(
    #                           a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) / (-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))), (((-(-6 * sin(a4) * cos(a3) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a4)) * (
    #             -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(
    #         a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) + (
    #                                                 -6 * sin(a4) * cos(a3) * cos(a1 + a2) + 6 * cos(a4) * cos(
    #                                             a1 + a2)) * (
    #                                                 -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                             a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                             a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2))) * (
    #                                                17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) - 6 * ((
    #                                                                                                                         -6.5 * sin(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2) - 6 * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a4) * cos(
    #                                                                                                                     a1 + a2) - 17 * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2)) * (
    #                                                                                                                         -22.3 * sin(
    #                                                                                                                     a1) + 17 * sin(
    #                                                                                                                     a3) * sin(
    #                                                                                                                     a1 + a2) - 6 * sin(
    #                                                                                                                     a4) * cos(
    #                                                                                                                     a1 + a2) - 6 * sin(
    #                                                                                                                     a1 + a2) * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a4) - 6.5 * sin(
    #                                                                                                                     a1 + a2) * cos(
    #                                                                                                                     a3) - 25.3 * sin(
    #                                                                                                                     a1 + a2)) - (
    #                                                                                                                         -6 * sin(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a4) * cos(
    #                                                                                                                     a1 + a2) - 6.5 * sin(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2) - 17 * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2)) * (
    #                                                                                                                         -17 * sin(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2) - 6 * sin(
    #                                                                                                                     a4) * sin(
    #                                                                                                                     a1 + a2) + 22.3 * cos(
    #                                                                                                                     a1) + 6 * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a4) * cos(
    #                                                                                                                     a1 + a2) + 6.5 * cos(
    #                                                                                                                     a3) * cos(
    #                                                                                                                     a1 + a2) + 25.3 * cos(
    #                                                                                                                     a1 + a2))) * sin(
    #     a3) * sin(a4)) * cos(a1 + a2) + (-(
    #             -6.5 * sin(a3) * cos(a1 + a2) - 6 * cos(a3) * cos(a4) * cos(a1 + a2) - 17 * cos(a3) * cos(a1 + a2)) * (
    #                                                  -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                              a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #                                              a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) + (
    #                                                  -6 * sin(a3) * cos(a4) * cos(a1 + a2) - 6.5 * sin(a3) * cos(
    #                                              a1 + a2) - 17 * cos(a3) * cos(a1 + a2)) * (
    #                                                  -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #                                              a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(
    #                                              a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2))) * (
    #                                               (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(
    #                                           a3) * sin(a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2))) / ((-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(
    #     a1 + a2) + 22.3 * cos(a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(
    #     a1 + a2)) + (-17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(a4) * cos(
    #     a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (-22.3 * sin(a1) + 17 * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #     a3) - 25.3 * sin(a1 + a2))) * ((17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(
    #     a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2)) * (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3))), -((
    #                                                                                                                               -(
    #                                                                                                                                           -6 * sin(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a1 + a2) * cos(
    #                                                                                                                                       a4)) * (
    #                                                                                                                                           -17 * sin(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a4) * sin(
    #                                                                                                                                       a1 + a2) + 22.3 * cos(
    #                                                                                                                                       a1) + 6 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) + 6.5 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) + 25.3 * cos(
    #                                                                                                                                       a1 + a2)) + (
    #                                                                                                                                           -6 * sin(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) + 6 * cos(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2)) * (
    #                                                                                                                                           -22.3 * sin(
    #                                                                                                                                       a1) + 17 * sin(
    #                                                                                                                                       a3) * sin(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a1 + a2) * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) - 6.5 * sin(
    #                                                                                                                                       a1 + a2) * cos(
    #                                                                                                                                       a3) - 25.3 * sin(
    #                                                                                                                                       a1 + a2))) * (
    #                                                                                                                               17 * sin(
    #                                                                                                                           a3) - 6 * cos(
    #                                                                                                                           a3) * cos(
    #                                                                                                                           a4) - 6.5 * cos(
    #                                                                                                                           a3)) - 6 * (
    #                                                                                                                               (
    #                                                                                                                                           -6.5 * sin(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) - 6 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) - 17 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2)) * (
    #                                                                                                                                           -22.3 * sin(
    #                                                                                                                                       a1) + 17 * sin(
    #                                                                                                                                       a3) * sin(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a1 + a2) * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) - 6.5 * sin(
    #                                                                                                                                       a1 + a2) * cos(
    #                                                                                                                                       a3) - 25.3 * sin(
    #                                                                                                                                       a1 + a2)) - (
    #                                                                                                                                           -6 * sin(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) - 6.5 * sin(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) - 17 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2)) * (
    #                                                                                                                                           -17 * sin(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) - 6 * sin(
    #                                                                                                                                       a4) * sin(
    #                                                                                                                                       a1 + a2) + 22.3 * cos(
    #                                                                                                                                       a1) + 6 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a4) * cos(
    #                                                                                                                                       a1 + a2) + 6.5 * cos(
    #                                                                                                                                       a3) * cos(
    #                                                                                                                                       a1 + a2) + 25.3 * cos(
    #                                                                                                                                       a1 + a2))) * sin(
    #     a3) * sin(a4)) / ((-(
    #             17 * sin(a3) * sin(a1 + a2) - sin(a4) * cos(a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(
    #         a1 + a2) * cos(a3) - 25.3 * sin(a1 + a2)) * (
    #                                    -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 22.3 * cos(
    #                                a1) + 6 * cos(a3) * cos(a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(
    #                                a1 + a2) + 25.3 * cos(a1 + a2)) + (
    #                                    -17 * sin(a3) * cos(a1 + a2) - 6 * sin(a4) * sin(a1 + a2) + 6 * cos(a3) * cos(
    #                                a4) * cos(a1 + a2) + 6.5 * cos(a3) * cos(a1 + a2) + 25.3 * cos(a1 + a2)) * (
    #                                    -22.3 * sin(a1) + 17 * sin(a3) * sin(a1 + a2) - 6 * sin(a4) * cos(
    #                                a1 + a2) - 6 * sin(a1 + a2) * cos(a3) * cos(a4) - 6.5 * sin(a1 + a2) * cos(
    #                                a3) - 25.3 * sin(a1 + a2))) * (
    #                                   (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(
    #                               a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2)))], [0, 0, sin(a3) * sin(a1 + a2) / (
    #             (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(a1 + a2) - 6 * sin(a3) * sin(
    #         a4) * cos(a1 + a2)), -6 * sin(a3) * sin(a4) / ((17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(
    #     a3) * sin(a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2))], [0, 0, -cos(a1 + a2) / (
    #             (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(a1 + a2) - 6 * sin(a3) * sin(
    #         a4) * cos(a1 + a2)), (17 * sin(a3) - 6 * cos(a3) * cos(a4) - 6.5 * cos(a3)) / ((17 * sin(a3) - 6 * cos(
    #     a3) * cos(a4) - 6.5 * cos(a3)) * sin(a3) * sin(a1 + a2) - 6 * sin(a3) * sin(a4) * cos(a1 + a2))]])


    i_jac = np.matrix([[(
                    -17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) / ((-17 * np.sin(a3) * np.sin(a1 + a2) + np.sin(
                    a4) * np.cos(a1 + a2) + 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) + 6.5 * np.sin(a1 + a2) * np.cos(
                    a3) + 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) - (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (
                    22.3 * np.sin(a1) - 17 * np.sin(a3) * np.sin(a1 + a2) + 6 * np.sin(a4) * np.cos(a1 + a2) + 6 * np.sin(a1 + a2) *
                    np.cos(a3) * np.cos(a4) + 6.5 * np.sin(a1 + a2) * np.cos(a3) + 25.3 * np.sin(a1 + a2))),

                    (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(
                    a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) / ((-17 * np.sin(a3) * np.sin(
                    a1 + a2) + np.sin(a4) * np.cos(a1 + a2) + 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) + 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) + 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(
                    a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) - (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin( a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(
                    a1 + a2)) * (22.3 * np.sin(a1) - 17 * np.sin(a3) * np.sin(a1 + a2) + 6 * np.sin(a4) * np.cos(a1 + a2) + 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) + 6.5 * np.sin(a1 + a2) * np.cos(a3) + 25.3 * np.sin(a1 + a2))),

                    (((-(-(-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a4)) * (-17 * np.sin(
                    a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(
                    a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-6 * np.sin(a4) * np.cos(a3) * np.cos(
                    a1 + a2) + 6 * np.cos(a4) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(
                    a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * (17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) + (-(17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(
                    a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (
                    -17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * (-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a4))) * (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) - 6 * (-((-6.5 * np.sin(
                    a3) * np.cos(a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (
                    -22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(
                    a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) - (-6 * np.sin(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2))) * (17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2)) + (-(17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(
                    a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * (-6 * np.sin(a3) * np.cos(a4) * np.cos(a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(
                    a3) * np.cos(a1 + a2))) * np.sin(a3) * np.sin(a4)) * np.cos(a1 + a2) + (((-6.5 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(
                    a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) - (-6 * np.sin(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2))) * (17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2)) - (-(17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(
                    a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * (-6 * np.sin(a3) * np.cos(a4) * np.cos(a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(
                    a3) * np.cos(a1 + a2))) * ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2))) / ((-(17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (
                    -17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(
                    a3)) * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2)) * (17 * np.sin(a3) - 6 * np.cos(
                    a3) * np.cos(a4) - 6.5 * np.cos(a3)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))),

                    -((-(-(-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a4)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-6 * np.sin(a4) * np.cos(a3) * np.cos(
                    a1 + a2) + 6 * np.cos(a4) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(
                    a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(
                    a3) - 25.3 * np.sin(a1 + a2))) * (17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) + (-(
                    17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(
                    a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(
                    a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * (-6 * np.sin(
                    a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a4))) * (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.cos(a3)) - 6 * (-((-6.5 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(
                    a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(
                    a3) - 25.3 * np.sin(a1 + a2)) - (-6 * np.sin(a3) * np.cos(a4) * np.cos(a1 + a2) - 6.5 * np.sin(a3) * np.cos(
                    a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2))) * (17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) + (-(17 * np.sin(
                    a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(
                    a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * (-6 * np.sin(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2))) * np.sin(a3) * np.sin(a4)) / ((-(
                    17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)))],

                [(17*np.sin(a3)*np.cos(a1 + a2) + 6*np.sin(a4)*np.sin(a1 + a2) - 25.3*np.sin(a1 + a2) - 22.3*np.cos(a1) - 6*np.cos(a3)*np.cos(
                    a4)*np.cos(a1 + a2) - 6.5*np.cos(a3)*np.cos(a1 + a2))/(7.5*np.sin(2*a1 + 2*a2) + 72.475*np.sin(
                    a2 - a3) + 72.475*np.sin(a2 + a3) - 39.025*np.sin(a2 - a4) + 39.025*np.sin(a2 + a4) + 3.75*np.sin(
                    a3 - 2*a4) + 8.125*np.sin(a3 - a4) - 8.125*np.sin(a3 + a4) - 3.75*np.sin(a3 + 2*a4) + 282.095*np.sqrt(2)*np.sin(
                    2*a1 + a2 + pi/4) + 27.875*np.sin(2*a1 + a2 - a4) - 27.875*np.sin(2*a1 + a2 + a4) - 3.75*np.sin(
                    2*a1 + 2*a2 - 2*a4) - 3.75*np.sin(2*a1 + 2*a2 + 2*a4) - 33.45*np.sin(-a2 + a3 + a4) + 33.45*np.sin(
                    a2 - a3 + a4) + 33.45*np.sin(a2 + a3 - a4) + 33.45*np.sin(a2 + a3 + a4) + 1.875*np.sin(
                    2*a1 + 2*a2 - a3 - 2*a4) + 4.0625*np.sin(2*a1 + 2*a2 - a3 - a4) - 4.0625*np.sin(2*a1 + 2*a2 - a3 + a4) -
                    1.875*np.sin(2*a1 + 2*a2 - a3 + 2*a4) + 1.875*np.sin(2*a1 + 2*a2 + a3 - 2*a4) + 4.0625*np.sin(
                    2*a1 + 2*a2 + a3 - a4) - 4.0625*np.sin(2*a1 + 2*a2 + a3 + a4) - 1.875*np.sin(2*a1 + 2*a2 + a3 + 2*a4) -
                    282.095*np.sqrt(2)*np.cos(a2 + pi/4) - 189.55*np.cos(a2 - a3) + 189.55*np.cos(a2 + a3) + 21.25*np.cos(a3 - a4) -
                    21.25*np.cos(a3 + a4) - 31.625*np.cos(2*a1 + 2*a2 - a4) + 31.625*np.cos(2*a1 + 2*a2 + a4) - 10.625*np.cos(
                    2*a1 + 2*a2 - a3 - a4) + 10.625*np.cos(2*a1 + 2*a2 - a3 + a4) + 10.625*np.cos(2*a1 + 2*a2 + a3 - a4) -
                    10.625*np.cos(2*a1 + 2*a2 + a3 + a4)),

                    (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) / (7.5 * np.sin(
                    2 * a1 + 2 * a2) + 72.475 * np.sin(a2 - a3) + 72.475 * np.sin(a2 + a3) - 39.025 * np.sin(
                    a2 - a4) + 39.025 * np.sin(a2 + a4) + 3.75 * np.sin(a3 - 2 * a4) + 8.125 * np.sin(a3 - a4) - 8.125 * np.sin(
                    a3 + a4) - 3.75 * np.sin(a3 + 2 * a4) + 282.095 * np.sqrt(2) * np.sin(2 * a1 + a2 + pi / 4) + 27.875 * np.sin(
                    2 * a1 + a2 - a4) - 27.875 * np.sin(2 * a1 + a2 + a4) - 3.75 * np.sin(2 * a1 + 2 * a2 - 2 * a4) -
                    3.75 * np.sin(2 * a1 + 2 * a2 + 2 * a4) - 33.45 * np.sin(-a2 + a3 + a4) + 33.45 * np.sin(a2 - a3 + a4) +
                    33.45 * np.sin(a2 + a3 - a4) + 33.45 * np.sin(a2 + a3 + a4) + 1.875 * np.sin(2 * a1 + 2 * a2 - a3 - 2 * a4) +
                    4.0625 * np.sin(2 * a1 + 2 * a2 - a3 - a4) - 4.0625 * np.sin(2 * a1 + 2 * a2 - a3 + a4) - 1.875 * np.sin(
                    2 * a1 + 2 * a2 - a3 + 2 * a4) + 1.875 * np.sin(2 * a1 + 2 * a2 + a3 - 2 * a4) + 4.0625 * np.sin(
                    2 * a1 + 2 * a2 + a3 - a4) - 4.0625 * np.sin(2 * a1 + 2 * a2 + a3 + a4) - 1.875 * np.sin(
                    2 * a1 + 2 * a2 + a3 + 2 * a4) - 282.095 * np.sqrt(2) * np.cos(a2 + pi / 4) - 189.55 * np.cos(a2 - a3) +
                    189.55 * np.cos(a2 + a3) + 21.25 * np.cos(a3 - a4) - 21.25 * np.cos(a3 + a4) - 31.625 * np.cos(
                    2 * a1 + 2 * a2 - a4) + 31.625 * np.cos(2 * a1 + 2 * a2 + a4) - 10.625 * np.cos(2 * a1 + 2 * a2 - a3 - a4)
                    + 10.625 * np.cos(2 * a1 + 2 * a2 - a3 + a4) + 10.625 * np.cos(2 * a1 + 2 * a2 + a3 - a4) - 10.625 * np.cos(
                    2 * a1 + 2 * a2 + a3 + a4)),

                    (((-(-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a4)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) + 6 * np.cos(
                    a4) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) - 6 * ((-6.5 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(
                    a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) - (-6 * np.sin(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2))) * np.sin(a3) * np.sin(a4)) * np.cos(a1 + a2) + (-(
                    -6.5 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(
                    a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) + (-6 * np.sin(
                    a3) * np.cos(a4) * np.cos(a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(
                    a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(
                    a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2))) * ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.cos(a3)) * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2))) / ((-(
                    17 * np.sin(a3) * np.sin(a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(
                    a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(
                    a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(
                    a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(
                    a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * ((17 * np.sin(
                    a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(
                    a1 + a2)) * (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3))),

                    -((-(-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a4)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (-6 * np.sin(a4) * np.cos(a3) * np.cos(a1 + a2) + 6 * np.cos(
                    a4) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(
                    a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(
                    a1 + a2))) * (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) - 6 * ((-6.5 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(
                    a1) + 17 * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.sin(a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2)) - (-6 * np.sin(a3) * np.cos(a4) * np.cos(
                    a1 + a2) - 6.5 * np.sin(a3) * np.cos(a1 + a2) - 17 * np.cos(a3) * np.cos(a1 + a2)) * (-17 * np.sin(a3) * np.cos(
                    a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2))) * np.sin(a3) * np.sin(a4)) / ((-(17 * np.sin(a3) * np.sin(
                    a1 + a2) - np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(a1 + a2) * np.cos(
                    a3) - 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) + (
                    -17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (-22.3 * np.sin(a1) + 17 * np.sin(a3) * np.sin(
                    a1 + a2) - 6 * np.sin(a4) * np.cos(a1 + a2) - 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) - 6.5 * np.sin(
                    a1 + a2) * np.cos(a3) - 25.3 * np.sin(a1 + a2))) * ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(
                    a3)) * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2)))],
                [0, 0, -np.sin(a1 + a2)/((-17*np.sin(a3) + 6*np.cos(a3)*np.cos(a4) + 6.5*np.cos(a3))*np.sin(a1 + a2) + 6*np.sin(a4)*np.cos(
                    a1 + a2)),
                    6 * np.sin(a4) / ((-17 * np.sin(a3) + 6 * np.cos(a3) * np.cos(a4) + 6.5 * np.cos(a3)) * np.sin(a1 + a2) + 6 * np.sin(
                    a4) * np.cos(a1 + a2))],
                [0, 0, -np.cos(a1 + a2)/((17*np.sin(a3) - 6*np.cos(a3)*np.cos(a4) - 6.5*np.cos(a3))*np.sin(a3)*np.sin(a1 + a2) - 6*np.sin(
                    a3)*np.sin(a4)*np.cos(a1 + a2)),
                    (17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(a4) - 6.5 * np.cos(a3)) / ((17 * np.sin(a3) - 6 * np.cos(a3) * np.cos(
                    a4) - 6.5 * np.cos(a3)) * np.sin(a3) * np.sin(a1 + a2) - 6 * np.sin(a3) * np.sin(a4) * np.cos(a1 + a2))]])
    print ((
                    -17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(a1 + a2) + 6 * np.cos(a3) * np.cos(
                    a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) / ((-17 * np.sin(a3) * np.sin(a1 + a2) + np.sin(
                    a4) * np.cos(a1 + a2) + 6 * np.sin(a1 + a2) * np.cos(a3) * np.cos(a4) + 6.5 * np.sin(a1 + a2) * np.cos(
                    a3) + 25.3 * np.sin(a1 + a2)) * (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(
                    a1 + a2) + 25.3 * np.sin(a1 + a2) + 22.3 * np.cos(a1) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(
                    a3) * np.cos(a1 + a2)) - (-17 * np.sin(a3) * np.cos(a1 + a2) - 6 * np.sin(a4) * np.sin(a1 + a2) + 25.3 * np.sin(
                    a1 + a2) + 6 * np.cos(a3) * np.cos(a4) * np.cos(a1 + a2) + 6.5 * np.cos(a3) * np.cos(a1 + a2)) * (
                    22.3 * np.sin(a1) - 17 * np.sin(a3) * np.sin(a1 + a2) + 6 * np.sin(a4) * np.cos(a1 + a2) + 6 * np.sin(a1 + a2) *
                    np.cos(a3) * np.cos(a4) + 6.5 * np.sin(a1 + a2) * np.cos(a3) + 25.3 * np.sin(a1 + a2))))
    print (np.dot(i_jac, Ta))
if __name__ == "__main__":
    main()
