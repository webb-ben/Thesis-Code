# Supervisor.py
# Spring 2020
# Ben Webb

from Arm import *
from ArmSimulator import ArmSimulator
from dynamixelSDK.src.dynamixel_sdk import PortHandler, PacketHandler
import time
import random

np.set_printoptions(precision=5, suppress=True)


def step(p1, p2, x, y, s):
    """
     Follows the linear equations
     X Axis:  P1 = -5.5913346 * X + 1139.582 - P2
              P2 = -0.1292621 * X **2 + -8.7684171 * Y + 890.1188361
     Y Axis:  P1 = 9.6372704 * Y + 172.9242101
              P2 = -0.1342670 * Y**2 + -3.9898714 * X + 774.8796034
    :param t1:
    :param t2:
    :param xy:
    :param s:
    :return:
    """
    return np.array((p1, p2)) + np.array(
        ((-5.5913346 + 0.2585242 * x, 9.372704),
         (-0.2585242 * x, -0.268534 * y))
    ) @ s


class PID:
    def __init__(self, pos):
        """
        Y-motor primitive
        px, ix, dx = 0.0132, 0.00014, 0.01
        py, iy, dy = 0.0064, 0.00025, 0.0047
        """
        px, ix, dx = 0.074765625, 0.0002, 0.220625
        py, iy, dy = 0.0671875, 0.00025, 0.218

        self.pid = np.array(((0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)))

        self.pid_weights = np.array(((px, py),
                                     (ix, iy),
                                     (dx, dy)))

        self.pid_abs_error = np.array((0.0, 0.0))
        self.pos = np.array(pos)

    def update_origin(self, pos):
        self.pos = np.array(pos)

    def update_pid_weight(self, px, ix, dx, py, iy, dy):
        self.pid = np.array(((0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)))
        self.pid_weights = np.array(((px, py),
                                     (ix, iy),
                                     (dx, dy)))
        self.pid_abs_error = np.array((0.0, 0.0))

    def update_pid(self, pos, d):
        self.pos += d
        residuals = self.pos - pos

        self.pid[:, 2] = residuals - self.pid[:, 2]
        self.pid[:, 0] = residuals
        self.pid[:, 1] += residuals

        self.pid_abs_error += abs(residuals)

        result = self.pid @ self.pid_weights

        return np.array((result[0, 0], result[1, 1]))

class PDA(PID):
    def __init__(self, pos):
        PID.__init__(self, pos)

    def update_pid(self, pos, d):
        self.pos += d
        residuals = self.pos - pos

        # print (self.pid, self.pid[:, 0], residuals, self.pid[:, 1])
        self.pid[:, 2] = (residuals - self.pid[:, 1]) - self.pid[:, 0]
        self.pid[:, 0] = residuals - self.pid[:, 1]
        self.pid[:, 1] = residuals

        self.pid_abs_error += abs(self.pid[:, 2])

        result = self.pid @ self.pid_weights

        return np.array((result[0, 0], result[1, 1]))

class Supervisor:
    def __init__(self):
        import sys
        import os
        if sys.platform == 'linux':
            port_handler = PortHandler('/dev/ttyACM0')
        else:
            port_handler = PortHandler('/dev/' + os.listdir('/dev')[-2])
        packet_handler = PacketHandler(1.0)

        try:
            port_handler.openPort()
            port_handler.setBaudRate(100000)
        except OSError:
            _ = None

        self.arm = Arm(port_handler, packet_handler)
        self.apply_pressure = False
        self.pid = PDA(self.arm.get_xy())
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_positions()

    def move(self, s, step_time):
        """
        :param d:
        :param steps:
        :param step_time:
        :return:
        """
        _ = time.perf_counter() + step_time
        x, y = self.arm.get_xy()
        pid = self.pid.update_pid(np.array((x,y)), s)
        self.p1, self.p2 = step(self.p1, self.p2, x, y, s)
        self.arm.set_positions((self.p1, self.p2, self.p3, self.p4))
        print (*self.pid.pos, x, y,
               self.p1, self.arm.get_position(1),
               self.p2, self.arm.get_position(2))
        while time.perf_counter() < _:
            pass

    def move_i(self, s, step_time):
        """
        :param d:
        :param steps:
        :param step_time:
        :return:
        """
        _ = time.perf_counter() + step_time
        x, y = self.arm.get_xy()
        self.pid.update_pid(np.array((x,y)), s)
        try:
            self.p1, self.p2, self.p3, self.p4 = inverse_jacobian(self.p1, self.p2, self.p3, self.p4, s[0], s[1],0, 0)
        except np.linalg.LinAlgError:
            pass
        self.arm.set_thetas((self.p1, self.p2, self.p3, self.p4))
        print (*self.pid.pos, x, y,
               self.p1, self.arm.get_theta(1),
               self.p2, self.arm.get_theta(2))
        # while time.perf_counter() < _:
        #     pass

    def movement_planner(self, d, steps=400, step_time=0.005):
        i = 0
        while i < steps:
            self.move(d * (np.cos(2 * np.pi * i / steps + np.pi) + 1), step_time)
            i += 1
        return i

    def pressure(self):
        self.apply_pressure = True
        a1, a2, a3, a4 = self.arm.get_positions()
        self.arm.set_positions((a1, a2, 490, a4))
        time.sleep(0.6)

    def train_pid(self):
        prev_error_x, prev_error_y = 10000.0, 10000.0
        px, ix, dx = 0.059360, 0.000038, 0.731131
        py, iy, dy = 0.044491, 0.000099, 0.239653

        vx, vy = 0.2, 0.05
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_positions()
        self.pid.update_origin(self.arm.get_xy())

        for j in range(1):
            self.pid.update_pid_weight(px, ix, dx, py, iy, dy)
            r = 0
            for f in range(1):
                r += self.movement_planner(np.array([0.00, 0.025]))
                r += self.movement_planner(np.array([0.025, 0.00]))
                r += self.movement_planner(np.array([0.00, -0.025]))
                r += self.movement_planner(np.array([-0.025, 0.00]))
                # r += self.movement_planner(np.array([ 0.025,  0.00]))
                # r += self.movement_planner(np.array([ 0.00,  0.025]))
                # r += self.movement_planner(np.array([-0.025,  0.00]))
                # r += self.movement_planner(np.array([ 0.00, -0.025]))
            print(
                "%.8f %.8f %.6f %.6f %.6f %.6f %.6f %.6f" %
                (*(self.pid.pid_abs_error / r).tolist(), px, ix, dx, py, iy, dy))

            if self.pid.pid_abs_error[0] / r < prev_error_x:
                px += vx
                vx /= 2
            else:
                px -= vx
                vx *= -1

            if self.pid.pid_abs_error[1] / r < prev_error_y:
                py += vy
                vy /= 2
            else:
                py -= vy
                vy *= -1

            prev_error_x, prev_error_y = self.pid.pid_abs_error / r

    def train_pda(self):
        prev_error_x, prev_error_y = 10000.0, 10000.0

        px, dx, ax = 0.0083159, 0.0092100, 0.019590
        py, dy, ay = 0.00115781,  0.0097181, 0.042877

        # px, dx, ax = 0.0,0.0,0.0
        # py, dy, ay = 0.0,0.0,0.0
        vx, vy = 0.02, 0.02
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_positions()
        self.pid.update_origin(self.arm.get_xy())
        print (self.arm.get_thetas(), self.pid.pos)
        for j in range(1):
            self.pid.update_pid_weight(dx, px, ax, dy, py, ay)

            r = 0
            for f in range(1):
                r += self.movement_planner(np.array([ 0.00,  0.025]))
                r += self.movement_planner(np.array([ 0.025,  0.00]))
                r += self.movement_planner(np.array([ 0.00, -0.025]))
                r += self.movement_planner(np.array([-0.025,  0.00]))
                r += self.movement_planner(np.array([ 0.025,  0.025]))
                r += self.movement_planner(np.array([-0.025,  0.00]))
                r += self.movement_planner(np.array([ 0.025, -0.025]))
                r += self.movement_planner(np.array([-0.025,  0.00]))
            print(
                  "%.8f %.8f %.6f %.6f %.6f %.6f %.6f %.6f" %
                  (*(self.pid.pid_abs_error / r).tolist(), dx, px, ax, dy, py, ay))

            if self.pid.pid_abs_error[0]/r < prev_error_x:
                ax += vx
                vx /= 2
            else:
                ax -= vx
                vx *= -1

            if self.pid.pid_abs_error[1]/r < prev_error_y:
                ay += vy
                vy /= 2
            else:
                ay -= vy
                vy *= -1

            prev_error_x, prev_error_y = self.pid.pid_abs_error / r

'''
0.12102758 0.06611705 0.022100 0.059360 0.000000 0.021800 0.044490 0.000000
0.11071103 0.06223150 0.022100 0.059360 0.050000 0.021800 0.044490 0.050000
0.11166849 0.06378180 0.022100 0.059360 0.075000 0.021800 0.044490 0.075000
0.11271641 0.06547987 0.022100 0.059360 0.062500 0.021800 0.044490 0.062500
0.11793190 0.06510396 0.022100 0.059360 0.075000 0.021800 0.044490 0.075000
0.11391005 0.06209788 0.022100 0.059360 0.062500 0.021800 0.044490 0.087500
0.11137903 0.06155672 0.022100 0.059360 0.050000 0.021800 0.044490 0.093750
0.11821373 0.06647013 0.022100 0.059360 0.043750 0.021800 0.044490 0.096875
0.11760760 0.06742683 0.022100 0.059360 0.046875 0.021800 0.044490 0.095313
0.11267667 0.06396901 0.022100 0.059360 0.050000 0.021800 0.044490 0.096875
'''
if __name__ == "__main__":
    sup = Supervisor()

    # sup.pressure()
    t = time.time()
    try:
        # sup.train_pda()
        sup.arm.trial()
        print (time.time()-t)
    finally:
        sup.arm.close_connection()

"""
(14.183619690700096, 29.93854626661372) [13.797831297501167, 31.52263138084135, 0.2596464380613739]
18.630549558051385, 29.38649328492162) [18.48891994045686, 32.9048012791081, 1.7457253917922628]
"""