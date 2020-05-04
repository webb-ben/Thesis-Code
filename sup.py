# Supervisor.py
# Spring 2020
# Ben Webb

from Arm import *
from ArmSimulator import ArmSimulator
from dynamixelSDK.src.dynamixel_sdk import PortHandler, PacketHandler
import time

np.set_printoptions(precision=5, suppress=True)


def step(p1, p2, x, y, s):
    """
     Follows the linear equations
     X Axis:  P1 = -5.5913346 * x + 1139.582 - P2
              P2 = -0.1292621 * x **2 + -8.7684171 * y + 890.1188361
     Y Axis:  P1 = 9.6372704 * Y + 172.9242101
              P2 = -0.1342670 * Y**2 + -3.9898714 * X + 774.8796034
    :param t1:
    :param t2:
    :param x:
    :param y:
    :param s:
    :return:
    """
    return np.array((p1, p2)) + np.array(
        ((-5.5913346 + 0.2585242 * x, 9.6372704),
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
        residuals = self.pos - pos
        self.pos += d


        self.pid[:, 2] = residuals - self.pid[:, 0]
        self.pid[:, 0] = residuals
        self.pid[:, 1] += residuals

        self.pid_abs_error += abs(residuals)

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
        self.pid = PID(self.arm.get_xy())
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
        pid = self.pid.update_pid(np.array((x, y)), s)
        self.p1, self.p2 = step(self.p1, self.p2, x, y, s)
        self.arm.set_positions((self.p1, self.p2, self.p3, self.p4))
        print (x, y, self.arm.get_position(1), self.arm.get_position(2))
        while time.perf_counter() < _:
            pass

    def movement_planner(self, d, steps=400, step_time=0.005):
        i = 0
        while i < steps:
            self.move(d * (np.cos(2 * np.pi * i / steps + np.pi) + 1), step_time)
            i += 1
        return i

    def pressure(self):
        self.apply_pressure = True
        a1, a2, a3, a4 = self.arm.get_positions()
        self.arm.set_positions((a1, a2, 512, a4))
        time.sleep(0.6)

    def train_pid(self):
        prev_error_x, prev_error_y = 10000.0, 10000.0
        # px, ix, dx = 0.189063, 0.000038, 0.0731131
        # py, iy, dy = 0.185938, 0.000099, 0.0239653
        px, ix, dx = 0.074765625, 0.0, 0.220625
        py, iy, dy = 0.0671875, 0.0, 0.218

        vx, vy = 0.05, 0.05
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_position(1), self.arm.get_position(2),  self.arm.get_position(3), 512
        self.pid.update_origin(self.arm.get_xy())

        for j in range(1):
            self.pid.update_pid_weight(px, ix, dx, py, iy, dy)
            r = 0
            for f in range(1):
                r += self.movement_planner(np.array([0.00, 0.025]))
                # r += self.movement_planner(np.array([0.025, 0.00]))
                r += self.movement_planner(np.array([0.00, -0.025]))
                # r += self.movement_planner(np.array([-0.025, 0.00]))
                # r += self.movement_planner(np.array([ 0.1,  0.00]))
                # r += self.movement_planner(np.array([ 0.00,  0.1]))
                # r += self.movement_planner(np.array([-0.1,  0.00]))
                # r += self.movement_planner(np.array([ 0.00, -0.1]))

            print(
                  "%.8f %.8f %.6f %.6f %.6f %.6f %.6f %.6f" %
                  (*(self.pid.pid_abs_error / r).tolist(), px, ix, dx, py, iy, dy))

            if self.pid.pid_abs_error[0]/r < prev_error_x:
                px += vx
                vx /= 2
            else:
                px -= vx
                vx *= -1

            if self.pid.pid_abs_error[1]/r < prev_error_y:
                py += vy
                vy /= 2
            else:
                py -= vy
                vy *= -1

            prev_error_x, prev_error_y = self.pid.pid_abs_error / r



if __name__ == "__main__":
    sup = Supervisor()
    # sup.pressure()
    t = time.time()
    print (sup.arm.get_thetas())
    # try:
        # print(sup.arm.get_positions())
    sup.train_pid()
        # sup.arm.trial()
        # print("Time: ", time.time() - t)
    # finally:
    sup.arm.close_connection()

