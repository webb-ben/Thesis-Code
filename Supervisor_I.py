# Supervisor.py
# Spring 2020
# Ben Webb

from Arm import *
from dynamixelSDK.src.dynamixel_sdk import PortHandler, PacketHandler
np.set_printoptions(precision=5, suppress=True)

def step(t1, t2, x, y, step_x, step_y):
    """
     Follows the linear equations
     X Axis:  θ1 + θ2 = -0.0273428 * X  + 2.144
              θ2 = -0.0006346 * X**2 + -0.0446889 * Y + 3.5440421
     Y Axis:  θ1 = 0.0468319 * Y + -1.8564326
              θ2 = -0.0006110 * Y**2 + -0.0299428 * X + 3.1923881
    :param t1:
    :param t2:
    :param x:
    :param y:
    :param step_x:
    :param step_y:
    :return:
    """
    motor_primitive = np.mat(
        [[-0.0273428 + 0.0012492 * x, 0.046831],
         [-0.0012492 * x, -0.001222 * y]]
    ) * np.mat([step_x, step_y]).T
    return (np.mat((t1, t2)).T + motor_primitive).flatten().tolist()[:][0]


# class Movement:
#     def __init__(self, direction, distance):
#         self.dir = direction
#         self.dist = distance


class PID:
    def __init__(self, arm):
        self.arm = arm
        self.x, self.y, _ = self.arm.get_ea()

        self.pid_x = np.mat((0.0, 0.0, 0.0))
        self.pid_x_weights = np.mat((0.05, 0.001, 0.001)).T

        self.pid_y = np.mat((0.0, 0.0, 0.0))
        self.pid_y_weights = np.mat((0.0, 0.0, 0.0)).T


    def update_origin(self, x, y):
        self.x, self.y, = x, y

    def update_pid(self, dx=0.0, dy=0.0):
        self.x += dx
        self.y += dy
        x, y, _ = self.arm.get_ea()
        residual_x, residual_y = self.x - x, self.y - y

        self.pid_x[0, 2] = residual_x - self.pid_x[0, 0]
        self.pid_x[0, 0] = residual_x
        self.pid_x[0, 1] += residual_x
        solution_x = (self.pid_x * self.pid_x_weights)[0, 0]

        self.pid_y[0, 2] = residual_y - self.pid_y[0, 0]
        self.pid_y[0, 0] = residual_y
        self.pid_y[0, 1] += residual_y
        solution_y = (self.pid_y * self.pid_y_weights)[0, 0]

        print(*self.pid_x.tolist()[0][:], solution_x, x, self.x, y)
        return solution_x, solution_y


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
            port_handler.setBaudRate(1000000)
        except OSError:
            _ = None

        self.arm = Arm(port_handler, packet_handler)

        self.apply_pressure = False
        self.pid = PID(self.arm)

    def move(self, dx=0.0, dy=0.0, dz=0.0, steps=200, step_time=0.0025):
        """

        :param dx:
        :param dy:
        :param dz:
        :param steps:
        :param step_time:
        :return:
        """
        self.arm.set_positions(self.arm.get_positions())
        a1, a2, a3, a4 = self.arm.get_thetas()
        x, y, z = self.arm.get_ea()
        self.pid.update_origin(x, y)
        goal_x, goal_y = x + (dx*steps), y + (dy*steps)
        i = 0
        # while abs((x - goal_x)*dx) > 0.005 or abs((y - goal_y)*dy) > 0.005:
        while abs((y - goal_y) * dy) > 0.01:
            pid_x, pid_y = self.pid.update_pid(dx, dy)
            if np.pi / 2 > a1 > -np.pi / 2 and 3 * np.pi / 4 > a1 + a2 > np.pi / 4:
                x, y, z = self.arm.get_ea()
                a1, a2 = step(a1, a2, x, y, dx + pid_x, dy + pid_y)
                if self.apply_pressure:
                    a3 = 0.06163240578217778
                # print(i, a1, a2, *self.arm.get_thetas(), x, y, goal_x, goal_y, *self.pid.pid_x, *self.pid.pid_y)
                self.arm.set_thetas((a1, a2, a3, a4))
                time.sleep(step_time)

            # if dz != 0.0:
            #     self.arm.move(dx=0, dy=0, dz=dz, Ya=0, steps=2)

            i += 1
        self.arm.set_positions(self.arm.get_positions())

    def movement_planner(self, movements, steps=1000):
        """

        :param movements:
        :param steps:
        :return:
        """
        for m in movements:
            if m.dir == 'dx':
                self.move(dx=m.dist / steps)
            if m.dir == 'dy':
                self.move(dy=m.dist / steps)

    def pressure(self):

        self.apply_pressure = True
        # self.arm.move(0, 0, -0.1, 0, steps=5)
        a1, a2, a3, a4 = self.arm.get_thetas()
        self.arm.set_thetas((a1, a2, 0.06663240578217778, a4), wait=True)




if __name__ == "__main__":
    t = time.time()
    sup = Supervisor()
    sup.pressure()
    # square = (Movement('dx', 10), Movement('dy', 10), Movement('dx', -10), Movement('dy', -10))
    # sup.movement_planner(square)
    # sup.trial_name = "SquareII.csv"
    # sup.move(dx=-0.1, dy=-0.1, steps=100)
    # print(sup.arm.get_position(2))
    sup.move(dy=-0.05, steps=200)
    # sup.move(dx=-0.1, steps=100)
    # sup.move(dy=0.05, steps=200)
    # sup.move(dx=0.1, steps=100)
    print("Time: ", time.time() - t)
    sup.arm.close_connection()
