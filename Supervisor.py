# Supervisor.py
# Spring 2020
# Ben Webb

from dynamixelSDK.src.dynamixel_sdk import PortHandler, PacketHandler
from Arm_Simulator import *
import time
import data


class Movement:
    def __init__(self, direction, distance):
        self.dir = direction
        self.dist = distance


class PID:
    def __init__(self, arm):
        self.arm = arm
        self.x, self.y, _ = self.arm.get_ea()
        self.pid_x = np.mat((0, 0, 0))
        self.pid_x_weights = np.mat((0.01, 0.03, 0.001)).T
        self.pid_y = np.mat((0, 0, 0))
        self.pid_y_weights = np.mat((0.0, 0.0, 0.0)).T

    def update_pid(self, dx=0.0, dy=0.0):
        t1, t2, t3, t4 = self.arm.get_thetas()
        x, y, _ = self.arm.get_ea()
        residual_x, residual_y = x + dx - self.x, y + dy - self.y
        self.update_pid_x(residual_x, t1, t2, t3, t4)
        self.update_pid_x(residual_y, t1, t2, t3, t4)

    def update_pid_x(self, residual_x, t1, t2, t3, t4):
        if residual_x == 0.0: return
        self.pid_x[0, 2] = residual_x - self.pid_x[0, 0]
        self.pid_x[0, 0] = residual_x
        self.pid_x[0, 1] += residual_x
        solution_y = self.pid_x * self.pid_x_weights
        t1, t2 = step_y(t1, t2, solution_y)
        self.arm.set_thetas((t1, t2, t3, t4))

    def update_pid_y(self, residual_y, t1, t2, t3, t4):
        if residual_y == 0.0: return
        self.pid_y[0, 2] = residual_y - self.pid_y[0, 0]
        self.pid_y[0, 0] = residual_y
        self.pid_y[0, 1] += residual_y
        solution_x = self.pid_y * self.pid_y_weights
        t1, t2 = step_y(t1, t2, solution_x)
        self.arm.set_thetas((t1, t2, t3, t4))

def step(t1, t2, x, y, step_x, step_y):
    """
     Follows the linear equations
     X Axis:  θ1 = -0.0273428 * x - θ2 + 2.144
              θ2 = -0.0006346 * x ** 2 + -0.0446889 * y + 3.5440421
     Y Axis:  θ1 = 0.0468319 * Y + -1.8564326
              θ2 = -1.0093699*θ1 + -0.2813674*θ1**2 + -0.0286046*x + 2.208498
    :param t1:
    :param t2:
    :param x:
    :param y:
    :param step_x:
    :param step_y:
    :return:
    """

    x_tan = step_x / (step_x + step_y)
    y_tan = step_y / (step_x + step_y)
    print (x_tan, y_tan, step_x)
    x += step_x
    y += step_y
    motor_primitive = np.mat(
        [[-0.0273428 * x - t2 + 2.144, 0.0468319 * y + -1.8564326],
        [-0.0006346 * x ** 2 + -0.0446889 * y + 3.5440421, -1.0093699 * t1 + -0.2813674 * t1 ** 2 + -0.0286046 * x + 2.208498]]
    )

    # tf_mat = np.mat(
    #     [[x_tan, y_tan],
    #     [step_x, step_y]]
    # )

    tf_mat = np.mat([x_tan, y_tan]).T
    return motor_primitive * tf_mat


def step_y(t1, t2, step):
    """
        Follows the linear equations: θ1 = 0.046834 * Y - 1.856 &
                                      θ2 = -1.009 * θ1 - 0.2810 * θ**2 - 0.02886 * x + 2.208
    :param t1: θ1
    :param t2: θ2
    :param step: distance in Y of movement

    :return:
    """
    dt1 = 0.046884 * step
    dt2 = -0.047323631268 * step - 0.0263816268 * step * t1
    return t1 + dt1, t2 + dt2


def step_x(t1, t2, x, step):
    """
    Follows the linear equations: (θ1 + θ2) = -0.027x + 2.144 &
                                   θ2 = -0.0006246 * x**2 + -0.044708*y + 3.544510
    :param t1: θ1
    :param t2: θ2
    :param step: distance in Y of movement

    :return:
    """
    dt1 = - 0.0253428 * step + 0.0012492 * x * step
    dt2 = - 0.0012492 * x * step
    return t1 + dt1, t2 + dt2


class Supervisor:
    def __init__(self, trial=None):
        import sys, os
        if sys.platform == 'linux':
            self.port_handler = PortHandler('/dev/ttyACM0')
        else:
            self.port_handler = PortHandler('/dev/' + os.listdir('/dev')[-2])

        self.trial_name = trial
        self.trial_record = data.Data('template.csv')

        self.packet_handler = PacketHandler(1.0)
        try:
            self.port_handler.openPort()
            self.port_handler.setBaudRate(1000000)
        except:
            None
        self.arm = Arm_Simulator(self.port_handler, self.packet_handler)
        self.pid = PID(self.arm)

    def move(self, dx=0.0, dy=0.0, dz=0.0, steps=1000, step_time=0):
        """

        :param dx:
        :param dy:
        :param dz:
        :param steps:
        :param step_time:
        :return:
        """
        for m in self.arm.motor_list:
            m.compliance_margin = 0
            m.set_angular_compliance()
            m.compliance_slope = 0
            m.set_angular_slope()
            m.set_speed()
        self.arm.set_positions(self.arm.get_positions())
        a1, a2, a3, a4 = self.arm.get_thetas()
        for i in range(steps):
            # if self.trial_name:
            #     self.trial_record.addRow((i, *self.arm.get_thetas(), *self.arm.get_ea(), *self.arm.get_speeds(), *self.arm.get_loads()))
            #
            # print (*self.arm.get_thetas(), *self.arm.get_ea(), *self.arm.get_speeds(), *self.arm.get_loads())

            if dx != 0.0 and 3 * np.pi / 4 > a1 + a2 > np.pi / 4:

                x, y, _ = self.arm.get_ea()
                a1, a2 = step_x(a1, a2, x, dx)
                # r = step(a1, a2, x, y, dx, 0)
                # a1, a2 = r[0,0], r[1,0]
                # print(a1, a2, x, y)
                self.arm.set_thetas((a1, a2, a3, a4))
                # self.pid.update_pid(dx=dx)
                # time.sleep(step_time)

            if dy != 0.0 and np.pi / 2 > a1 > -np.pi / 2:
                a1, a2 = step_y(a1, a2, dy)
                self.arm.set_thetas((a1, a2, a3, a4))
                # time.sleep(step_time)

            if dz != 0.0:
                self.arm.move(dx=0, dy=0, dz=dz, Ya=0, steps=2)

        # self.pid.update_pid(dx, dy)
        time.sleep(0.1)
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

    def apply_pressure(self):
        self.arm.set_speed()
        self.arm.set_theta(sup.arm.motor_list[2], 0, True)

if __name__ == "__main__":
    t = time.time()
    try:
        sup = Supervisor()
        # square = (Movement('dx', 10), Movement('dy', 10), Movement('dx', -10), Movement('dy', -10))
        # sup.movement_planner(square)
        # sup.trial_name = "SquareII.csv"
        # sup.apply_pressure()
        print (sup.arm.get_ea())
        sup.move(dx=-0.1, steps=100)
        print (sup.arm.get_ea())
        sup.move(dy=-0.1, steps=100)
        print (sup.arm.get_ea())
        sup.move(dx=0.1, steps=100)
        print(sup.arm.get_ea())
        sup.move(dy=0.1, steps=100)
        print (sup.arm.get_ea())
    except KeyboardInterrupt:
        sup.trial_record.write("Results/" + sup.trial_name)
    print("hi ", time.time() - t)
    sup.arm.close_connection()
