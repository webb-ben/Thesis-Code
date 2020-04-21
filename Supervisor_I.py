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
     X Axis:  P1 = -5.3439414 * X + 1136.064 - P2
              P2 = -0.1240340 * X **2 + -8.7340980 * y + 897.655633
     Y Axis:  P1 = 9.1529305 * Y + 149.1745896
              P2 = -0.1194068 * Y**2 + -5.8520929 * X + 828.9275838
    :param t1:
    :param t2:
    :param x:
    :param y:
    :param s:
    :return:
    """
    return np.array((p1, p2)) + np.array(
        ((-5.3439414 + 0.248068 * x, 9.1529305),
         (-0.248068 * x, -0.2388136 * y))
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
            port_handler.setBaudRate(1000000)
        except OSError:
            _ = None

        self.arm = ArmSimulator(port_handler, packet_handler)

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
        self.p1, self.p2 = step(self.p1, self.p2, x, y, s + pid)
        self.arm.set_positions((self.p1, self.p2, self.p3, self.p4))
        print (self.pid.pos[0], x, self.pid.pos[1], y,
               self.p1, self.arm.get_position(1),
               self.p2, self.arm.get_position(2))
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
        px, ix, dx = 0.041235, 0.000038, 0.406131
        py, iy, dy = 0.034491, 0.000099, 0.365280

        vx, vy = 0.01, 0.01
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_position(1), self.arm.get_position(2), 512, 512
        self.pid.update_origin(self.arm.get_xy())

        for j in range(1):
            self.pid.update_pid_weight(px, ix, dx, py, iy, dy)
            r = 0
            for f in range(1):
                r += self.movement_planner(np.array([ 0.00,  0.025]))
                r += self.movement_planner(np.array([ 0.025,  0.00]))
                r += self.movement_planner(np.array([ 0.00, -0.025]))
                r += self.movement_planner(np.array([-0.025,  0.00]))
                # r += self.movement_planner(np.array([ 0.025,  0.00]))
                # r += self.movement_planner(np.array([ 0.00,  0.025]))
                # r += self.movement_planner(np.array([-0.025,  0.00]))
                # r += self.movement_planner(np.array([ 0.00, -0.025]))
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

'''
643.8830877189278 303.72287507548714 10000 10000 0.0075 0.005
696.453379797246 307.69105620247564 643.8830877189278 303.72287507548714 0.0125 0.01
719.1808566471108 324.40590987080543 696.453379797246 307.69105620247564 0.01 0.0075
705.868515423729 324.5148855639921 719.1808566471108 324.40590987080543 0.0125 0.01
728.8244944673896 337.6749557419218 705.868515423729 324.5148855639921 0.015000000000000001 0.0075

0.46996680 0.47318441 0.00500 0.00000 0.00220 0.00150 0.00000 0.00110
0.42030037 0.30397424 0.00500 0.00000 0.00270 0.00150 0.00000 0.00160
0.47279692 0.31583070 0.00500 0.00000 0.00295 0.00150 0.00000 0.00185
0.47127508 0.33234551 0.00500 0.00000 0.00283 0.00150 0.00000 0.00172
0.43922467 0.30536477 0.00500 0.00000 0.00270 0.00150 0.00000 0.00185
0.40998432 0.30491520 0.00500 0.00000 0.00264 0.00150 0.00000 0.00198
0.39439062 0.29559661 0.00500 0.00000 0.00261 0.00150 0.00000 0.00204
0.41306375 0.28868056 0.00500 0.00000 0.00259 0.00150 0.00000 0.00207
0.41786780 0.28694614 0.00500 0.00000 0.00260 0.00150 0.00000 0.00208
0.42054947 0.34659056 0.00500 0.00000 0.00259 0.00150 0.00000 0.00209

0.46088514 0.24785328 0.00500 0.00000 0.00260 0.00150 0.00000 0.00208
0.48746732 0.24273448 0.00510 0.00000 0.00260 0.00160 0.00000 0.00208
0.47956108 0.24669842 0.00505 0.00000 0.00260 0.00165 0.00000 0.00208
0.51094401 0.24468197 0.00500 0.00000 0.00260 0.00162 0.00000 0.00208
0.48934883 0.26720067 0.00503 0.00000 0.00260 0.00160 0.00000 0.00208
0.50339880 0.24947273 0.00505 0.00000 0.00260 0.00161 0.00000 0.00208
0.47852467 0.24437587 0.00504 0.00000 0.00260 0.00162 0.00000 0.00208
0.47598820 0.26329172 0.00502 0.00000 0.00260 0.00163 0.00000 0.00208
0.45237502 0.26916721 0.00502 0.00000 0.00260 0.00163 0.00000 0.00208
0.51974882 0.27134092 0.00502 0.00000 0.00260 0.00163 0.00000 0.00208

0.48695793 0.30458314 0.00502 0.00001 0.00260 0.00163 0.00001 0.00208
0.46719501 0.42058786 0.00502 0.00002 0.00260 0.00163 0.00002 0.00208
0.45079472 0.33820897 0.00502 0.00003 0.00260 0.00163 0.00002 0.00208
0.40557133 0.28543828 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.43498817 0.32718758 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.42085486 0.31168317 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.46989095 0.32822550 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.44819912 0.36604331 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.44145286 0.34223696 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208
0.38791984 0.34973114 0.00502 0.00003 0.00260 0.00163 0.00001 0.00208

0.41522840 0.21887038 0.00502 0.00003 0.002600 0.001630 0.00001 0.00208
0.37692482 0.27164277 0.00502 0.00003 0.003600 0.001630 0.00001 0.00308
0.37958812 0.22572004 0.00502 0.00003 0.004100 0.001630 0.00001 0.00258
0.37484665 0.26155028 0.00502 0.00003 0.003850 0.001630 0.00001 0.00208
0.39732353 0.21706915 0.00502 0.00003 0.003600 0.001630 0.00001 0.00233
0.38065857 0.20999026 0.00502 0.00003 0.003725 0.001630 0.00001 0.00258
0.37821544 0.20906885 0.00502 0.00003 0.003850 0.001630 0.00001 0.00270
0.36078353 0.23295346 0.00502 0.00003 0.003912 0.001630 0.00001 0.00277
0.38837039 0.22753862 0.00502 0.00003 0.003944 0.001630 0.00001 0.00274
0.39094840 0.26133627 0.00502 0.00003 0.003928 0.001630 0.00001 0.00270

0.37896089 0.25639254 0.00502 0.00003 0.003912 0.001630 0.00001 0.00270
0.32788777 0.22813436 0.00602 0.00003 0.003912 0.002630 0.00001 0.00270
0.38326536 0.18436255 0.00652 0.00003 0.003912 0.003130 0.00001 0.00270
0.33432872 0.18992730 0.00627 0.00003 0.003912 0.003380 0.00001 0.00270
0.38243670 0.16835535 0.00602 0.00003 0.003912 0.003255 0.00001 0.00270
0.38576211 0.18513771 0.00614 0.00003 0.003912 0.003130 0.00001 0.00270
0.37804508 0.18494384 0.00602 0.00003 0.003912 0.003193 0.00001 0.00270
0.40588419 0.17771794 0.00589 0.00003 0.003912 0.003255 0.00001 0.00270
0.40976448 0.20078930 0.00596 0.00003 0.003912 0.003286 0.00001 0.00270
0.38857670 0.16611136 0.00589 0.00003 0.003912 0.003271 0.00001 0.00270

0.36447927 0.21356784 0.00502 0.00003 0.003912 0.003271 0.00001 0.00270
0.34475730 0.17507821 0.00602 0.00003 0.003912 0.004271 0.00001 0.00270
0.33641832 0.17455551 0.00652 0.00003 0.003912 0.004771 0.00001 0.00270
0.31095328 0.15863153 0.00677 0.00003 0.003912 0.005021 0.00001 0.00270
0.29927448 0.15852475 0.00690 0.00003 0.003912 0.005146 0.00001 0.00270
0.29297853 0.15988414 0.00696 0.00003 0.003912 0.005209 0.00001 0.00270
0.29692056 0.14558157 0.00699 0.00003 0.003912 0.005177 0.00001 0.00270
0.32735334 0.15468232 0.00697 0.00003 0.003912 0.005146 0.00001 0.00270
0.29904374 0.16225937 0.00699 0.00003 0.003912 0.005162 0.00001 0.00270
0.29072260 0.15246693 0.00700 0.00003 0.003912 0.005146 0.00001 0.00270

0.37850653 0.21870782 0.007000 0.000030 0.005037 0.005177 0.000010 0.002700
0.36838465 0.20717353 0.007000 0.000030 0.006037 0.005177 0.000010 0.003700
0.35883281 0.20697465 0.007000 0.000030 0.006537 0.005177 0.000010 0.004200
0.35178946 0.20585953 0.007000 0.000030 0.006787 0.005177 0.000010 0.004450
0.33200660 0.20606813 0.007000 0.000030 0.006912 0.005177 0.000010 0.004575
0.36167599 0.21691295 0.007000 0.000030 0.006974 0.005177 0.000010 0.004513
0.34758585 0.21255437 0.007000 0.000030 0.006943 0.005177 0.000010 0.004575
0.34770948 0.22179852 0.007000 0.000030 0.006912 0.005177 0.000010 0.004638
0.36711656 0.21819916 0.007000 0.000030 0.006928 0.005177 0.000010 0.004606
0.36127709 0.21418067 0.007000 0.000030 0.006912 0.005177 0.000010 0.004575

'''
'''
0.53855769 0.37631260 0.007000 0.000030 0.006912 0.005177 0.000010 0.004575
0.44907691 0.26840708 0.008000 0.000030 0.006912 0.006177 0.000010 0.004575
0.43370866 0.25121536 0.008500 0.000030 0.006912 0.006677 0.000010 0.004575
0.43183432 0.24678701 0.008750 0.000030 0.006912 0.006927 0.000010 0.004575
0.41967948 0.24262582 0.008875 0.000030 0.006912 0.007052 0.000010 0.004575
0.42362944 0.24275795 0.008938 0.000030 0.006912 0.007115 0.000010 0.004575
0.42754923 0.24143678 0.008906 0.000030 0.006912 0.007083 0.000010 0.004575
0.41895817 0.24236946 0.008938 0.000030 0.006912 0.007052 0.000010 0.004575
0.42794455 0.24281418 0.008969 0.000030 0.006912 0.007068 0.000010 0.004575
0.42587031 0.24692323 0.008953 0.000030 0.006912 0.007052 0.000010 0.004575

0.96357298 0.94818617 0.008000 0.000030 0.006912 0.005177 0.000010 0.004575
0.84939382 0.90691359 0.009000 0.000030 0.006912 0.005177 0.000010 0.004575
0.82023712 0.91191914 0.009500 0.000030 0.006912 0.005177 0.000010 0.004575
0.78753177 0.89467251 0.009750 0.000030 0.006912 0.005177 0.000010 0.004575
0.78685912 0.90573995 0.009875 0.000030 0.006912 0.005177 0.000010 0.004575
0.77904165 0.89943686 0.009938 0.000030 0.006912 0.005177 0.000010 0.004575
0.77314670 0.89690161 0.009969 0.000030 0.006912 0.005177 0.000010 0.004575
0.77573168 0.89383128 0.009984 0.000030 0.006912 0.005177 0.000010 0.004575
0.78408882 0.89670823 0.009977 0.000030 0.006912 0.005177 0.000010 0.004575
0.76696993 0.88420790 0.009984 0.000030 0.006912 0.005177 0.000010 0.004575

0.30284154 0.31203187 0.009984 0.000030 0.006912 0.005177 0.000010 0.004575
0.34106752 0.35245111 0.009984 0.000030 0.007912 0.005177 0.000010 0.005575
0.35124979 0.36155799 0.009984 0.000030 0.007412 0.005177 0.000010 0.005075
0.35582268 0.35964333 0.009984 0.000030 0.007912 0.005177 0.000010 0.005575
0.34662566 0.35754797 0.009984 0.000030 0.007412 0.005177 0.000010 0.006075
0.35367452 0.35600678 0.009984 0.000030 0.006912 0.005177 0.000010 0.006325
0.34140786 0.35463850 0.009984 0.000030 0.007162 0.005177 0.000010 0.006450
0.34321811 0.35268435 0.009984 0.000030 0.007412 0.005177 0.000010 0.006513
0.35070567 0.35774482 0.009984 0.000030 0.007287 0.005177 0.000010 0.006544
0.34987783 0.35799578 0.009984 0.000030 0.007412 0.005177 0.000010 0.006528

0.39980246 0.40111552 0.009984 0.000030 0.006912 0.005177 0.000010 0.006528
0.40153613 0.37926661 0.009984 0.000030 0.011912 0.005177 0.000010 0.011528
0.40343802 0.38055204 0.009984 0.000030 0.009412 0.005177 0.000010 0.014028
0.41630406 0.39584699 0.009984 0.000030 0.011912 0.005177 0.000010 0.012778
0.40471324 0.37919601 0.009984 0.000030 0.009412 0.005177 0.000010 0.014028
0.40142609 0.37802220 0.009984 0.000030 0.006912 0.005177 0.000010 0.015278
0.40214665 0.37792834 0.009984 0.000030 0.005662 0.005177 0.000010 0.015903
0.39974484 0.37561028 0.009984 0.000030 0.006287 0.005177 0.000010 0.016216
0.40215359 0.37451463 0.009984 0.000030 0.006912 0.005177 0.000010 0.016372
0.40524188 0.37512570 0.009984 0.000030 0.006599 0.005177 0.000010 0.016450

0.43054713 0.20273626 0.009984 0.000030 0.006912 0.005177 0.000010 0.016372
0.35724812 0.16176999 0.014984 0.000030 0.006912 0.010177 0.000010 0.016372
0.31720801 0.14540117 0.017484 0.000030 0.006912 0.012677 0.000010 0.016372
0.30055848 0.13545223 0.018734 0.000030 0.006912 0.013927 0.000010 0.016372
0.28846830 0.12941226 0.019359 0.000030 0.006912 0.014552 0.000010 0.016372
0.28649876 0.12608070 0.019672 0.000030 0.006912 0.014865 0.000010 0.016372
0.28345665 0.12611933 0.019828 0.000030 0.006912 0.015021 0.000010 0.016372
0.28765279 0.12870670 0.019906 0.000030 0.006912 0.014943 0.000010 0.016372
0.29407166 0.12699247 0.019867 0.000030 0.006912 0.015021 0.000010 0.016372
0.29300476 0.12687130 0.019906 0.000030 0.006912 0.015099 0.000010 0.016372

0.27498674 0.20859172 0.019828 0.000030 0.006912 0.015021 0.000010 0.016372
0.26150730 0.20780422 0.019828 0.000030 0.011912 0.015021 0.000010 0.021372
0.25629803 0.20808051 0.019828 0.000030 0.014412 0.015021 0.000010 0.023872
0.26040888 0.21349776 0.019828 0.000030 0.015662 0.015021 0.000010 0.022622
0.26844458 0.21382232 0.019828 0.000030 0.015037 0.015021 0.000010 0.023872
0.26442847 0.21315280 0.019828 0.000030 0.015662 0.015021 0.000010 0.022622
0.26653580 0.21105345 0.019828 0.000030 0.016287 0.015021 0.000010 0.021372
0.26194352 0.20994077 0.019828 0.000030 0.015974 0.015021 0.000010 0.020747
0.26140206 0.20868979 0.019828 0.000030 0.015662 0.015021 0.000010 0.020434
0.26593246 0.20765605 0.019828 0.000030 0.015506 0.015021 0.000010 0.020278

0.23796582 0.15979965 0.019828 0.000030 0.015662 0.015021 0.000010 0.020278
0.23602593 0.17328733 0.019828 0.000030 0.020662 0.015021 0.000010 0.025278
0.23497267 0.17226059 0.019828 0.000030 0.023162 0.015021 0.000010 0.022778
0.22673724 0.15838383 0.019828 0.000030 0.024412 0.015021 0.000010 0.020278
0.22770874 0.17764531 0.019828 0.000030 0.025037 0.015021 0.000010 0.019028
0.23411900 0.15619699 0.019828 0.000030 0.024724 0.015021 0.000010 0.019653
0.22934861 0.15983289 0.019828 0.000030 0.025037 0.015021 0.000010 0.020278
0.22632484 0.16103384 0.019828 0.000030 0.025350 0.015021 0.000010 0.019966
0.23595068 0.16320163 0.019828 0.000030 0.025506 0.015021 0.000010 0.020278
0.23331436 0.15925163 0.019828 0.000030 0.025428 0.015021 0.000010 0.019966

0.23448298 0.16774723 0.019828 0.000030 0.024412 0.015021 0.000010 0.020278
0.20297481 0.13893299 0.024828 0.000030 0.024412 0.020021 0.000010 0.020278
0.19284565 0.12938343 0.027328 0.000030 0.024412 0.022521 0.000010 0.020278
0.19089867 0.12338092 0.028578 0.000030 0.024412 0.023771 0.000010 0.020278
0.18638661 0.12188582 0.029203 0.000030 0.024412 0.024396 0.000010 0.020278
0.17430441 0.11924862 0.029516 0.000030 0.024412 0.024709 0.000010 0.020278
0.17963756 0.11752902 0.029672 0.000030 0.024412 0.024865 0.000010 0.020278
0.18568278 0.12091064 0.029594 0.000030 0.024412 0.024943 0.000010 0.020278
0.18260276 0.11738731 0.029672 0.000030 0.024412 0.024904 0.000010 0.020278
0.18171711 0.11643437 0.029750 0.000030 0.024412 0.024865 0.000010 0.020278

0.19361756 0.09180210 0.038266 0.000030 0.024412 0.033615 0.000010 0.020278
0.19940095 0.08916673 0.038266 0.000030 0.034412 0.033615 0.000010 0.030278
0.19328058 0.09031193 0.038266 0.000030 0.029412 0.033615 0.000010 0.035278
0.20196290 0.09222864 0.038266 0.000030 0.024412 0.033615 0.000010 0.032778
0.20107612 0.09343935 0.038266 0.000030 0.026912 0.033615 0.000010 0.035278
0.19867067 0.08891734 0.038266 0.000030 0.029412 0.033615 0.000010 0.032778
0.19509898 0.09041798 0.038266 0.000030 0.030662 0.033615 0.000010 0.030278
0.19357494 0.08895859 0.038266 0.000030 0.031287 0.033615 0.000010 0.031528
0.19556652 0.09100552 0.038266 0.000030 0.031599 0.033615 0.000010 0.032778
0.19354434 0.08917969 0.038266 0.000030 0.031443 0.033615 0.000010 0.032153

0.74947238 0.08495868 0.003827 0.000030 0.024412 0.033615 0.000010 0.020278
0.75968154 0.08201977 0.003827 0.000030 0.073236 0.033615 0.000010 0.020278
0.18333047 0.10190558 0.038266 0.000030 0.250000 0.033615 0.000010 0.020278
0.18487708 0.09408020 0.038266 0.000030 0.250000 0.033615 0.000010 0.202780

0.28945292 0.09488200 0.018266 0.000100 0.250000 0.033615 0.000060 0.202780
0.28607829 0.09783569 0.018266 0.000075 0.250000 0.033615 0.000085 0.202780
0.26809350 0.09849936 0.018266 0.000050 0.250000 0.033615 0.000073 0.202780
0.26490386 0.09713885 0.018266 0.000038 0.250000 0.033615 0.000085 0.202780
0.26976424 0.09680423 0.018266 0.000031 0.250000 0.033615 0.000098 0.202780
0.26386096 0.10389164 0.018266 0.000034 0.250000 0.033615 0.000104 0.202780
0.27217846 0.09864405 0.018266 0.000038 0.250000 0.033615 0.000101 0.202780
0.27501240 0.10208444 0.018266 0.000036 0.250000 0.033615 0.000098 0.202780
0.27004262 0.09988597 0.018266 0.000038 0.250000 0.033615 0.000099 0.202780

0.19818343 0.09735217 0.028266 0.000038 0.361305 0.033615 0.000099 0.184030
0.19166989 0.09814587 0.033266 0.000038 0.361305 0.034115 0.000099 0.184030
0.18569260 0.09719748 0.035766 0.000038 0.361305 0.033865 0.000099 0.184030
0.18714260 0.09620421 0.037016 0.000038 0.361305 0.033615 0.000099 0.184030
0.18365827 0.09714651 0.036391 0.000038 0.361305 0.033490 0.000099 0.184030
0.18965416 0.09602816 0.035766 0.000038 0.361305 0.033552 0.000099 0.184030
0.19840012 0.09924208 0.036078 0.000038 0.361305 0.033615 0.000099 0.184030
0.19368820 0.09901763 0.035766 0.000038 0.361305 0.033584 0.000099 0.184030
0.19176469 0.09588010 0.035453 0.000038 0.361305 0.033553 0.000099 0.184030
0.18655631 0.09674770 0.035297 0.000038 0.361305 0.033537 0.000099 0.184030

0.15332771 0.08686533 0.035766 0.000038 0.356131 0.033553 0.000099 0.184030
'''

if __name__ == "__main__":
    sup = Supervisor()
    sup.pressure()
    t = time.time()
    try:
        # print (sup.arm.get_xy(), sup.arm.get_ea())
        sup.train_pid()
        print("Time: ", time.time() - t)
    finally:
        sup.arm.close_connection()

"""
(14.183619690700096, 29.93854626661372) [13.797831297501167, 31.52263138084135, 0.2596464380613739]
18.630549558051385, 29.38649328492162) [18.48891994045686, 32.9048012791081, 1.7457253917922628]
"""