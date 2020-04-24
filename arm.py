# arm.py
# Fall 2019
# Ben Webb

from dynamixelSDK.src.dynamixel_sdk import *  # Uses Dynamixel SDK library
import time, data, datetime
import numpy as np

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def inverse_jacobian(a1,a2,a3,a4,x,y,z,Ya):
    return np.linalg.inv(jacobian(a1,a2,a3,a4)) \
           * np.matrix([x, y, z, Ya]).T

def jacobian(a1,a2,a3,a4):
    return np.matrix([[-22.3*np.sin(a1)-25.3*np.sin(a1+a2)-6.5*np.sin(a1+a2)*np.cos(a3)+17*np.sin(a1+a2)*np.sin(a3)-6*np.sin(a1+a2)*np.cos(a3)*np.cos(a4)-6*np.cos(a1+a2)*np.sin(a4),
                   -25.3*np.sin(a1+a2)-6.5*np.sin(a1+a2)*np.cos(a3)+17*np.sin(a1+a2)*np.sin(a3)-6*np.sin(a1+a2)*np.cos(a3)*np.cos(a4)-np.cos(a1+a2)*np.sin(a4),
                   -6.5*np.cos(a1+a2)*np.sin(a3)-17*np.cos(a1+a2)*np.cos(a3)-6*np.cos(a1+a2)*np.sin(a3)*np.cos(a4),
                   -6*np.cos(a1+a2)*np.cos(a3)*np.sin(a4)-6*np.sin(a1+a2)*np.cos(a4)],
                  [22.3*np.cos(a1)+25.3*np.cos(a1+a2)+6.5*np.cos(a1+a2)*np.cos(a3)-17*np.cos(a1+a2)*np.sin(a3)+6*np.cos(a1+a2)*np.cos(a3)*np.cos(a4)-6*np.sin(a1+a2)*np.sin(a4),
                   25.3*np.cos(a1+a2)+6.5*np.cos(a1+a2)*np.cos(a3)-17*np.cos(a1+a2)*np.sin(a3)+6*np.cos(a1+a2)*np.cos(a3)*np.cos(a4)-6*np.sin(a1+a2)*np.sin(a4),
                   -6.5*np.cos(a1+a2)*np.sin(a3)-17*np.cos(a1+a2)*np.cos(a3)-6*np.cos(a1+a2)*np.cos(a3)*np.cos(a4),
                   -6*np.cos(a1+a2)*np.cos(a3)*np.sin(a4)+6*np.cos(a1+a2)*np.cos(a4)],
                  [0, 0, -6.5*np.cos(a3)+17*np.sin(a3)-6*np.cos(a3)*np.cos(a4), 6*np.sin(a3)*np.sin(a4)],
                  [0, 0, np.cos(a1+a2), np.sin(a1+a2)*np.sin(a3)]])

port_handler = PortHandler('/dev/cu.usbmodem14101')
# port_handler = PortHandler('/dev/cu.usbserial-A5052R9T')

packet_handler = PacketHandler(1.0)
if port_handler.openPort():
    print("Port Open")
else:
    exit(-1)
if port_handler.setBaudRate(1000000):
    print("Baudrate Set")
else:
    exit(-1)

class Motor:
    def __init__(self, motor_id, min_pos, max_pos, max_speed, compliance_margin, compliance_slope, theta_shift=np.pi/2):
        self.id = motor_id
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.max_speed = max_speed
        self.compliance_margin = compliance_margin
        self.compliance_slope = compliance_slope
        ax_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, self.id, 36)
        self.goal_pos = ax_present_position
        self.cur_pos = ax_present_position
        self.theta_shift = theta_shift

    def enable_torque(self, verbose):
        ax_comm_result, ax_error = packet_handler.write1ByteTxRx(port_handler, self.id, 24, 1)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
            else: print("[ID:%01d] has been successfully connected" % self.id)

    def disable_torque(self):
        packet_handler.write1ByteTxRx(port_handler, self.id, 24, 0)

    def set_speed(self, verbose):
        ax_comm_result, ax_error = packet_handler.write2ByteTxRx(port_handler, self.id, 32, self.max_speed)
        if verbose:
            if ax_comm_result != COMM_SUCCESS:
                print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0:
                print("%s" % packet_handler.getRxPacketError(ax_error))
            else:
                print("[ID:%01d] Speed Set" % self.id)

    def set_angular_compliance(self, verbose):
        ax_comm_result, ax_error = packet_handler.write1ByteTxRx(port_handler, self.id, 26, self.compliance_margin)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
            else: print("[ID:%01d] CW Compliance Set" % self.id)

        ax_comm_result, ax_error = packet_handler.write1ByteTxRx(port_handler, self.id, 27, self.compliance_margin)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
            else: print("[ID:%01d] CCW Compliance Set" % self.id)

        ax_comm_result, ax_error = packet_handler.write1ByteTxRx(port_handler, self.id, 28, self.compliance_slope)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
            else: print("[ID:%01d] CW Compliance Set" % self.id)

        ax_comm_result, ax_error = packet_handler.write1ByteTxRx(port_handler, self.id, 29, self.compliance_slope)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
            else: print("[ID:%01d] CCW Compliance Set" % self.id)

    def get_id(self):
        return self.id

    def get_goal_pos(self):
        return self.goal_pos

    def get_compliance_margin(self):
        return self.compliance_margin

    def get_position(self):
        ax_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, self.id, 36)
        return ax_present_position
        # self.cur_pos = self.cur_pos * 2 / 3 + ax_present_position / 3
        # return self.cur_pos

    def get_theta(self):
        return (self.get_position() - 205) * np.pi / 614 - (self.theta_shift)

    def set_theta(self, theta, wait, verbose):
        self.set_position(int(205+(theta + self.theta_shift) * 614 / np.pi), wait, verbose)

    def get_speed(self):
        ax_present_speed, _, _ = packet_handler.read2ByteTxRx(port_handler, self.id, 38)
        return ax_present_speed

    def get_load(self):
        ax_present_load, _, _ = packet_handler.read2ByteTxRx(port_handler, self.id, 40)
        return ax_present_load

    def is_moving(self):
        ax_moving, _, _ = packet_handler.read1ByteTxRx(port_handler, self.id, 46)
        if ax_moving == 1:
            return True


    def set_position(self, position, wait, verbose):
        if position > self.max_pos: position = self.max_pos
        elif position < self.min_pos: position = self.min_pos
        self.goal_pos = position
        ax_comm_result, ax_error = packet_handler.write2ByteTxRx(port_handler, self.id, 30, self.goal_pos)
        if verbose:
            if ax_comm_result != COMM_SUCCESS: print("%s" % packet_handler.getTxRxResult(ax_comm_result))
            elif ax_error != 0: print("%s" % packet_handler.getRxPacketError(ax_error))
        if wait:
            while abs(position - self.get_position()) > self.compliance_margin: None


class Arm:
    def __init__(self, motor_list=(), verbose=True):
        self.verbose = verbose
        if motor_list == ():
            self.motor_list = [Motor(1, 255, 767, 80, 12, 128),
                               Motor(2, 100, 973, 120, 10, 64, 0),
                               Motor(3, 105, 800, 180, 10, 32),
                               Motor(4, 50, 793, 210, 10, 32)]
        else: self.motor_list = motor_list

    def bootup(self):
        self.enable_torque()
        self.set_speed()
        self.set_angular_compliance()

    def enable_torque(self):
        [m.enable_torque(self.verbose) for m in self.motor_list]

    def disable_torque(self):
        [m.disable_torque() for m in self.motor_list]

    def close_connection(self):
        self.disable_torque()
        port_handler.closePort()
        exit()

    def set_speed(self):
        [m.set_speed(self.verbose) for m in self.motor_list]

    def set_angular_compliance(self):
        [m.set_angular_compliance(self.verbose) for m in self.motor_list]

    def set_position(self, motor, position=512, wait=False):
        motor.set_position(position, wait, self.verbose)

    def set_positions(self, position=(512, 512, 512, 512), wait=False):
        for i in range(len(self.motor_list)):
            self.set_position(self.motor_list[-1 - i], position[-1 - i], wait)
        for i in range(len(self.motor_list)):
            m = self.motor_list[-1 - i]
            while abs(m.get_goal_pos() - m.get_position()) > m.get_compliance_margin():
                print(m.id, m.get_goal_pos(), m.get_position())

    def set_theta(self, motor, position = 0, wait = False):
        motor.set_theta(position, wait, self.verbose)

    def set_thetas(self, position=(0, 0, 0, 0), wait = False):
        for i in range(len(self.motor_list)):
            self.set_theta(self.motor_list[-1 - i], position[-1 - i], wait)
        # for i in range(len(self.motor_list)):
        #     m = self.motor_list[-1 - i]
        #     while abs(m.get_goal_pos() - m.get_position()) > m.get_compliance_margin()/2:
        #         print(m.id, m.get_goal_pos(), m.get_position())
        #         None

    def get_position(self, id):
        return self.motor_list[id-1].get_position()

    def get_positions(self):
        return [m.get_position() for m in self.motor_list]

    def get_theta(self, id):
        return self.motor_list[id-1].get_theta()

    def get_thetas(self):
        return [m.get_theta() for m in self.motor_list]

    def get_transformation(self):
        t1, t2, t3, t4 = self.get_thetas()
        return np.matrix([[-np.sin(t4)*np.sin(t1+t2)+np.cos(t3)*np.cos(t4)*np.cos(t1+t2), -np.sin(t4)*np.cos(t3)*np.cos(t1+t2)-np.sin(t1+t2)*np.cos(t4), np.sin(t3)*np.cos(t1+t2), -18.5*np.sin(t3)*np.cos(t1+t2)-6*np.sin(t4)*np.sin(t1+t2)+22.3*np.cos(t1)+6*np.cos(t3)*np.cos(t4)*np.cos(t1+t2)+6.5*np.cos(t3)*np.cos(t1+t2)+25.3*np.cos(t1+t2)],
                          [np.sin(t4)*np.cos(t1+t2)+np.sin(t1+t2)*np.cos(t3)*np.cos(t4), -np.sin(t4)*np.sin(t1+t2)*np.cos(t3)+np.cos(t4)*np.cos(t1+t2), np.sin(t3)*np.sin(t1+t2), 22.3*np.sin(t1)-18.5*np.sin(t3)*np.sin(t1+t2)+6*np.sin(t4)*np.cos(t1+t2)+6*np.sin(t1+t2)*np.cos(t3)*np.cos(t4)+6.5*np.sin(t1+t2)*np.cos(t3)+25.3*np.sin(t1+t2)],
                          [-np.sin(t3)*np.cos(t4), np.sin(t3)*np.sin(t4), np.cos(t3), -6*np.sin(t3)*np.cos(t4)-6.5*np.sin(t3)-18.5*np.cos(t3)+19],
                          [0, 0, 0, 1]])

    def get_jacobian(self):
        return jacobian(*self.get_thetas())

    def get_inv_jacobian(self, x, y, z, Ya):
        return inverse_jacobian(*self.get_thetas(), x, y, z, Ya)

    def get_ea(self):
        return self.get_transformation()[:-1, -1].T.tolist()[0]

    def get_speed(self, id):
        return self.motor_list[id-1].get_speed()

    def get_speeds(self):
        return [m.get_speed() for m in self.motor_list]

    def get_load(self, id):
        return self.motor_list[id-1].get_load()

    def get_loads(self):
        return [m.get_load() for m in self.motor_list]

    def move(self, dx, dy, dz, Ya, steps, sleeptime = 0.03):
        for m in self.motor_list:
            m.angular_compliance = 1
            m.set_angular_compliance(self.verbose)
        a1, a2, a3, a4 = self.get_thetas()
        pos = self.get_ea()
        x, y, z = pos[0] + dx, pos[1] + dy, pos[2] + dz
        while abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1:
            print (pos)
            pos = self.get_ea()
            dx, dy, dz = x - pos[0], y - pos[1], z - pos[2]
            p = self.get_inv_jacobian(dx/steps, dy/steps, dz/steps, Ya/steps)
            a1, a2, a3, a4 = (np.matrix((a1, a2, a3, a4)).T + p).T.tolist()[0]
            self.set_thetas((a1,a2,a3,a4))
            time.sleep(sleeptime)

    def move_x(self, distance, steps=300, sleep=0.03):
        a1, a2, a3, a4 = self.get_thetas()
        x, y, z = self.get_ea()
        step_a2, step_x = ((22.3 * a2) - (distance * 0.5)) / (22.3 * steps), distance / steps
        for i in range(steps):
            a2 += step_a2
            x += step_x
            a1 = 2.13668 - a2 + (0.02779 * x)
            self.set_thetas((a1, a2, a3, a4))
            time.sleep(sleep)

    def trial(self, type='Line'):
        self.disable_torque()
        self.motor_list[2].enable_torque()
        self.motor_list[3].enable_torque()
        toc = data.Data('TableOfContents.csv')
        while 1:
            c = getch()
            if c == chr(0x1b): break
            elif c == chr(0x20):
                newtrial = data.Data('template.csv')
                filename = '%s%.3i.csv' % (type, toc.get_num_points())
                print (toc.count_matching('filename', type))
                print('Now tracking ' + filename)
                i = 0
                p0 = self.get_ea()
                # for i in range(500):
                speed = self.get_speeds()
                while speed[0] != 0 or speed[1] != 0 or speed[2] != 0 or speed[3] != 0 or i < 200:
                    newtrial.addRow((i, *self.get_thetas(), *self.get_ea(), *speed, *self.get_loads()))
                    print (i)
                    i += 1
                    speed = self.get_speeds()
                print('Closing ' + filename)
                p = self.get_ea()
                dist = np.sqrt((p0[0] - p[0])**2 + (p0[1] - p[1])**2)
                toc.addRow((filename, '/DataFolder/Data/LineY'+filename, datetime.datetime.now(), type, dist, np.arctan2(p0[1]-p[1], p0[0]-p[0]), *p0, *p))
                newtrial.write(filename, True, 'LineY')
                toc.write('TableOfContents.csv', False)
                print('Done tracking ' + filename)

def main():
    arm = Arm(verbose=True)
    try:
        
        arm.bootup()
        # arm.set_angular_compliance()
        # print(arm.get_ea())
        # arm.move_x(-1, sleep=.1)
        # arm.set_positions([560, 300, 200, 632])
        # arm.track('testTriangle.csv')
        # print(arm.get_transformation())
        # print(arm.get_jacobian())
        # arm.move(0, 0, 1, 0, 12)
        # time.sleep(1)
        # print(arm.get_ea())
        # print(arm.get_positions())
        # print(arm.get_thetas())
        # print ('hi')
        # arm.move(-5, -10, 0, 0, 16)
        arm.trial()
        # arm.set_positions()
        # arm.move(6, 3, 0, 0, 24)
        # time.sleep(1)
        # arm.move(-6,-3, 0, 0, 24)
        # arm.track('corner.csv')
        # arm.linear()
        # arm.set_positions()
        # arm.set_positions((300,500,200,200))
        # time.sleep(.3)
        # arm.set_positions()
        arm.close_connection()
        # arm.set_positions((512, 512, 512, 512))
        # arm.set_positions(wait=True)
        # print (arm.get_ea())
    except KeyboardInterrupt:
        arm.close_connection()


if __name__ == "__main__":
    main()

'''

[19.06116913267098, 41.24101485525948, 1.0946642699830313]
-1cm y
[18.973489142882265, 40.111575539755435, 1.0266920104840551]
+1cm y
[19.07485550239428, 41.12773860859118, 1.0946642699830313]
+1cm x
[19.819376950905944, 41.292583404101556, 1.0945883834606462]
-1cm x
[19.07485550239428, 41.12773860859118, 1.0946642699830313]
+2cm Y
[18.937279524206478, 43.16856674053385, 1.0946642699830313]

'''