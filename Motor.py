# Motor.py
# Spring 2020
# Ben Webb

from numpy import pi

class Motor:
    def __init__(self, motor_id, port_handler, packet_handler, min_pos, max_pos, max_speed, compliance_margin, cw_slope,
                 ccw_slope, theta_shift=pi / 2):
        self.id = motor_id
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.max_speed = max_speed
        self.compliance_margin = compliance_margin
        self.cw_compliance_slope = cw_slope
        self.ccw_compliance_slope = ccw_slope
        ax_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, self.id, 36)
        self.goal_pos = ax_present_position
        self.cur_pos = ax_present_position
        self.theta_shift = theta_shift

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 0)

    def set_speed(self):
        self.packet_handler.write2ByteTxOnly(self.port_handler, self.id, 32, self.max_speed)

    def set_angular_compliance(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 26, self.compliance_margin)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 27, self.compliance_margin)

    def set_angular_range(self):
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, 6, self.min_pos)
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, 8, self.max_pos)

    def set_angular_slope(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 28, self.cw_compliance_slope)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 29, self.ccw_compliance_slope)

    def set_cw_slope(self, slope):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 0)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 28, slope)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 1)
        self.cw_compliance_slope = slope

    def set_ccw_slope(self, slope):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 0)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 29, slope)
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, 24, 1)
        self.ccw_compliance_slope = slope

    def get_id(self):
        return self.id

    def get_goal_pos(self):
        goal_pos, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, 30)
        return goal_pos

    def get_compliance_margin(self):
        return self.compliance_margin

    def get_position(self):
        ax_present_position, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, 36)
        return ax_present_position

    def get_theta(self):
        return (self.get_position() - 205) * 0.00511660041 - self.theta_shift

    def set_theta(self, theta, wait):
        self.set_position(int(205 + (theta + self.theta_shift) * 195.442270117), wait)

    def get_speed(self):
        ax_present_speed, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, 38)
        return ax_present_speed

    def get_load(self):
        ax_present_load, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, 40)
        return ax_present_load

    def is_moving(self):
        ax_moving, _, _ = self.packet_handler.read1ByteTxRx(self.port_handler, self.id, 46)
        if ax_moving == 1:
            return True

    def set_position(self, position, wait):
        if position > self.max_pos:
            position = self.max_pos
        elif position < self.min_pos:
            position = self.min_pos
        self.goal_pos = int(position)
        self.packet_handler.write2ByteTxOnly(self.port_handler, self.id, 30, self.goal_pos)
        if wait:
            while abs(self.goal_pos - self.get_position()) > self.compliance_margin:
                pass

    def set_maximum_torque(self, torque=1023):
        self.set_speed()
        self.packet_handler.write2ByteTxRx(self.port_handler, 3, 14, torque)
        self.enable_torque()
