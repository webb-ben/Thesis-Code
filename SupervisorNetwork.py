# Supervisor.py
# Spring 2020
# Ben Webb

from Supervisor_I import *
import socket
import struct

class SupervisorServerTCP(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)

    def open_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # assign IP address and port number to socket and open
        server_socket.bind(('', 12345))
        server_socket.listen(10)

        try:
            while True:
                # Create a new TCP connection
                connection, client_address = server_socket.accept()
                try:
                    while True:
                        # receive the as many packets as there are and echo
                        message = connection.recv(1024)
                        if message:
                            m = np.array(struct.unpack('<2f', message))
                            self.move(m, 0.01)
                            connection.sendall(message)
                            continue
                        break
                finally:
                    connection.close()
        except KeyboardInterrupt:
            server_socket.close()

class SupervisorServerUDP(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)

    def open_server(self):
        # create a UDP socket
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_positions()
        self.pid.update_origin(self.arm.get_xy())

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # assign IP address and port number to socket
        server_socket.bind(('', 12345))

        while True:
            # receive the client packet from port 8000, the packet includes a
            # message and the address of the ping client
            message, address = server_socket.recvfrom(8000)
            m = np.array(struct.unpack('<2f', message))
            self.move(m, 0.01)
            server_socket.sendto(message, address)

class SupervisorServerRAW(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)

    def open_server(self):
        # create a UDP socket
        self.p1, self.p2, self.p3, self.p4 = self.arm.get_positions()
        self.pid.update_origin(self.arm.get_xy())
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_RAW)
        # assign IP address and port number to socket
        server_socket.bind(('', 12345))
        while True:
            # receive the client packet from port 8000, the packet includes a
            # message and the address of the ping client
            message, address = server_socket.recvfrom(8000)
            m = np.array(struct.unpack('<2f', message))
            self.move(m, 0.01)
            server_socket.sendto(message, address)

if __name__ == "__main__":
    sup = SupervisorServerRAW()
    # try:
    sup.open_server()
    # finally:
    print (sup.pid.pid_abs_error/3200)
    sup.arm.close_connection()
