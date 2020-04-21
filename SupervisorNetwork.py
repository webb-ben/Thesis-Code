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
        server_socket.listen(6)

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
                            self.move(m, 0.005)
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
        self.pressure()
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # assign IP address and port number to socket
        server_socket.bind(('', 12345))

        while True:
            # receive the client packet from port 8000, the packet includes a
            # message and the address of the ping client
            message, address = server_socket.recvfrom(8000)
            m = np.array(struct.unpack('<2f', message))
            self.move(m, 0.005)
            server_socket.sendto(message, address)


if __name__ == "__main__":
    sup = SupervisorServerUDP()
    sup.open_server()
    sup.arm.close_connection()
