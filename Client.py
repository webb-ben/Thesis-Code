# Client.py
# Ben Webb
#

# Ben Webb
# 12/6/2019

import socket
import numpy as np
import struct
import time

steps = 400
movements = (np.array([0.00, 0.025]), np.array([0.025, 0.00]), np.array([0.00, -0.025]), np.array([-0.025, 0.00]))
SERVER_ADDR = '10.0.0.162'
SERVER_PORT = 12345

def TCP():

    # Create TCP socket and send first datagram to find server IP address
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_ADDR, SERVER_PORT))

    # Send all packets and record time when sent

    try:
        for move in movements:
            for i in range(steps):
                d = move * (np.cos(2 * np.pi * i / steps + np.pi) + 1)
                client_socket.sendall(struct.pack('<2f', *d))
                recv = 0
                while recv < len(d):
                    # Keep connection open for as long as the expected response
                    message = client_socket.recv(1024)
                    recv += len(message)
    finally:
        client_socket.close()

def UDP():
    # Define default settings
    # Create UDP socket and send first datagram to find server IP address
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Initialize variables for the loop
    # Loop until exit criteria are met (max packets or KeyboardInterrupt)
    for move in movements:
        for i in range(steps):
            recv_time = time.perf_counter()
            d = move * (np.cos(2 * np.pi * i / steps + np.pi) + 1)
            # Send packet and record time when sent
            client_socket.sendto(struct.pack('<2f', *d), (SERVER_ADDR, SERVER_PORT))
            while time.perf_counter() < _:
                pass
if __name__ == "__main__":
    UDP()
