# SimplePingClient.py
# Ben Webb
# 12/6/2019

import socket
import statistics as st
import time

def main():
    content = bytearray(1)

    # Create TCP socket and send first datagram to find server IP address
    clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientSocket.connect(('10.0.0.196', 1234))

    # Send all packets and record time when sent
    RTT = []
    try:
        for i in range(20):
            clientSocket.sendall(content)
            sendTime = time.time()
            received = 0
            while received < len(content):
                # Keep connection open for as long as the expected response
                message = clientSocket.recv(100)
                received += len(message)
                RTT.append((time.time() - sendTime)*1000)
    finally:
        clientSocket.close()
    print(RTT)
    print('Round Trip min/avg/max/stddev = %.3f/%.3f/%.3f/%.3f ms' % (min(RTT), st.mean(RTT), max(RTT), st.stdev(RTT)))


if __name__ == "__main__":
    main()
