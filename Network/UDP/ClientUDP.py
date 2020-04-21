# ClientUDP.py
# Ben Webb
# 12/6/2019

import socket
import statistics as st
import time


def main():
    # Define default settings
    SERVER_ADDR = '10.0.0.196'
    SERVER_PORT = 12345
    content = bytearray(1)

    # Create UDP socket and send first datagram to find server IP address
    clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clientSocket.settimeout(0.5)

    # Initialize variables for the loop
    RTT = []

    # Loop until exit criteria are met (max packets or KeyboardInterrupt)
    for i in range(20):
        # Send packet and record time when sent
        clientSocket.sendto(content, (SERVER_ADDR, SERVER_PORT))
        sendTime = time.perf_counter()

        # Receive datagram from server and print to terminal
        content, address = clientSocket.recvfrom(8000)
        RTT.append((time.perf_counter() - sendTime)*1000)
        # time.sleep(.25)
        # print (i)
        # time.sleep(0.01)

    # Print statistics summary at the end of the program.
    print(len(RTT))
    print('round-trip min/avg/max/stddev = %.3f/%.3f/%.3f/%.3f ms' % (min(RTT), st.mean(RTT), max(RTT), st.stdev(RTT)))


if __name__ == "__main__":
    main()
