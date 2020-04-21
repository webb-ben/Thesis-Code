# ServerUDP.py
# Ben Webb
# 12/6/2019

import socket
import random

def main():
    """ create a simple ping server """
    
    # create a UDP socket
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # assign IP address and port number to socket
    serverSocket.bind(('', 12345))
    
    while True:
        # receive the client packet from port 8000, the packet includes a 
        # message and the address of the ping client  
        message, address = serverSocket.recvfrom(8000)
        serverSocket.sendto(message, address)
        
if __name__ == "__main__":
    main()
