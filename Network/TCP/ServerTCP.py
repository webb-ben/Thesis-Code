# SimplePingServer.py
# Ben Webb
# 12/6/2019

import socket

def main():
    """ create a TCP server """
    
    # create a TCP socket
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # assign IP address and port number to socket and open
    serverSocket.bind(('', 12345))
    serverSocket.listen(6)

    try:
        while True:
            # Create a new TCP connection
            connection, client_address = serverSocket.accept()
            try:
                while True:
                    # receive the as many packets as there are and echo
                    message = connection.recv(100).decode()
                    if message:

                        connection.sendall(message.encode())
                        continue
                    break
            finally:
                connection.close()

    except KeyboardInterrupt:
        serverSocket.close()


if __name__ == "__main__":
    main()
