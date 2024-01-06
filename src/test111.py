import subprocess
import socket

HOST = '192.168.1.1'
# Enter IP or Hostname of your server
PORT = 12345
# Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

#Lets loop awaiting for your input
while True:
    data = s.recv(1024)
    if len(data)!=0:
        print(data.decode())
