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
    data1 = subprocess.check_output(["rostopic", "echo", "/sonar_dist", "-n1"])
    data2 = subprocess.check_output(["rostopic", "echo", "/scan/ranges", "-n1"])
    data = data1[6:17] + '\n' + data2
    s.send(data)
    #reply = s.recv(2)
    print(data)

exit()
