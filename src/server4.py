import socket
import threading
from queue import Queue

def open_cv(conn, addr, queue):
	print("thread started")
	ACK = 0
	while True:
		try:
			data = conn.recv(1024)
			conn.send("ACK".encode())
			if len(data) == 0:
				conn.close()
				print(addr,"didnt send data, thread quit")
				break
			data = data.decode().split(',')
#print(data[0])
			if int(data[2])>700 and int(data[2])<1280:
				queue.put('d')
				print('put d command')
			if int(data[2])<600 and int(data[2])>0:
				queue.put('a')
				print('put a command')
			if int(data[0]) < 300 and int(data[0]) > 110 and int(data[2]) < 700 and int(data[2]) > 600:
				queue.put('w')
				print('put w command')

		except:
			conn.close()
			print(addr,"offline")
			break


def ros(conn, addr, queue):
	while True:
		try:
			if not queue.empty():
#data = queue.get().encode()
#				tmp = queue.get().encode()
#				while tmp == data:
#					tmp = queue.get().encode()
#				data = tmp
				conn.send(queue.get().encode())
				print('send command')
#data = conn.recv(1024)
#
#			if len(data) == 0:
#				conn.close()
#				print(addr,"didnt send data, thread quit")
#				break
		except:
			conn.close()
			print(addr,"offline")
			break

HOST = '192.168.1.1'
# Server IP or Hostname
PORT = 12345
# Pick an open Port (1000+ recommended), must match the client sport
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created")

#managing error exception
try:
        s.bind((HOST, PORT))
except socket.error:
        print("Bind failed")

s.listen(5)
print("Socket awaiting messages")

opencvToRos = Queue()

while True:
        conn, addr = s.accept()
#print(addr[0] == "192.168.1.101")
        if addr[0] == '192.168.1.101':
                t=threading.Thread(target=open_cv,args=(conn, addr, opencvToRos))
                t.start()
                print(addr," connected")
        if addr[0] == '192.168.1.102':
                t=threading.Thread(target=ros,args=(conn, addr, opencvToRos))
                t.start()
                print(addr," connected")
conn.close()
# Close connections
