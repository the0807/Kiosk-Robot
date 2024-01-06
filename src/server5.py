import socket
import threading
from queue import Queue


status = 0    # 0 = waiting msg from ROS, 1 = counting 60s, 2 = command ROS to go back , x --> not go back , turn aroundß
lock = threading.Lock()
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
			if int(data[2])>800 and int(data[2])<1280:
				queue.put('d')
				print('put d command')
			if int(data[2])<500 and int(data[2])>0:
				queue.put('a')
				print('put a command')
			if int(data[0]) < 300 and int(data[0]) > 60 and int(data[2]) < 800 and int(data[2]) > 500:
				queue.put('w')
				print('put w command')

		except:
			conn.close()
			print(addr,"offline")
			break


def msgRecv(conn, addr):
    global status
    while True:
        if status == 0:
            conn.recv(16)
            lock.acquire()
            status = 1
            lock.release()
        if status == 2:
            conn.send('x'.encode())
            lock.acquire()
            status = 0
            lock.release()

def ros(conn, addr, queue):
    t=threading.Thread(target=msgRecv,args=(conn, addr))
    t.start()
    while True:
        try:
            if not queue.empty():
                conn.send(queue.get().encode())
                print('send command')

        except:
            conn.close()
            print(addr,"offline")
            break

def UI(conn, addr):
    global status
    while True:
        try:
            if status == 1:
                conn.settimeout(60)
                conn.recv()
                lock.acquire()
                status = 2
                lock.release()
        except socket.timeout:
            lock.acquire()
            status = 0
            lock.release()

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
        if addr[0] == '192.168.1.1':
                t=threading.Thread(target=UI,args=(conn, addr))
                t.start()
                print(addr," connected")
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
