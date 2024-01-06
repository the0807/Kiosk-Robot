#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import subprocess
import socket
import multiprocessing
import time
from pythonds.basic.stack import Stack

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

HOST = '192.168.1.1'
PORT = 12345
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

LIN_VEL_STEP_SIZE = 0.3
ANG_VEL_STEP_SIZE = 0.5

e = """
Communications Failed
"""
target_linear_vel = 0.0
target_angular_vel = 0.0
control_linear_vel = 0.0
control_angular_vel = 0.0


def reset():
    global target_linear_vel
    global target_angular_vel
    global control_linear_vel
    global control_angular_vel
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def checkScan(cmd_q):
    go = True
    while True:
        
        data = subprocess.check_output(["rostopic", "echo", "/scan/ranges", "-n1"])
        data_arr = data.split(',')

        for i in range(140,190):
            data_scan = float(data_arr[i])
            #print(data_scan)
            if 0 < data_scan <= 0.5:
                print('dont move!')
                cmd_q.put('s')
                go = False
                break
            go = True
        if not go:
            continue
        cmd_q.put('g')
        print('get to go')
            


#multiprocessing part
cmd_q = multiprocessing.Queue()
cmd_p = multiprocessing.Process(target=checkScan,args=(cmd_q,))
cmd_p.start()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    ready = True
    return_way = Stack()

    try:
        while(1):
            data = s.recv(1)
            #s.send('ACK')
            if not cmd_q.empty():
                cmd = cmd_q.get()
                if cmd == 's':   
                    reset()
                    ready = False
                    print('start return---------------')
                    while not return_way.isEmpty():
                        time.sleep(0.25)
                        r_data = return_way.pop()
                        print(r_data)
                        if r_data == 'w' :
                            reset()
                            target_linear_vel = 0.26
                            status = status + 1
                            print(vels(target_linear_vel,target_angular_vel))
                        elif r_data == 'x' :
                            reset()
                            target_linear_vel = -0.26
                            status = status + 1
                            print(vels(target_linear_vel,target_angular_vel))
                        elif r_data == 'a' :
                            reset()
                            target_angular_vel = 1.82
                            status = status + 1
                            print(vels(target_linear_vel,target_angular_vel))
                        elif r_data == 'd' :
                            reset()
                            target_angular_vel = -1.82
                            status = status + 1
                            print(vels(target_linear_vel,target_angular_vel))
                        elif r_data == 's' :
                            target_linear_vel   = 0.0
                            control_linear_vel  = 0.0
                            target_angular_vel  = 0.0
                            control_angular_vel = 0.0
                            print(vels(target_linear_vel, target_angular_vel))
                        
                        if status == 20 :
                            status = 0
                        
                        twist = Twist()

                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                        pub.publish(twist)
                    
                    print('return finished-----------')
                else:
                    ready = True


            if ready:
                if data != '':
                    print(data.decode())
                    if data == 'w':
                        return_way.push('x')
                    if data == 'a':
                        return_way.push('d')
                    if data == 'd':
                        return_way.push('a')

                if data == 'w' :
                    reset()
                    target_linear_vel = 0.26
                    status = status + 1
                    #print(vels(target_linear_vel,target_angular_vel))
                elif data == 'x' :
                    reset()
                    target_linear_vel = -0.26
                    status = status + 1
                    #print(vels(target_linear_vel,target_angular_vel))
                elif data == 'a' :
                    reset()
                    target_angular_vel = 1.82
                    status = status + 1
                    #print(vels(target_linear_vel,target_angular_vel))
                elif data == 'd' :
                    reset()
                    target_angular_vel = -1.82
                    status = status + 1
                    #print(vels(target_linear_vel,target_angular_vel))
                elif data == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    #print(vels(target_linear_vel, target_angular_vel))
                else:
                    if (data == 'q'):
                        break

                if status == 20 :
                    status = 0

                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                pub.publish(twist)


    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)