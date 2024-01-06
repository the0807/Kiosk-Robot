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
s.settimeout(5)

LIN_VEL_STEP_SIZE = 0.3
ANG_VEL_STEP_SIZE = 0.5

e = """
Communications Failed
"""
target_linear_vel = 0.0
target_angular_vel = 0.0
control_linear_vel = 0.0
control_angular_vel = 0.0

class RobotControl():

    def __init__(self, pub):
        #rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = pub #rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.stop_robot()
        self.ctrl_c = True

    def stop_robot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def get_inputs_rotate(self):
        self.angular_speed_d = 60
        self.angle_d = 180
        clockwise_yn = 'y'  #y = clock / n = ccw
        if clockwise_yn == "y":
            self.clockwise = True
        if clockwise_yn == "n":
            self.clockwise = False

        return [self.angular_speed_d, self.angle_d]

    def convert_degree_to_rad(self, speed_deg, angle_deg):

        self.angular_speed_r = speed_deg * 3.14 / 180
        self.angle_r = angle_deg * 3.14 / 180
        return [self.angular_speed_r, self.angle_r]

    def rotate(self):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        # Convert speed and angle to radians
        speed_d, angle_d = self.get_inputs_rotate()
        self.convert_degree_to_rad(speed_d, angle_d)

        # Check the direction of the rotation
        if self.clockwise:
            self.cmd.angular.z = -abs(self.angular_speed_r)
        else:
            self.cmd.angular.z = abs(self.angular_speed_r)

        # t0 is the current time
        t0 = rospy.Time.now().secs

        current_angle = 0

        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (current_angle < self.angle_r):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            # t1 is the current time
            t1 = rospy.Time.now().secs
            # Calculate current angle
            current_angle = self.angular_speed_r * (t1 - t0)
            self.rate.sleep()

        # set velocity to zero to stop the robot
        #self.stop_robot()



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
    isWaiting = False
    cmd = ''
    tmp = ''
    while True:
        
        data = subprocess.check_output(["rostopic", "echo", "/scan/ranges", "-n1"])
        data_arr = data.split(',')
        for i in range(140,190):
            data_scan = float(data_arr[i])
            #print(data_scan)
            if 0 < data_scan <= 0.5:
            	tmp = 's'
            	go = False
            	if tmp != cmd:
            	    cmd = tmp
            	    #print('dont move!')
                    cmd_q.put(cmd)
                if not isWaiting:
                    isWaiting = True
                    time.sleep(7)

                else:
                    isWaiting = False
                    tmp = 'x'
                    if tmp != cmd:
            		cmd = tmp
            		#print('return')
            		cmd_q.put(cmd)
                break
            go = True
        
        if not go:
            continue

        tmp = 'g'

        if tmp != cmd:
            cmd = tmp
            #print('get to go')
            cmd_q.put(cmd)
    	
            

def move(r_data, target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel):
    if r_data == 'w' :
        reset()
        target_linear_vel = 0.26
        #print(vels(target_linear_vel,target_angular_vel))
    elif r_data == 'x' :
        reset()
        target_linear_vel = -0.26
        #print(vels(target_linear_vel,target_angular_vel))
    elif r_data == 'a' :
        reset()
        target_angular_vel = 1.82
        #print(vels(target_linear_vel,target_angular_vel))
    elif r_data == 'd' :
        reset()
        target_angular_vel = -1.82
        #print(vels(target_linear_vel,target_angular_vel))
    elif r_data == 's' :
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        #print(vels(target_linear_vel, target_angular_vel))
    
    
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    pub.publish(twist)


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
    data = ''
    #isWaiting = False
    try:
        while(1):
            try:
                data = s.recv(1)
            except socket.timeout:
                #print('time out')
                data = ''
                pass
            #s.send('ACK')
            if not cmd_q.empty():
                cmd = cmd_q.get()
                print cmd
                if cmd == 's':   
                    reset()
                    ready = False
                    
                elif cmd == 'x':
                    print('start return---------------')
                    robotcontrol_object = RobotControl(pub)
                    res = robotcontrol_object.rotate()
                    while not return_way.isEmpty():
                        time.sleep(0.29)
                        r_data = return_way.pop()
                        print(r_data)
                        move(r_data, target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel)
                    print('return finished------------')
                    res = robotcontrol_object.rotate()
                else:
                    ready = True
        
            if ready:
                
                if data != '':
                    print(data.decode())
                    return_way.push(data)
                    move(data, target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel)

    except socket.timeout:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
