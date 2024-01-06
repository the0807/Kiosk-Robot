#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

#Lets loop awaiting for your input
def callback(msg):
    for i in range(140, 190):
        print('%.3f' %msg.ranges[i])
    print('\n')

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()