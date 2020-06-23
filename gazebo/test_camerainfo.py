#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import LaserScan

def scan_cb(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

g_range_ahead = 1
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
