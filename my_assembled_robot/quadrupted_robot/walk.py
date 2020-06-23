#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = msg.ranges[len(msg.ranges)/2]

# initial value of g_range_ahead
g_range_ahead = 1

rospy.init_node('send_motion', anonymous=True)
scan_sub = rospy.Subscriber('/range_sensor', LaserScan, scan_callback)
act_client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

act_client.wait_for_server()

rate = rospy.Rate(10)
start = rospy.Time.now()
while not rospy.is_shutdown():
    # gen msg
    traj_msg = FollowJointTrajectoryGoal()
    traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
    traj_msg.trajectory.joint_names = ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7']


    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-0.55, 0.4, 0.1, -0.4, 0.1, 0.4, -0.55, -0.4],
                                                           time_from_start = rospy.Duration(1)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.1, 0.4, -0.55, -0.4, -0.55, 0.4, 0.1, -0.4],
                                                           time_from_start = rospy.Duration(2)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-0.55, 0.4, 0.1, -0.4, 0.1, 0.4, -0.55, -0.4],
                                                        time_from_start = rospy.Duration(3)))
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.1, 0.4, -0.55, -0.4, -0.55, 0.4, 0.1, -0.4],
                                                            time_from_start = rospy.Duration(4)))

    # send to robot
    act_client.send_goal(traj_msg)

    act_client.wait_for_result()

    rospy.loginfo("done")
