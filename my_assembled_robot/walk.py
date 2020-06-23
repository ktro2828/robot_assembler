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

rospy.init_node('send_motion')
scan_sub = rospy.Subscriber('/range_sensor', LaserScan, scan_callback)
act_client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

act_client.wait_for_server()

rate = rospy.Rate(10)
start = rospy.Time.now()
while (1):
    # gen msg
    traj_msg = FollowJointTrajectoryGoal()
    traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
    traj_msg.trajectory.joint_names = ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7']


    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-0.55, 0.4, 0.1, -0.4, 0.1, 0.4, -0.55, -0.4], #姿勢1
                                                           time_from_start = rospy.Duration(1))) ## 前の姿勢から2se
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.1, 0.4, -0.55, -0.4, -0.55, 0.4, 0.1, -0.4], #姿勢2
                                                           time_from_start = rospy.Duration(2)))## 前の姿勢から4sec
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-0.55, 0.4, 0.1, -0.4, 0.1, 0.4, -0.55, -0.4], #姿勢1
                                                        time_from_start = rospy.Duration(3))) ## 前の姿勢から2se
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.1, 0.4, -0.55, -0.4, -0.55, 0.4, 0.1, -0.4], #姿勢2
                                                            time_from_start = rospy.Duration(4)))## 前の姿勢から4sec
    '''
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[-0., 0, 0.7, 0, 0, 0, 0, 0], #姿勢3
                                                           time_from_start = rospy.Duration(10)))## 前の姿勢から4sec
    traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0.7, 0, 0.7, 0, -0.55, 0, -0.55, 0], #姿勢4
                                                           time_from_start = rospy.Duration(12)))
    '''

    # send to robot
    act_client.send_goal(traj_msg)

    act_client.wait_for_result()

    rospy.loginfo("done")
    if g_range_ahead < 0.8:
        break
