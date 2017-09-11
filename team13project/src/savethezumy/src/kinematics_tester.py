#!/usr/bin/env python
""" Kinematics Testing Code
Course: EE 106A, Fall 2016
Written by: Valmik Prabhu, 12/12/16
Used by: EE106A Project, 12/12/16

Tests inverse Kinematics code

Sources:
"""

import numpy as np
from numpy import linalg
import math


import roslib; roslib.load_manifest('ur_driver')
import rospy
import tf
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

import kinematics as kin


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']



def run_ik(q_des):
    """
    Moves the robot

    Args:
    q_des - (3,) ndarray: desired position

    Returns:
    """

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        theta = kin.ik(q_des)

        if theta[0] > 0:
            if abs(joints_pos[0] - theta[0]) > abs(joints_pos[0] - (theta[0] - math.pi*2)):
                theta[0] = theta[0] - 2*math.pi
        elif theta[0] < 0:
            if abs(joints_pos[0] - theta[0]) > abs(joints_pos[0] - (theta[0] + math.pi*2)):
                theta[0] = theta[0] + 2*math.pi

        theta[3] = -theta[1]-theta[2]-math.pi/2

        # print kin.rad2deg(theta)

        # inp = raw_input("Continue? y/n: ")[0]
        inp = 'y'
        if (inp == 'y'):
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=theta, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
            client.send_goal(g)
            client.wait_for_result()

        else:
            print "Halting program"

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def run_fk(theta):
    """
    Moves the robot

    Args:
    q_des - (6,) ndarray: desired joint joint_states

    Returns:
    """

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        # print kin.rad2deg(theta)

        # inp = raw_input("Continue? y/n: ")[0]
        inp = 'y'
        if (inp == 'y'):
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=theta, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
            client.send_goal(g)
            client.wait_for_result()

        else:
            print "Halting program"

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise


def move_to_q(q_des):
    """
    Moves the robot. Wrapper for run_ik.

    Args:
    q_des - (3,) ndarray: desired position

    Returns:
    """
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        #print str([Q1[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):

            run_ik(q_des)
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

def move_to_theta(theta):
    """
    Moves the robot. Wrapper for run_fk.

    Args:
    theta - (6,) ndarray: desired joint states

    Returns:
    """
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        #print str([Q1[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            run_fk(theta)
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

def test_fk():
    """
    Tests forward kinematics to make sure that joints work as expected

    Args:

    Returns:
    """

    move_to_theta(np.array([kin.deg2rad(181.25), kin.deg2rad(-43.35), kin.deg2rad(62.58), kin.deg2rad(-108.99), kin.deg2rad(-90), kin.deg2rad(0)]))

def test_ik():
    """
    Tests forward kinematics to make sure that joints work as expected

    Args:

    Returns:
    """

    move_to_theta(np.array([kin.deg2rad(-90), kin.deg2rad(-40), kin.deg2rad(50), kin.deg2rad(-100), kin.deg2rad(-90), kin.deg2rad(0)]))

    # Figure out the current position
    listener = tf.TransformListener()
    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "initial q: ", q_des

    move_to_q(q_des)

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "new q; should be the same:", q_des

    move_to_q(q_des - np.array([0, 0, 0.1]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved down 0.1 from previous: ", q_des

    move_to_q(q_des + np.array([0, 0, 0.1]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved back up 0.1: ", q_des

    move_to_q(q_des - np.array([0.2, 0, 0]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved inwards 0.2: ", q_des

    move_to_q(q_des - np.array([-0.2, 0.3, 0]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved right 0.3 and forwards 0.2: ", q_des

    move_to_q(q_des - np.array([0, 0, 0.1]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved down 0.1: ", q_des

    move_to_q(q_des - np.array([0, 0, 0.1]))

    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "moved up 0.1: ", q_des

def test_z():
    move_to_theta(np.array([kin.deg2rad(181.25), kin.deg2rad(-43.35), kin.deg2rad(62.58), kin.deg2rad(-108.99), kin.deg2rad(-90), kin.deg2rad(0)]))

    listener = tf.TransformListener()
    a = 0
    while a == 0:
        try:
            (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
            a=1
        except Exception as e: 
            a=0

    q_des = np.array(trans)

    print "initial q: ", q_des

    move_to_q(q_des - np.array([0, 0.3, 0]))

    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position

    joints = np.array(joints_pos)[:4]
    joints[3] = joints[3] + math.pi/2

    g_d = kin.fk_helper(joints)
    g_d[2, 3] = g_d[2, 3] - 0.1

    theta = kin.ik_helper(g_d)
    theta[3] = theta[3]-math.pi/2
    theta = np.append(theta, [-math.pi/2, 0])
    move_to_theta(theta)


if __name__ == '__main__':
    test_ik()

