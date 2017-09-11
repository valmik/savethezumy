#!/usr/bin/env python
""" Grid Traversal Code
Course: EE 106A, Fall 2016
Written by: Chris Berthelet, Eric Noordam, Valmik Prabhu 12/5/16
Used by: EE106A Project, 12/5/16

This code does contains functions to travesrse the 4x4 grid and test each block
"""

#More dependencies
import tf
# import pdb
import rospy
import sys
import math
from math import pi
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, Vector3, Wrench
import time
import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import kinematics as KIN

forceReading = None

def move(client,JOINT_NAMES,index,tags2base, speed = 8):
    """
    Figures out where to move. Split it into an XY move and a Z move. Repeat XY move until correct
        
    Args:
    client - 
    JOINT_NAMES - 
    index - 
    tags2base - 
    speed - 

    """

    current_state = np.array(rospy.wait_for_message("joint_states", JointState).position)

    q_des = tags2base[index,:]

    q_current = KIN.fk(current_state)

    # We only care about XY motion
    q_des = np.array([q_des[0], q_des[1], q_current[2]])
    q_des[0] = q_des[0] + 0.03-0.004*int(index/5)

    # Define a new one that we can change

    move_sub(client, current_state, JOINT_NAMES, q_des, speed)

def move_sub(client,joint_states,JOINT_NAMES,q_des, speed):
    """
    Actually moves the arm

    """

    q_current = KIN.fk(joint_states)
    q_ik = q_des

    # print "Goal: ", q_des

    while True:
        final_states = KIN.ik(q_ik)

        ## Hacky fixes
        # Fix theta4
        final_states[3] = -final_states[2] - final_states[1] - math.pi/2
        # Stop theta1 from going all the way around
        if final_states[0] > 0:
            if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] - math.pi*2)):
                final_states[0] = final_states[0] - 2*math.pi
        elif final_states[0] < 0:
            # print "yo"
            # print abs(joint_states[0] - final_states[0])
            # print abs(joint_states[0] - (final_states[0] + math.pi*2))
            if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] + math.pi*2)):
                final_states[0] = final_states[0] + 2*math.pi


        q_current = KIN.fk(final_states)
        # print "Current p:", q_current, "Distance: ", np.linalg.norm(q_current - q_des)

        if np.linalg.norm(q_current - q_des) <= 0.001:
            break
        else:
            q_ik = q_ik + (q_des - q_current)/2

    # raw_input("Check before moving")

    # print final_states
    # raw_input()


    # Actually move
    move_speed = speed
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joint_states, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=final_states, velocities=[0]*6, time_from_start=rospy.Duration(move_speed))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_Z(client,JOINT_NAMES,zTest):

    #Get current robot joint angles
    joint_states = np.array(rospy.wait_for_message("joint_states", JointState).position)
    
    #Solve XY IK solution
    z_des = zTest
    z_current = KIN.fk(joint_states)[2]

    z_ik = z_des - z_current
    # print "z_ik", z_ik
    while True:
        final_states = KIN.move_z(z_ik, np.array(joint_states))
        z_current = KIN.fk(final_states)[2]
        # print "z_current", z_current
        # raw_input()
        if abs(z_current - z_des) <= 0.001:
            break
        else:
            # print "z_des"
            z_ik = (z_des - z_current)/2
            # print "z_ik", z_ik

    # raw_input("It's gunna move now")
    # print final_states
    if final_states[0] > 0:
        if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] - math.pi*2)):
            final_states[0] = final_states[0] - 2*math.pi
    elif final_states[0] < 0:
        # print "yo"
        # print abs(joint_states[0] - final_states[0])
        # print abs(joint_states[0] - (final_states[0] + math.pi*2))
        if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] + math.pi*2)):
            final_states[0] = final_states[0] + 2*math.pi

    # print final_states
    # raw_input()

    #Set movement speed
    #This will be MUCH slower so we will set a velocity variable here to calculate time
    #z_dist = 2 in travel
    # move_time_slow = 5.0
    move_time = 2.0

    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try: 
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joint_states, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=final_states, velocities=[0]*6, time_from_start=rospy.Duration(move_time))]
        
        #Do not insert a wait_for_result in order to allow for interrupts
        client.send_goal(g)
        client.wait_for_result()

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise


def test_Z(client,JOINT_NAMES,zTest,tfListener):

    """
    Reads the force vs. time values from the sensor and sets the tested block to wood (1) or foam (0)
    """

    global forceReading

    #Create instance of force sensor subscriber
    force_sensor_sub = rospy.Subscriber("ati_ft_data", Wrench, force_sensor_callback)

    #Get current robot joint angles
    joint_states = np.array(rospy.wait_for_message("joint_states", JointState).position)

    # print "Initial position", KIN.fk(joint_states)
    
    #Solve XY IK solution
    # (trans, rot) = tfListener.lookupTransform('base','ee_link', rospy.Time(0))

    z_des = zTest
    z_ik = z_des

    while True:
        final_states = KIN.move_z(z_ik, np.array(joint_states))
        z_current = KIN.fk(final_states)[2]
        if abs(z_current - z_des) <= 0.001:
            break
        else:
            z_ik = z_ik + (z_des - z_current)/2

    # print z_current

    # print "Predicted position", KIN.fk(final_states)
    

    # print final_states
    if final_states[0] > 0:
        if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] - math.pi*2)):
            final_states[0] = final_states[0] - 2*math.pi
    elif final_states[0] < 0:
        if abs(joint_states[0] - final_states[0]) > abs(joint_states[0] - (final_states[0] + math.pi*2)):
            final_states[0] = final_states[0] + 2*math.pi

    time.sleep(.2) #This allows the subscriber to obtain the first force reading

    #Set movement speed
    #This will be MUCH slower so we will set a velocity variable here to calculate time
    #z_dist = 2 in travel
    move_time_slow = 8.0
    move_time = 0.5

    ##SENSOR sample time == approx. 0.008 seconds

    #Get force value from sensor
    vector_force = forceReading.force

    value_force_calib = np.array([vector_force.x, vector_force.y, vector_force.z])
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try: 
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joint_states, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=final_states, velocities=[0]*6, time_from_start=rospy.Duration(move_time_slow))]
        
        #Do not insert a wait_for_result in order to allow for interrupts
        client.send_goal(g)
        # client.wait_for_result()
        start_time = time.time()

        value_force = 0
        maxForce=0
        force_threshold = 10000
        force_min_thresh = 10
        wood_force_thresh = 250


        detectedWood = 0
        forcePts = 20
        forceArray = np.zeros(forcePts)
        i=0
        # continuously read the force values in a loop
        while True:
            init_force = np.array([forceReading.force.x, forceReading.force.y, forceReading.force.z])
            value_force = np.linalg.norm(init_force - value_force_calib)

            # Check if over start threshold and haven't already collected enough points
            if value_force>force_min_thresh and i<forcePts:
                # print "Z force", value_force
                # print "Total force", total_force
                forceArray[i]=value_force
                i=i+1
                time.sleep(.005)
            # If we've collected enough points check the sloope
            elif i>=forcePts:
                # print value_force
                # print forceArray
                avg_force = np.mean(forceArray)
                print "Average Force", avg_force
                if avg_force > wood_force_thresh:
                    detectedWood=1
                    # print 'This is wood'
                else:
                    detectedWood=0
                    # print "This is foam"
                break
        # After testing stop the robot
        client.cancel_goal()

        # Raise the end effector back up
        curr_state = rospy.wait_for_message("joint_states", JointState).position

        g.trajectory.points = [
            JointTrajectoryPoint(positions=curr_state, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=joint_states, velocities=[0]*6, time_from_start=rospy.Duration(move_time))]
        client.send_goal(g)
        client.wait_for_result()

        # Return whether or not we detected wood.
        return detectedWood

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def force_sensor_callback(msg):
    """
    Callback function to constantly update the global forceReading variable with the current value from the sensor
    """

    global forceReading
    forceReading = msg

