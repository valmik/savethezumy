#!/usr/bin/env python
""" Initialization Code
Course: EE 106A, Fall 2016
Written by: Chris Berthelet, Eric Noordam 12/5/16
Used by: EE106A Project, 12/5/16

This code uses creates and stores the grid array of AR tag positions to be used in Inverse Kinematics calculations
This code also moves the UR5 to a desired position, given a set of joint angles
"""

#More dependencies
import tf
import pdb
import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import JointState
import time
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

listener = None

def grid_gen(corner_tag, ar_tag_nums, spacing,tfListener):

    """
    Uses camera/tf data to map AR tags into a 4x4 grid array
    """

    # Set the board properties
    xspacing = spacing
    yspacing = spacing
    tol = .08
    direction = -1 #right
    boardLength = np.sqrt(ar_tag_nums.size)
    try:
        grid = np.zeros((ar_tag_nums.size,1))
        grid[0]=corner_tag
        for i in range(0,ar_tag_nums.size-1): #loop over number of tags
            currentTag = int(grid[i])
            for j in range(0,ar_tag_nums.size): #check all tags
                nextTag = int(ar_tag_nums[j])
                print('Cur',currentTag,'Next',nextTag)
                print grid.T
                a=0
                while a == 0:
                    try:
                        (trans, rot) = tfListener.lookupTransform('ar_marker_'+str(currentTag),'ar_marker_'+str(nextTag), rospy.Time(0))
                        a=1
                    except Exception as e: 
                        # print e
                        a=0

                # print(trans)
                x = trans[1]
                y = trans[0]
                print(x,y)
                # print(direction*x>0)
                # print((abs(x) > (xspacing-tol)))
                # print((abs(x) < (xspacing+tol)))
                if (i+1)%boardLength==0: #check if at end
                     if (abs(x)<tol)&((y < (yspacing+tol)) & (y > (yspacing-tol))):
                        grid[i+1]=nextTag
                        direction = direction*-1
                        break
                elif (abs(y)<tol)& (abs(x) < (xspacing+tol)) & (abs(x) > (xspacing-tol)) & (direction*x>0):
                        grid[i+1]=nextTag
                        break
    except Exception as e:
        print(e)
    return grid

def gridTransformsBase(grid,tfListener,referenceTag): 

    """
    Generates transforms between grid AR tags and the tag on the base of the UR5
    """

    tags2base = np.zeros((grid.size,3))
    offset = 0.02
    marker2base = np.array([.255 - .065 + offset, .2375 -.065 - offset, 0.012 + .030])
    for i in range(0,grid.size): 

        while a == 0:
            try:
                (trans, rot) = tfListener.lookupTransform(referenceTag, 'ar_marker_'+str(int(grid[i])), rospy.Time(0))
                print trans

                z = np.array(trans) + marker2base

                tags2base[i,:] = z

                # pdb.set_trace()
                a=1
            except Exception as e: 
                a=0
                print(e)
                continue
    np.save('tags2base',tags2base)
    print tags2base[0,:]
    return tags2base

def gridTransformsCam(grid,tfListener):

    """
    Generates transforms between AR tags and camera frame
    """

    tags2cam = np.zeros((grid.size,3))
    for i in range(0,grid.size): 
        a=0
        while a == 0:
            try:
                (trans, rot) = tfListener.lookupTransform('usb_cam','ar_marker_'+str(int(grid[i])), rospy.Time(0))
                tags2cam[i,:]=np.array(trans)
                a=1
            except Exception as e: 
                a=0
                print(e)
                continue
        a = 0

    np.save('tags2cam',tags2cam)

    return tags2cam

# def init_favorable_pos():
def moveToAngles(jointAngles,joint_states,client,JOINT_NAMES):

    """
    Moves UR5 to a position given desired set of joint angles. Used when moving UR5 into and out of favorable positions
    """

    ##Set movement speed
    #This is the default speed (i.e. time to this position) in test_move
    move_speed = 8.0
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joint_states, velocities=[0]*6, time_from_start=rospy.Duration(0.0,0)),
            JointTrajectoryPoint(positions=jointAngles, velocities=[0]*6, time_from_start=rospy.Duration(move_speed,0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise  
   