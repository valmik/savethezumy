#!/usr/bin/env python
""" Zumy Code
Course: EE 106A, Fall 2016
Written by: Mimi Parker and Valmik Prabhu, 12/11/16
Used by: EE106A Project, 12/11/16

This code moves the zumy along the path


Sources:


"""

import numpy as np
import scipy as sp
from numpy import linalg
import math
import sys
import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import kin_func_skeleton as kfs
import exp_quat_func as eqf
import zumy_constants as zumyc
import initialization as INIT
import time


def read_position(tagInd,tags2cam):
    """
    Reads the position of the tag from the list of tags
        
    Args:
    tag: the position and rotation of each tag

    Returns:
    position - (3,) ndarray: the position of the tag in the camera frame
    """

    position = tags2cam[tagInd][:3]
    # position = tags2cam
    position = np.array(position)
    return position

def read_position_live(tagInd):
    """
    Reads the position of the tag from the list of tags
        
    Args:
    tag: the position and rotation of each tag

    Returns:
    position - (3,) ndarray: the position of the tag in the camera frame
    """
    gridArrayIDs=np.load('gridArrayIDs.npy',)
    tagnum = int(gridArrayIDs[tagInd])
    (pz, rz) = listener.lookupTransform('usb_cam', 'ar_marker_'+str(tagnum), rospy.Time(0))
            
    # position = tags2cam[tagInd][:3]
    # position = tag
    position = np.array(pz)

    return position

def transform_z(p_gc, p_zc, r_zc):
    """
    Converts a point in the camera frame to a point in the zumy frame
        
    Args:
    p_cg - (3,) ndarray: a point in the camera frame
    p_cz - (3,) ndarray: the position of the zumy in the camera frame
    r_cz - (4,) ndarray: the rotation (quaternion) from the camera frame to the zumy frame

    Returns:
    position - (3,) ndarray: the position of the point in the zumy frame
    """

    [omega_zc, theta_zc] = eqf.quaternion_to_exp(r_zc)
    g_zc = np.round(eqf.create_rbt(omega_zc, theta_zc, p_zc), 5)
    g_zc = np.linalg.inv(g_zc)
    # print theta_zc
    position = np.dot(g_zc, np.append(p_gc, 1))[:3]
    # position = np.dot(g_zc, np.append(p_gc, 1))

    return position

def check_pos(pos):
    """
    Converts a point in the camera frame to a point in the zumy frame
        
    Args:
    pos - (3,) ndarray: position of the tag in the zumy frame

    Returns:
    reached - boolean: TRUE if the zumy is close enough to the tag
    """

    p = pos[:2]
    reached = np.linalg.norm(p) <= zumyc.position_tolerance

    return reached

def drive(publisher, x, rz):
    """
    moves zumy to the goal
        
    Args:
    publisher: zumy publisher
    x: desired x velocity
    rz: desired z angular velocity

    Returns:
    """

    twistMessage = Twist()
    l = Vector3()
    l.x = x
    l.y = 0
    l.z = 0
    twistMessage.linear = l
    v = Vector3()
    v.x = 0
    v.y = 0
    v.z = rz
    twistMessage.angular = v
    # print(twistMessage)
    publisher.publish(twistMessage)



def move_final():

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumyc.zumy, Twist, queue_size=2)


    time.sleep(1.0)
    drive(zumy_vel, 0.1, 0)
    time.sleep(1.5)
    drive(zumy_vel, 0, 0)
    print "We made it ACROSS! :D"
    print "Party Dance!"
    drive(zumy_vel, 0, 1)
    time.sleep(5.0)
    drive(zumy_vel, 0, 0)
            #PARTY DANCE

def move_zumy(publisher, goal_z, count):
    """
    moves zumy to the goal
        
    Args:
    publisher: zumy publisher
    goal_z - (3,) ndarray: position of the tag in the zumy frame
    count - int: tag index (1st, 2nd, etc.)

    Returns:
    """

    ### This is where most of my code from lab 8 should be

    theta = math.atan(goal_z[1]/goal_z[0])
    if goal_z[0] < 0:
        theta = math.pi + theta
    elif goal_z[0] > math.pi:
        theta = theta - 2*math.pi

    p = goal_z[:2]
    distance = np.linalg.norm(p)
    #print 'x', p[0], 'y', p[1], 'dist = ', np.around(np.linalg.norm(p), decimals=3), 'theta = ', np.around(theta*190/math.pi, decimals=3)
    if abs(theta) < zumyc.angle_tolerance: # need to add "AND dist < threshold"?
        print "D to ", count, ": dist = ", np.around(np.linalg.norm(p), decimals=3), " theta = ", np.around(theta*180/math.pi, decimals=3)
        drive(publisher, 0.025, 0)   # previously 0.01
    else:
        print "T to ", count, ": dist = ", np.around(np.linalg.norm(p), decimals=3), " theta = ", np.around(theta*180/math.pi, decimals=3)
        if abs(theta*0.5) > 0.2:
            drive(publisher, 0, theta*0.4) # previously 0.3
        else:
            driveTheta = 0.3*theta/abs(theta)
            # print driveTheta
            drive(publisher, 0, driveTheta)  




def move(goal_c, count):
    """
    Moves the zumy to the goal
        
    Args:
    goal_c - (3,) ndarray: position of the goal in the camera frame 
    count - int: tag index (1st, 2nd, etc.)
    

    Returns:
    """

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumyc.zumy, Twist, queue_size=2)


    while not rospy.is_shutdown():

        ### Find zumy position and rotation relative to the camera frame
        ### You could listen to ar_tag_alvar or do a lookup transform on tf
        try:
            # print zumyc.zumytag
            (pz, rz) = listener.lookupTransform('usb_cam', zumyc.zumytag, rospy.Time(0))
            ### Kalman Filter ###
            # rospy.wait_for_service('innovation')
            # KF = rospy.ServiceProxy('innovation', NuSrv)
            # transformMessage = Transform()
            # transformMessage.translation = Vector3(trans[0], trans[1], trans[2])
            # transformMessage.rotation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            # kfMessage = NuSrvRequest()
            # kfMessage.transform = transformMessage
            # kfMessage.origin_tag = ar_tags['ar1']
            # KF(kfMessage)
        except Exception as e:
            # print 'oops'
            # print e
            continue
        else:
            # print 'oooooops'
            pass
        pz = np.array(pz)
        rz = np.array(rz)

        ### Calculate the position of the goal in relation to the zumy frame
        # print 'goal', goal_c
        # print 'pz', pz
        # print 'rz', rz
        goal_z = transform_z(goal_c, pz, rz)
        

        # print goal_z

        #this code checks if the zumy has reached the target, and exits the function if it has
        if check_pos(goal_z):
            print "Stopping"
            drive(zumy_vel, 0, 0)
            #time.sleep(1.0)
            #drive(zumy_vel, 0, 0)
            #PARTY DANCE
            # drive(zumy_vel, 0, 0)
            return
        else:
            move_zumy(zumy_vel, goal_z, count)

def main():
    """
    Reads the arrays from the ur5. Then moves the zumy along the path
        
    Args:    

    Returns:
    """
    rospy.init_node('follow_ar_tag_twist')

    tfListener= tf.TransformListener()
    gridArrayIDs=np.load('gridArrayIDs.npy')    
    
    tags2cam= INIT.gridTransformsCam(gridArrayIDs,tfListener)

    count = 1
    zumyPath=np.load('zumyPath.npy')
    # tags2cam=np.load('tags2cam.npy')
    
    for tagInd in zumyPath:
        tagInd=int(tagInd)
        pos = read_position(tagInd,tags2cam)
        print pos
        print 'Going to: ', count
        print 'trying to get to tag', gridArrayIDs[tagInd]
        move(pos, count)
        # drive(zumy_vel, .1, 0)
        count = count + 1
    print 'We made it!'
    move_final()
    
if __name__ == "__main__": 
    # rospy.init_node('main', anonymous=True)
    # spacing = 0.13
    # tfListener= tf.TransformListener()
    # ar_tag_nums = np.array([7,5,40,39,25,13,20,22,12,14,37,29,9,8,27,21])
    # corner_tag=21
    # gridArrayIDs = INIT.grid_gen(corner_tag,ar_tag_nums,spacing,tfListener)
    # [tags2base,tags2cam] = INIT.gridTransforms(gridArrayIDs,tfListener)
    # np.save('gridArrayIDs',gridArrayIDs)    

    main()






