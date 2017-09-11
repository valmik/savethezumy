#!/usr/bin/env python

""" Kinematics Code
Course: EE 106A, Fall 2016
Written by: Chris Berthelet, Eric Noordam, Valmik Prabhu, 12/5/16
Used by: EE106A Project, 12/5/16

This code controls the UR5, and checks the grid for hard and soft tiles

FUNCTIONS:
    -joint_state_callback()
    -force_sensor_callback()
     runZumy
     initNode()
     tags2base=createGridArray()
     moveToGrid()
     traverseGrid(tags2base)
     createZumyOutput(gridWoodBlocks)
     moveToHome()

VARIABLES:
    -JOINT_NAMES 
    -client 
    -jointState 
    -forceReading 
    -gridArray 
    -zumyPath 

"""


#More dependencies
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi as pi
import tf
import pdb
import rospy
import sys
import math
import numpy as np
# from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Transform, Vector3
import time
#IMPORT other .py files!
import initialization as INIT
import grid_traversal as GRTR
import output_to_zumy as OTZ
import subprocess
import kinematics as KIN
import zumy as ZUMY

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

#instantiate GLOBAL variables to store 
client = None
jointState = None
forceReading = None
gridArrayIDs = None
gridArrayValues = None
zumyPath = None

#ALL CALLBACK FUNCTIONS used w/ Listener nodes
def joint_state_callback(message):

    """
    Callback function to constantly update the global jointState variable via /joint_states topic
    """

    #Print the contents of the message to the console
    
    global jointState
    jointState = message
   
    # print(rospy.get_name() + ": I heard %s" % message.data)

def force_sensor_callback(message):

    """
    Callback function to constantly update the global forceReading variable via /ati_ft_data
    """

    #Print the contents of the message to the console
    
    global forceReading
    forceReading = message
   
    # print(rospy.get_name() + ": I heard %s" % message.data)

def initNode():

    """
    Callback function to constantly update the global forceReading variable via /ati_ft_data
    """

    # 0) General Setup
    #initialize listener node!
    rospy.init_node('main', anonymous=True)

    #Create instances of subscriber objects
    joint_state_sub = rospy.Subscriber("joint_states", JointState, joint_state_callback)


def createGridArray():

    """
    Callback function to constantly update the global forceReading variable via /ati_ft_data
    """

    #number of the AR_TAG at the start corner
    corner_tag = 21
    #number of tag on robot
    referenceTag = 'ar_marker_13'

    #array of the 25 AR_TAG numbers that are in use on the grid
    ar_tag_nums = np.array([21,25,5,9,12,28,23,27,20,10,39,35,14,8,22,6])

    #the known distance between ar_tags (i.e. 12cm)
    spacing = 0.12
    tfListener= tf.TransformListener()
    

    needData=False
    # Set the ids of tags on grid in order and generate robot base to tag transforms
    if needData:
        print("Generating Grid")
        global gridArrayIDs
        gridArrayIDs = INIT.grid_gen(corner_tag,ar_tag_nums,spacing,tfListener)
        print(gridArrayIDs)
        print("Grid Generated")
        tags2base= INIT.gridTransformsBase(gridArrayIDs,tfListener,referenceTag)
        np.save('gridArrayIDs',gridArrayIDs)
    # If we don't want to reread the camera, just load saved data
    else:
        gridArrayIDs=np.load('gridArrayIDs.npy')
        tags2base=np.load('tags2base.npy')
    print("Grid Transformed")
    return tags2base

def moveToGrid():
    #########################################################################################
    # 2) Forward Kinematics to move to favorable position
    #Capture current joint states in an array 
    time.sleep(.4)
    joint_states = np.array(rospy.wait_for_message("joint_states", JointState).position)
    print 'jjjjjjj'
    #Hard code in a Favorable Position that is close to the grid
    #This must be a 1x6 array input like in test_move.py code
    # favorable_pos = [0.51, -2.17, -1.18, -1.37, 1.53, 0] 
    favorable_pos = [pi/180*x for x in [-180, -40, 50, -100, -90, 0]]

    global client

    try:
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
        
        # print "This program will now make the robot move between the initial state:"
        # print str([joint_states[i]*180./pi for i in xrange(0,6)])
        print "Robot will now move to the favorable state:"
        print str([favorable_pos[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move to this pose!"
        
        inp = raw_input("Move to favorable position? y/n: ")[0]
        if (inp == 'y'):
            INIT.moveToAngles(favorable_pos,joint_states,client,JOINT_NAMES)
        else:
            print "Halting program"
            return

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        client.cancel_goal()
        raise 
    except:
        client.cancel_goal()
        raise

def traverseGrid(tags2base):
    zClearance = .06
    zTest = .1
    #Generate array to keep track of good or bad blocks that is the same size as the gridArrayIDs
    gridWoodBlocks = np.ones((np.size(gridArrayIDs),1))*2
    # Save the array to clear GUI
    np.save('gridWoodBlocks',gridWoodBlocks)

    inp = raw_input("Traverse Grid? y/n: ")[0]
    if (inp == 'y'):
        # First move down a bit
        z_des = tags2base[0][2] + zClearance
        GRTR.move_Z(client,JOINT_NAMES, z_des)

        #Loop through the testing method i = number of ar_tag times
        speed = 4 # First move slower than rest
        tfListener= tf.TransformListener()
        for i in range(0,gridArrayIDs.size):
            GRTR.move(client,JOINT_NAMES,i,tags2base,speed)
            gridWoodBlocks[i]=GRTR.test_Z(client,JOINT_NAMES,-zTest,tfListener)
            np.save('gridWoodBlocks',gridWoodBlocks)
            speed = 0.7

    else:
        print "Halting program"
    return gridWoodBlocks

def createZumyOutput(gridWoodBlocks):
    #Using the tested gridArrayValues generate an array of ar_tag numbers
    [startPos, zumyPath] = OTZ.create_path_DFS(gridWoodBlocks)

    if startPos > 5:
        zumyDead = True
        print("NO FEASIBLE PATH! Zumy is DOOOOOOOOOOOMED!")
    else:
        zumyDead = False
        arTagstoVisit = np.zeros(len(zumyPath))
        for i in range(0,len(zumyPath)):
            arTagstoVisit[i]=gridArrayIDs[zumyPath[i]]
        print('The Zumy should start at position ',startPos)    
        print 'The Zumy should go to tags: ',
        print arTagstoVisit
    
    #Publish the array to somewhere for the Zumy to pick it up!
    np.save('zumyDead',zumyDead)
    np.save('zumyPath',zumyPath)

def moveToHome():
    # Move robot to home position:
    home_pos = [pi/180*x for x in [0,-90,0,-90,0,0]]
    joint_states = np.array(rospy.wait_for_message("joint_states", JointState).position)
    INIT.moveToAngles(home_pos,joint_states,client,JOINT_NAMES)

def runZumy():
    ZUMY.main()

if __name__=='__main__':
    # Initialize arrays for the GUI
    np.save('gridWoodBlocks',np.ones(16)*2)
    inp = raw_input("Do you want to run the UR5?: y/n ")[0]
    if (inp == 'y'):        
        # print('Launching files')
        # subprocess.Popen('roslaunch ati_wnet_ft ft.launch', shell=True)
        # subprocess.Popen('roslaunch savethezumy run_all.launch', shell=True)
        # subprocess.Popen('rviz')
        # subprocess.Popen('roslaunch ur_modern_driver ur5_bringup_joint_limited.launch  robot_ip:=192.168.1.222', shell=True)
        # print('Waiting for boot')
        # time.sleep(1)
        try:
            np.save('zumyDead',False)
            proc = subprocess.Popen(['python','plotGUI.py'])
            initNode()
            tags2base=createGridArray()
            moveToGrid()
            gridWoodBlocks=traverseGrid(tags2base)
            createZumyOutput(gridWoodBlocks)
            moveToHome()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            client.cancel_goal
            proc.kill()
        except Exception as e:
            print e
    else:
        print "Skipping UR5"
    
    print "Please switch to the Zumy network!"
    inp = raw_input("Have you switched networks? y/n: ")[0]
    if (inp == 'y'):
        # subprocess.call('roslaunch odroid_machine remote_zumy.launch mname:=zumy7', shell=True)
        # time.sleep(5)
        runZumy()
        # subprocess.call('rosrun rqt_robot_steering rqt_robot_steering', shell=True)
    else:
        print "Halting program"
    


        