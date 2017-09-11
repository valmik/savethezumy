#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#This is a SUBSCRIBER node that will read in the joint states of the UR5

#Import the dependencies 
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np

#More dependencies
import tf
import pdb
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Wrench
import kin_func_skeleton as kfs
import exp_quat_func as eqf
from time import time

t_old = time()
t_new = time()

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    global t_new
    global t_old

    t_new = time()

    t_sample = t_new - t_old
    t_old = t_new

    #Print the contents of the message to the console
    print("Force: %s" % str(message.force.z))
    print("")
    print("Sampling Time: %s" % str(t_sample))
    
    # print(rospy.get_name() + ": I heard %s" % message.data)

#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type sensor_msgs/JointState from the topic /joint_states.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("ati_ft_data", Wrench, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':
    listener()
