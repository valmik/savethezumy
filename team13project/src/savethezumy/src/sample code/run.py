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
from geometry_msgs.msg import WrenchStamped
import robot_state_subs as rss
import time

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
# jointState = None
jointState1 = None
jointState2 = None

def joint_state_callback1(message):

    #Print the contents of the message to the console
    # print("Joint Names: %s" % message.name)
    # print("")
    # print("Position: %s" % str(message.position))
    # print("")
    global jointState1
    jointState1 = message.position
   
    # print(rospy.get_name() + ": I heard %s" % message.data)

    #Python's syntax for a main() method

def joint_state_callback2(message):

    #Print the contents of the message to the console
    # print("Joint Names: %s" % message.name)
    # print("")
    # print("Position: %s" % str(message.position))
    # print("")
    global jointState2
    jointState2 = message.wrench.force

if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type sensor_msgs/JointState from the topic /joint_states.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    sub1 = rospy.Subscriber("joint_states", JointState, joint_state_callback1)
    sub2 = rospy.Subscriber("wrench", WrenchStamped, joint_state_callback2)
    print("Position: %s" % str(jointState1))
    print("Joint Names: %s" % jointState2)
    time.sleep(5)
    print("Position: %s" % str(jointState1))
    print("Joint Names: %s" % str(jointState2))
    time.sleep(5)
    print("Position: %s" % str(jointState1))
    print("Joint Names: %s" % str(jointState2))
    
    rospy.spin()


   
