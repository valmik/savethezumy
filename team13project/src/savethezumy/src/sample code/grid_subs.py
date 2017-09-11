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

#More dependencies
import tf
import pdb
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    print("Joint Names: %s" % message.name)
    print("")
    print("Position: %s" % str(message.position))
    print("")

    # print(rospy.get_name() + ": I heard %s" % message.data)

#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type sensor_msgs/JointState from the topic /joint_states.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("joint_states", JointState, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':
    listener()


#ar_tag_subs.py code

#!/usr/bin/env python
import tf
import pdb
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf

listener = None

def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """

    [omega, theta] = eqf.quaternion_to_exp(rot)

    g = eqf.create_rbt(omega, theta, trans)

    return g

def compute_twist(rbt):
    """
    Computes the corresponding twist for the rigid body transform

    Input:
        rbt - (4,4) ndarray 

    Output:
        v - (3,) array
        w - (3,) array
    """

    R = rbt[0:3][:,0:3]
    trans = rbt[0:3][:,3:4]

    [omega, theta] = eqf.find_omega_theta(R)

    v = eqf.find_v(omega, theta, trans)

    #pdb.set_trace()

    v = np.array([v[0,0],v[1,0],v[2,0]])

    twist = np.array([v[0],v[1],v[2],omega[0],omega[1],omega[2]])

    return twist

if __name__=='__main__':
    rospy.init_node('ar_tags_subs')
    if len(sys.argv) < 4:
        print('Use: ar_tag_subs.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
            rot = np.array(rot)
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print twist
        except Exception as e:
            print(e)
            

        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['ar1'], rospy.Time(0))
            rot = np.array(rot)
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
            print twist
        except Exception as e:
            print(e)
            
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['arZ'], rospy.Time(0))
            rot = np.array(rot)
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
            print twist
        except Exception as e:
            print(e)

        rate.sleep()