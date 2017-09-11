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
            (trans, rot) = listener.lookupTransform('ar_marker_9','ar_marker_10', rospy.Time(0))
            # (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
            rot = np.array(rot)
            rbt = return_rbt(trans=trans, rot=rot)
            print('gab between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print rbt
            twist = compute_twist(rbt=rbt)
            print('twist between ' + ar_tags['arZ'] + ' and ' + ar_tags['ar1'])
            print twist
        except Exception as e:
            print(e)
            

        # try:
        #     (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['ar1'], rospy.Time(0))
        #     rot = np.array(rot)
        #     rbt = return_rbt(trans=trans, rot=rot)
        #     print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
        #     print rbt
        #     twist = compute_twist(rbt=rbt)
        #     print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['ar1'])
        #     print twist
        # except Exception as e:
        #     print(e)
            
        # try:
        #     (trans, rot) = listener.lookupTransform(ar_tags['ar0'], ar_tags['arZ'], rospy.Time(0))
        #     rot = np.array(rot)
        #     rbt = return_rbt(trans=trans, rot=rot)
        #     print('gab between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
        #     print rbt
        #     twist = compute_twist(rbt=rbt)
        #     print('twist between ' + ar_tags['ar0'] + ' and ' + ar_tags['arZ'])
        #     print twist
        # except Exception as e:
        #     print(e)

        rate.sleep()
