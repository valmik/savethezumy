#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf

lkp = {} # dictionary containing the last known position of AR tags

def callback(msg, ar_tags):
    for i in range(0, len(msg.transforms)):

        # YOUR CODE HERE
        # The code should look at the transforms for each AR tag
        # Then compute the rigid body transform between AR0 and AR1, 
        # AR0 and ARZ, AR1 and ARZ
        #  hint: use the functions you wrote in exp_quat_func
        #  note: you can change anything in this function to get it working
        #  note: you may want to save the last known position of the AR tag

        lkp[msg.transforms[i].child_frame_id] = 0 # position / orientation

        print msg.transforms[i]
  
if __name__=='__main__':
    rospy.init_node('ar_tags_subs_manual')
    if len(sys.argv) < 4:
        print('Use: ar_tags_subs_manual.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_tags = {}
    ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    rospy.Subscriber('/tf', TFMessage, callback, ar_tags)
    rospy.spin()
