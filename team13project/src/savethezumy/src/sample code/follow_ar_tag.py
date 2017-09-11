#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf

listener = None

def follow_ar_tag(zumy, ar_tags):
    """
    This function should obtain the rigid body transform between the Zumy and the AR ar_tag
    Then compute and send the twist required to drive the Zumy to the AR ar_tag
    """
    # YOUR CODE HERE
  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag')
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
