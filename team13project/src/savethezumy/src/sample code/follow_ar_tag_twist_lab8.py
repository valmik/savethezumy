#!/usr/bin/env python
import tf
import pdb
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
# import exp_quat_func as eqf
# import ar_tag_subs as ats
import time
from kalman_zumy.srv import ImuSrv,ImuSrvResponse,NuSrv,NuSrvResponse

listener = None

def follow_ar_tag(zumy, ar_tags):

    rospy.wait_for_service('innovation')

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
    #zumy_vel = rospy.Publisher('%s/zumy1b/cmd_vel' % zumy, Twist, queue_size=2)
    rate = rospy.Rate(10)
    print ar_tags
    trans = np.array([0, 0, 0])
    rot = np.array([0, 0, 0, 0])
    # prev_trans = np.array([0, 0, 0])
    # prev_rot = np.array([0, 0, 0, 0])

    
    while not rospy.is_shutdown():

        prev_trans = trans
        prev_rot = rot
        
        try:
            # (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
            (trans, rot) = listener.lookupTransform('usb_cam', ar_tags['arZ'], rospy.Time(0))
            #print('IM HERE!')
        except:
            continue

        #pdb.set_trace()
        
        # if np.any(prev_trans, trans) or np.any(prev_rot, rot):
        if np.any(prev_trans != trans) or np.any(prev_rot != rot):
            # zumy_name = sys.argv[1]
            # ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]   [ AR tag number for goal]
            # ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]   [ AR tag number for Zumy]
            msg = Transform()
            msg.translation.x = trans[0]
            msg.translation.y = trans[1]
            msg.translation.z = trans[2]
            msg.rotation.x = rot[0]
            msg.rotation.y = rot[1]
            msg.rotation.z = rot[2]
            msg.rotation.w = rot[3]
            update = rospy.ServiceProxy('innovation', NuSrv)
            update(msg, 'usb_cam')

        # prev_trans = trans
        # prev_rot = rot

        
        # YOUR CODE HERE
        #  The code should compute the twist given 
        #  the translation and rotation between arZ and ar1
        #  Then send it publish it to the zumy

         #compute twist

        # rot = np.array(rot)
        # rbt = ats.return_rbt(trans,rot)
        # twist = ats.compute_twist(rbt)
        # msg = Twist()
        # zero = np.array([0,0,0,0,0,0])
        # tolerance = 0.05
        # scaleFactor = 30

        #Stops the zumy if most twist parameters are close to 0
        # if sum(abs(twist-zero)<=tolerance) >= 4:
        #     #Assign Values to message
        #     msg.linear.x = 0
        #     msg.linear.y = 0
        #     msg.linear.z = 0
        #     msg.angular.x = 0
        #     msg.angular.y = 0
        #     msg.angular.z = 0

        #     zumy_vel.publish(msg)

        # else:
        #     #Assign Values to message
        #     msg.linear.x = twist[0]/scaleFactor
        #     msg.linear.y = twist[1]/scaleFactor
        #     msg.linear.z = twist[2]/scaleFactor
        #     msg.angular.x = twist[3]
        #     msg.angular.y = twist[4]
        #     msg.angular.z = twist[5]

        #     zumy_vel.publish(msg)

        #When the zumy is close enough, STOP it

        #zumy_vel.publish(msg)

        #start = time.time()

        #while time.time()-start < 10:
            #Publish created msg
            #zumy_vel.publish(msg)
            #print(msg)
        
  
if __name__=='__main__':
    rospy.init_node('follow_ar_tag_twist')
    if len(sys.argv) < 3:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[2]
    #ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    rospy.spin()
