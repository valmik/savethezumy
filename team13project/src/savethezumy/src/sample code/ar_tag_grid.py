#!/usr/bin/env python
import tf
# import pdb
import rospy
# import sys
# import math
import numpy as np
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import Transform, Vector3
# import kin_func_skeleton as kfs
# import exp_quat_func as eqf

listener = None

    # Will pass in one array of numbers, corner tag
    # if len(sys.argv) < 2:
    #     print('Use: ar_tag_subs.py [num1...numend],cornerNum')
    #     sys.exit()
    # ar_tagnums = sys.argv[0]
    # cornerNum = sys.argv[1]

    # cornerNum = 9;
    # ar_tagnums = np.array([10,11,12,13,14,15,16,17,9])
def grid_gen(cornerNum,ar_tagnums):
    rospy.init_node('ar_tags_gridgen')
    # cornerNum = 3;
    # ar_tagnums = np.array([13,14,15,2,3,5,6,7,8])

    listener = tf.TransformListener()
    # Set the board properties
    xspacing = .13
    yspacing = xspacing
    tol = .06
    direction = -1 #right
    # boardLength = 4
    boardLength = np.sqrt(ar_tagnums.size)
    try:
        grid = np.zeros((ar_tagnums.size,1))
        grid[0]=cornerNum
        for i in range(0,ar_tagnums.size): #loop over number of tags
            currentTag = int(grid[i])
            for j in range(0,ar_tagnums.size): #check all tags
                nextTag = int(ar_tagnums[j])
                print('Cur',currentTag,'Next',nextTag)
                a=0
                while a == 0:
                    try:
                        (trans, rot) = listener.lookupTransform('ar_marker_'+str(currentTag),'ar_marker_'+str(nextTag), rospy.Time(0))
                        a=1
                    except Exception as e: 
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
                elif (abs(y)<tol)& (abs(x) < (xspacing+tol)) & (abs(x) > (xspacing-tol)) & (direction*x>0):
                        grid[i+1]=nextTag
    except Exception as e:
        print(e)
    return grid
       