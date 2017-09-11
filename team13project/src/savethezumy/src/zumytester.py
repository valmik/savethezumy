#!/usr/bin/env python
""" Zumy Testing Code
Course: EE 106A, Fall 2016
Written by: Mimi Parker, 12/11/16
Skeleton by: Valmik Prabhu, 12/11/16
Used by: EE106A Project, 12/11/16

Used to test zumy.py

Sources:


"""

import numpy as np
import scipy as sp
from numpy import linalg
import rospy
import tf
import pdb
import zumy
import zumy_constants as zumyc

def test_transform():
	"""
	Tests zumy.transform_z
	"""

	p_cz = np.array([0.041, -0.03, 0.52])
	r_cz = np.array([0.62, 0.78, -0.05, 0.007])
	p_cg = np.array([0.084, 0.046, 0.52])
	print np.linalg.norm(zumy.transform_z(p_cg, p_cz, r_cz))

def test_transform2():
	"""
	tests the transform again
	"""

	rospy.init_node('follow_ar_tag_twist')
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
        ### Find zumy position and rotation relative to the camera frame
        ### You could listen to ar_tag_alvar or do a lookup transform on tf
		try:
			(p_cz, r_cz) = listener.lookupTransform('usb_cam', zumyc.zumytag, rospy.Time(0))
			(p_cg, r_cg) = listener.lookupTransform('usb_cam', 'ar_marker_27', rospy.Time(0))
			(p_zg, r_zg) = listener.lookupTransform(zumyc.zumytag, 'ar_marker_27', rospy.Time(0))
		except Exception as e:
			print e
			continue
		else:
			pass
		p_cz = np.array(p_cz)
		r_cz = np.array(r_cz)
		p_cg = np.array(p_cg)

		print "transform: ", zumy.transform_z(p_cg, p_cz, r_cz)

		p_zg = np.array(p_zg)
		print "tf: ", p_zg



def test_move():
	"""
	Tests zumy.transform_z
	"""

	rospy.init_node('test_move')
	listener = tf.TransformListener()

	# ## Undo this for one goal tag
	# pos = zumy.read_position(zumyc.ar_tag_pos)
	# zumy.move(pos)

	## Undo this for multiple goal tags
	count = 1
	for tag in zumyc.ar_tag_pos:
		# pdb.set_trace()
		pos = zumy.read_position(count-1, tag)
		print 'Going to: ', count
		zumy.move(pos, count)
		count = count + 1

	print 'We made it!'
	zumy.move_final()

	# while not rospy.is_shutdown():
        ### Find zumy position and rotation relative to the camera frame
        ### You could listen to ar_tag_alvar or do a lookup transform on tf
		# try:
		# 	# (p_cg1, r_cg1) = listener.lookupTransform('usb_cam', 'ar_marker_27', rospy.Time(0))
		# 	# (p_cg2, r_cg2) = listener.lookupTransform('usb_cam', 'ar_marker_13', rospy.Time(0))
		# 	# (p_cg3, r_cg3) = listener.lookupTransform('usb_cam', 'ar_marker_22', rospy.Time(0))
		# 	# (p_cg4, r_cg4) = listener.lookupTransform('usb_cam', 'ar_marker_20', rospy.Time(0))
		# 	# (p_cg5, r_cg5) = listener.lookupTransform('usb_cam', 'ar_marker_6', rospy.Time(0))

		# except Exception as e:
		# 	# print e
		# 	continue
		# else:
		# 	break

		### READ the tag's position
		

	# p_cg1 = np.array(p_cg1)
	# p_cg2 = np.array(p_cg2)
	# p_cg3 = np.array(p_cg3)
	# p_cg4 = np.array(p_cg4)
	# p_cg5 = np.array(p_cg5)

	# print 'Moving to p_cg1'
	# zumy.move(p_cg1)
	# zumy.move(p_cg2)
	# zumy.move(p_cg3)
	# zumy.move(p_cg4)
	# zumy.move(p_cg5)

if __name__ == "__main__": test_move()




