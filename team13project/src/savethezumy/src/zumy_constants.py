#!/usr/bin/env python
""" Zumy Constants
Course: EE 106A, Fall 2016
Written by: Mimi Parker and Valmik Prabhu, 12/11/16
Used by: EE106A Project, 12/11/16

Constants for the zumy

Sources:

"""

import numpy as np
import math

zumytag = 'ar_marker_' + '29'
zumy = 'zumy3'
position_tolerance = 0.02
angle_tolerance = math.pi/180*5

ar_tag_pos = np.array([[ 0.077, 0.149, 1.55],
                       [ -0.050, 0.155, 1.54],
                       [ -0.046, 0.002, 1.46],
                       [-0.180, 0.035, 1.55],
                       [-0.315, 0.040, 1.58]])

tags2cam = np.array([6, 5, 23, 25, 14])


	                   