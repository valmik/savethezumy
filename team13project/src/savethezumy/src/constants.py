#!/usr/bin/env python
""" Constants
Course: EE 106A, Fall 2016
Written by: Valmik Prabhu, 12/5/16
Used by: EE106A Project, 12/5/16

This code contains constants for the UR5:
joint lengths
joint axes
initial joint positions


Sources:
Joint lengths from:
http://www.zacobria.com/temp/ur5_dimensions.jpg

Joint axes and positions from testing, with help from:
"Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms", by Kelsey P. Hawkins, December 7, 2013

"""

import numpy as np
import math

## Angle restrictions and tolerance
# Meant to keep the robot from a singularity and to restrict number of solutions


tolerance = -1 # safety factor in degrees

# Angle restrictions in degrees
theta3min = 0
theta3max = 180
theta2min = -120
theta2max = 0

## UR5 Data ************************************

# scale should be 1000 if in mm instead of m
scale = 1

l_1 = .0892*scale
l_2 = .425*scale
l_3 = .392*scale
l_4 = .1093*scale
l_5 = .0948*scale
l_6 = .0825*scale

omega1 = np.array([0, 0, 1])
q1 = np.array([0, 0, 0])
omega2 = np.array([0, -1, 0])
q2 = np.array([0, 0, l_1])
omega3 = np.array([0, -1, 0])
q3 = np.array([-l_2, 0, l_1])
omega4 = np.array([0, -1, 0])
q4 = np.array([-l_2 - l_3, 0, l_1])

qe = np.array([-l_2 - l_3 - l_5, -l_4, l_1 - l_6])

# Initial angle
rx = math.pi
ry = 0
rz = math.pi/2


theta4offset = math.pi/2
theta5 = -math.pi/2
theta6 = 0


# HW3 Numbers for testing *****************************
# omega1 = np.array([0, 0, 1])
# q1 = np.array([0, 0, 0])
# omega2 = np.array([-1, 0, 0])
# q2 = np.array([0, 0, 5])
# omega3 = np.array([-1, 0, 0])
# q3 = np.array([0, 4, 5])
# omega4 = np.array([0, 0, 1])
# q4 = np.array([0, 7, 5])
# omega5 = np.array([-1, 0, 0])
# q5 = np.array([0, 7, 5])
# omega6 = np.array([0, 1, 0])
# q6 = np.array([0, 7, 5])
# qe = np.array([0, 9, 5])