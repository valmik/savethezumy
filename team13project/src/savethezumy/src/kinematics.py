#!/usr/bin/env python
""" Kinematics Code
Course: EE 106A, Fall 2016
Written by: Valmik Prabhu, 12/5/16
Used by: EE106A Project, 12/5/16

This code does forward and inverse kinematics for the UR5

Sources:
The exception code was heavily drawn from the python.org tutorial on errors and exceptions:
https://docs.python.org/3/tutorial/errors.html

"""

import numpy as np
import scipy as sp
from numpy import linalg
import math
import copy
import PKproblems as pk 
import kin_func_skeleton as kfs
import exp_quat_func as eqf
from constants import *

class Error(Exception):
    """ Base exception class """
    pass

class SolverError(Error):
    """
    This exception is fired when the IK solver can't solve the problem

    Attributes:
        message - string: explanation of the error
    """

    def __init__(self, angle):
        self.message = "\nThe solver was not able to find a solution for angle " + angle + "\n"

class AngleError(Error):
    """
    This exception is raised if none of the angles the IK solver finds are within acceptable limits
    Attributes:
        message -- explanation of the error
    """

    def __init__(self, angle, values):
        self.message = "\nThe solver was able to find a solution for angle " + angle + " but the value(s) " + str(values) + " did not meet constraints \n"


def deg2rad(deg):
    """
    Converts degrees to radians

    Args:
    deg: angle in degrees

    Returns:
    rad: angle in radians
    """

    rad = float(deg) * math.pi/180

    return rad

def rad2deg(rad):
    """
    Converts degrees to radians

    Args:
    rad: angle in radians

    Returns:
    deg: angle in degrees

    """

    deg = rad * 180/math.pi

    return deg

def rpy(x, y, z):
    """
    Constructs the rotation matrix for roll, pitch, yaw

    Args:
    x: roll
    y: pitch
    z: yaw

    Returns:
    R - (3,3) ndarray: Rotation matrix
    """

    Rx = kfs.rotation_3d(np.array([1, 0, 0]), x)
    Ry = kfs.rotation_3d(np.array([0, 1, 0]), y)
    Rz = kfs.rotation_3d(np.array([0, 0, 1]), z)

    R = np.dot(Rz, np.dot(Ry, Rx))

    return R

def make_xi():
    """
    Constructs the xi array

    Args:

    Returns:
    xi - (6,4) ndarray: an array of joint twists for the first 4 joints
    """

    xi1 = np.concatenate((-np.cross(omega1, q1), omega1))
    xi2 = np.concatenate((-np.cross(omega2, q2), omega2))
    xi3 = np.concatenate((-np.cross(omega3, q3), omega3))
    xi4 = np.concatenate((-np.cross(omega4, q4), omega4))

    xi = np.array(np.vstack([xi1, xi2, xi3, xi4])).T

    return xi

def make_g_0():
    """
    Constructs the g_0 configuration

    Args:

    Returns:
    g_0 - (4, 4) ndarray: the UR5 starting condition
    """

    R = rpy(rx, ry, rz)
    pos = np.array([qe]).T
    g_0 = np.hstack([R, pos])
    g_0 = np.vstack([g_0, np.array([0, 0, 0, 1])])
    
    g_0 = np.round(g_0, 4)

    # For the HW3 Test
    # g_0 = np.array([[1, 0, 0, qe[0]], [0, 1, 0, qe[1]], [0, 0, 1, qe[2]], [0, 0, 0, 1]])

    return g_0

def make_g_d(pos):
    """
    Constructs the g_d configuration, with the rotation the same as it already was

    Args:
    pos - (3,) ndarray: the desired end effector position for the UR5

    Returns:
    g_0 - (4, 4) ndarray: the UR5 starting condition
    """

    x1 = float(pos[0])
    y1 = float(pos[1])
    theta1 = np.arctan2(y1, x1)
    # print rad2deg(theta1)

    x0 = float(qe[0])
    y0 = float(qe[1])
    theta0 = np.arctan2(y0, x0)
    # print rad2deg(theta0)

    theta = theta1 - theta0
    # print rad2deg(theta)

    R = kfs.rotation_3d(np.array([0, 0, 1]), theta)
    R0 = rpy(rx, ry, rz)

    R = np.dot(R0, R)

    pos = np.array([pos]).T
    g_d = np.hstack([R, pos])
    g_d = np.vstack([g_d, np.array([0, 0, 0, 1])])
    g_d = np.round(g_d, 4)

    return g_d

def fk_helper(theta):
    """
    Runs the forward kinematics of the UR5

    Args:
    theta - (4,) ndarray: Desired joint states

    Returns:
    g_d - (4,4) ndarray: output configuration

    """

    xi = make_xi()
    g_0 = make_g_0()
    g_we = kfs.prod_exp(xi, theta)
    g_d = np.dot(g_we, g_0)

    return g_d

def ik_helper(g_d):
    """
    Finds the joint states that set the UR5 to state g_d
        
    Args:
    g_d - (4, 4) ndarray: the desired robot configuration, from the base frame to the tool frame
    

    Returns:
    theta - (4,) ndarray: the necessary angles for the first 4 joints
    """

    ## Define our inital state ***************

    g_0 = make_g_0()
    xi = make_xi()

    ## Solve for theta3 *****************

    g_1 = np.dot(g_d, np.linalg.inv(g_0))

    p = np.array([q4]).T
    q = np.array([q2]).T
    r = np.array([q3]).T
    w = np.array([omega3]).T
    delta = np.linalg.norm(np.dot(g_1, np.append(q4, 1)) - np.append(q2, 1))

    theta3 = pk.pk3(p, q, r, w, delta)

    try:
        if theta3[0] == 0 and np.linalg.norm(p-q) != delta:
            raise SolverError('3')
    except SolverError as e:
        print e.message
        raise

    try:
        if (theta3[0] > deg2rad(theta3min + tolerance)) and (theta3[0] < deg2rad(theta3max - tolerance)):
            theta3 = theta3[0]
        elif (theta3[1] > deg2rad(theta3min + tolerance)) and (theta3[1] < deg2rad(theta3max - tolerance)):
            theta3 = theta3[1]
        else:
            raise AngleError('3', theta3)
    except AngleError as e:
        print e.message
        raise


    ## Solve for theta1 and theta2 ****************
    p = np.array([np.dot(kfs.homog_3d(xi[:, 2], theta3), np.append(q4, 1))[:3]]).T

    q = np.dot(g_1, np.append(q4, 1))[:3]
    q = np.array([q]).T

    r = np.array([q2]).T
    w1 = np.array([omega1]).T
    w2 = np.array([omega2]).T
    theta12 = pk.pk2(p, q, r, w1, w2)

    try:
        if (theta12[0][1] > deg2rad(theta2min + tolerance)) and (theta12[0][1] < deg2rad(theta2max - tolerance)):
            theta12 = theta12[0]
        elif (theta12[1][1] > deg2rad(theta2min + tolerance)) and (theta12[1][1] < deg2rad(theta2max - tolerance)):
            theta12 = theta12[1]
        else:
            raise AngleError('2', theta12[:1])
    except AngleError as e:
        print e.message
        raise

    theta1 = theta12[0]
    theta2 = theta12[1]

    ## Solve for theta4 ***************
    g_2 = np.dot(kfs.homog_3d(-1*xi[:, 2], theta3), np.dot(kfs.homog_3d(-1*xi[:, 1], theta2), np.dot(kfs.homog_3d(-1*xi[:, 0], theta1), g_1)))

    p = np.array([qe]).T
    q = np.array([np.dot(g_2, np.append(qe, 1))[:3]]).T
    r = np.array([q4]).T
    w = np.array([omega4]).T

    theta4 = pk.pk1(p, q, r, w)

    try:
        if theta4 == 0 and np.linalg.norm(p-q) >= 0.001:
            raise SolverError('4')
    except SolverError as e:
        print e.message
        raise


    theta = np.array([theta1, theta2, theta3, theta4])
    return theta

def fk(theta):
    """
    User portal for forward kinematics

    Args:
    theta - (6,) ndarray: Desired joint states. Note: this totally ignores the last 2 angles

    Returns:
    pos - (3,) ndarray output position

    """
    t = copy.deepcopy(theta)
    t[3] = t[3] + theta4offset
    t = t[:4]
    g = fk_helper(t)

    pos = g[:, 3]
    pos = pos[:3]

    return pos

def ik(pos):
    """
    Finds the joint states that set the UR5 to position pos
        
    Args:
    pos - (3,) ndarray: the desired robot position
    

    Returns:
    theta - (6,) ndarray: the necessary angles for the first 6 joints (the last 2 will always be the same)
    """

    g_d = make_g_d(pos)

    theta = ik_helper(g_d)
    theta = np.append(theta, [theta5, theta6])
    theta[3] = theta[3] - theta4offset

    # print theta
    return theta

def move_z(z, theta_0):
    """
    Inverse kinematics for a z move. Requires current joint states, but less inaccurate
        
    Args:
    z: the desired movement in the z axis
    theta_0 - (6,) ndarray: current joint states
    

    Returns:
    theta_1 - (6,) ndarray: the necessary angles for the first 6 joints (the last 2 will always be the same)
    """

    joints = theta_0[:4]
    joints[3] = joints[3] + math.pi/2

    g_d = fk_helper(joints)
    g_d[2, 3] = g_d[2, 3] + z

    theta_1 = ik_helper(g_d)
    theta_1[3] = theta_1[3]-math.pi/2
    theta_1 = np.append(theta_1, [-math.pi/2, 0])

    return theta_1


if __name__ == '__main__':
    # theta = np.array([deg2rad(134.01), deg2rad(-41.11), deg2rad(52.86), deg2rad(-101.49), -90, 0])
    # print fk(theta)
    # print ik(fk(theta))

    # make_g_d(np.array([1, 0, 0]))
    # print fk(theta)
    # print rad2deg(ik(np.array([0.625, -.49, .383])))
    # print fk(ik(fk(theta)))
    # print rad2deg(ik([0.8, 0, 0.2]))
    # print fk(ik(np.array([0.8, 0, 0])))

    ## Zero Case
    # theta = np.array([0, 0, 0, -math.pi/2, -math.pi/2, 0])
    # # print ik(fk(theta))

    # ## Slight Rotation
    # # pos = np.array([-0.8, 0, 0])
    # # print make_g_d(pos)
    # # print ik(pos)
    # # print fk(ik(pos))

    # q_des = np.array([-.7924, -.2267, .0184])
    # print make_g_d(q_des)
    # # print rad2deg(ik(q_des))
    # q_des = np.array([-.7924, -.2267, .0284])
    # print make_g_d(q_des)

    theta = np.array([deg2rad(-180), deg2rad(-40), deg2rad(50), deg2rad(-10)])
    print fk_helper(theta)
    print make_g_d(np.array([-.1092, .80652, .21195]))
    print rad2deg(ik_helper(fk_helper(theta)))
    print rad2deg(ik_helper(make_g_d(np.array([-.1092, .80652, .21195]))))
    print fk_helper(ik_helper(make_g_d(np.array([-.1092, .80652, .21195]))))


