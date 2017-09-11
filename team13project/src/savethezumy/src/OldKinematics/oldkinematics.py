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
        self.message = "The solver was not able to find a solution for angle " + angle

class AngleError(Error):
    """
    This exception is raised if none of the angles the IK solver finds are within acceptable limits
    Attributes:
        message -- explanation of the error
    """

    def __init__(self, angle, values):
        self.message = "The solver was able to find a solution for angle " + angle + " but the value(s) " + str(values) + " did not meet constraints"


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

def make_xi():
    """
    Constructs the xi array

    Args:

    Returns:
    xi - (6,6) ndarray: an array of joint twists for the 6 joints (though the sixth is bs)
    """

    xi1 = np.concatenate((-np.cross(omega1, q1), omega1))
    xi2 = np.concatenate((-np.cross(omega2, q2), omega2))
    xi3 = np.concatenate((-np.cross(omega3, q3), omega3))
    xi4 = np.concatenate((-np.cross(omega4, q4), omega4))
    xi5 = np.concatenate((-np.cross(omega5, q5), omega5))
    xi6 = np.concatenate((-np.cross(omega6, q6), omega6))

    xi = np.array(np.vstack([xi1, xi2, xi3, xi4, xi5, xi6])).T

    return xi

def make_g_0():
    """
    Constructs the g_0 configuration

    Args:

    Returns:
    g_0 - (4, 4) ndarray: the UR5 starting condition
    """

    # For the UR5, since the tool frame starts rotated in relation to the body frame
    g_0 = np.array([[1, 0, 0, qe[0]], [0, 0, -1, qe[1]], [0, 1, 0, qe[2]], [0, 0, 0, 1]])
    
    # For the HW3 Test
    # g_0 = np.array([[1, 0, 0, qe[0]], [0, 1, 0, qe[1]], [0, 0, 1, qe[2]], [0, 0, 0, 1]])

    return g_0

def make_g_d(pos, omega, theta):
    """
    Constructs the g_d configuration, with the rotation the same as it already was

    Args:
    pos - (3,) ndarray: the desired end effector UR5
    omega - (3,) ndarray: the rotation axis
    theta: angle

    Returns:
    g_0 - (4, 4) ndarray: the UR5 starting condition
    """

    # g_d = np.array([[1, 0, 0, pos[0]], [0, 0, -1, pos[1]], [0, 1, 0, pos[2]], [0, 0, 0, 1]])

    g_d = eqf.create_rbt(omega, theta, pos)

    return g_d

def make_g_d0(pos):
    """
    Constructs the g_d configuration, with the rotation the same as it already was

    Args:
    pos - (3,) ndarray: the desired end effector UR5
    omega - (3,) ndarray: the rotation axis
    theta: angle

    Returns:
    g_0 - (4, 4) ndarray: the UR5 starting condition
    """

    g_d = np.array([[1, 0, 0, pos[0]], [0, 0, -1, pos[1]], [0, 1, 0, pos[2]], [0, 0, 0, 1]])

    # g_d = eqf.create_rbt(omega, theta, pos)

    return g_d

def fk(theta):
    """
    Tests the forward kinematics of the UR5

    Args:
    theta - (5,) ndarray: Desired joint states

    Returns:
    g_d - output position

    """

    xi = make_xi()
    g_0 = make_g_0()
    g_we = kfs.prod_exp(xi, theta)
    g_d = np.dot(g_we, g_0)

    return g_d



def ik(g_d):
    """
    Finds the joint states that set the UR5 to state g_d
        
    Args:
    g_d - (4, 4) ndarray: the desired robot configuration, from the base frame to the tool frame
    

    Returns:
    theta - (6,) ndarray: the necessary angles for each joint
    """

    ## Define our inital state ***************

    g_0 = make_g_0()
    xi = make_xi()

    ## Solve for theta3 *****************

    g_1 = np.dot(g_d, np.linalg.inv(g_0))
    p = np.array([q5]).T
    q = np.array([q2]).T
    r = np.array([q3]).T
    w = np.array([omega3]).T
    delta = np.linalg.norm(np.dot(g_1, np.append(q5, 1)) - np.append(q2, 1))

    theta3 = pk.pk3(p, q, r, w, delta)

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
    p = np.array([np.dot(kfs.homog_3d(xi[:, 2], theta3), np.append(q5, 1))[:3]]).T

    q = np.dot(g_1, np.append(q5, 1))[:3]
    q = np.array([q]).T

    print q

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

    # theta6 = -theta1


    ## Solve for theta4 ***************
    g_2 = np.dot(kfs.homog_3d(-1*xi[:, 2], theta3), np.dot(kfs.homog_3d(-1*xi[:, 1], theta2), np.dot(kfs.homog_3d(-1*xi[:, 0], theta1), g_1)))
    # g_2 = np.dot(g_2, kfs.homog_3d(-1*xi[:, 5], theta6))

    p = np.array([q6]).T
    q = np.array([np.dot(g_2, np.append(q6, 1))[:3]]).T
    r = np.array([q5]).T
    w = np.array([omega4]).T

    theta4 = pk.pk1(p, q, r, w)



    ## Solve for theta5 ***************
    g_3 = np.dot(kfs.homog_3d(-1*xi[:, 3], theta4), g_2)

    p = np.array([qe]).T
    q = np.array([np.dot(g_3, np.append(qe, 1))[:3]]).T
    r = np.array([q5]).T
    w = np.array([omega5]).T

    theta5 = pk.pk1(p, q, r, w)


    ## Solve for theta6 ***************
    theta6 = 0

    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    return theta

if __name__ == '__main__':
    theta = np.array([deg2rad(134.01), deg2rad(-41.11), deg2rad(52.86), deg2rad(-101.49), deg2rad(-104.89), 0])
    # theta = np.array([deg2rad(134), deg2rad(-41), deg2rad(52), deg2rad(-101), deg2rad(-104), 0])
    # g_d = make_g_0()
    # # print ik(g_d)
    print fk(theta)
    print rad2deg(ik(fk(theta)))