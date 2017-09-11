#!/usr/bin/env python
"""Exponential and Quaternion code for Lab 6.
Course: EE 106, Fall 2016
Author: Victor Shia, 9/24/15

This Python file is a code skeleton for Lab 6 which calculates the rigid body transform
given a rotation / translation.

When you think you have the methods implemented correctly, you can test your 
code by running "python exp_quat_func.py at the command line.

This code requires the NumPy and SciPy libraries and kin_func_skeleton which you 
should have written in lab 3.
"""

import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
from scipy import linalg
import pdb
#pdb.set_trace()

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    omega_hat = np.array([[0,-omega[2],omega[1]],[omega[2],0,-omega[0]],[-omega[1],omega[0],0]])

    return omega_hat

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    omega_hat = skew_3d(omega)
    omega_hat2 = np.dot(omega_hat, omega_hat)
    omega_norm = linalg.norm(omega)
    omega_norm2 = np.square(omega_norm)
    
    rot = np.eye(3) + omega_hat/omega_norm*np.sin(omega_norm*theta) + omega_hat2/omega_norm2*(1 - np.cos(omega_norm*theta))

    return rot

def quaternion_to_exp(rot):
    """
    Converts a quaternion vector in 3D to its corresponding omega and theta.
    This uses the quaternion -> exponential coordinate equation given in Lab 6
    
    Args:
    rot - a (4,) nd array or 4x1 array: the quaternion vector (\vec{q}, q_o)
    
    Returns:
    omega - (3,) ndarray: the rotation vector
    theta - a scalar
    """

    '''arg1 = np.array([1.0, 2, 3, 0.1])
    func_args = (arg1,)
    ret_desired = (np.array([1.005, 2.0101, 3.0151]), 2.94125)
    array_func_test_two_outputs(quaternion_to_exp, func_args, ret_desired)'''
    
    theta = 2*np.arccos(rot[3])

    if theta == 0:
    	omega = np.array([0, 0, 0])
    else:
        coeff = 1/np.sin(theta/2)
        omega = np.array([rot[0], rot[1], rot[2]])
        omega = coeff*omega
        
    #pdb.set_trace()

    return (omega, theta)
    
def create_rbt(omega, theta, trans):
    """
    Creates a rigid body transform using omega, theta, and the translation component.
    g = [R,p; 0,1], where R = exp(omega * theta), p = trans
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray or 3x1 array: the translation component of the rigid body motion
    
    Returns:
    g - (4,4) ndarray : the rigid body transform
    """

    '''#Test create_rbt()
    arg1 = np.array([1.0, 2, 3])
    arg2 = 2
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(create_rbt, func_args, ret_desired)'''
    
    R = rotation_3d(omega, theta)
    trans = np.transpose(trans)
    bottom = np.array([[0,0,0,1]])
    
    g = np.matrix([ [R[0,0], R[0,1], R[0,2], trans[0] ],
    				[R[1,0], R[1,1], R[1,2], trans[1] ],
    				[R[2,0], R[2,1], R[2,2], trans[2] ] ])
    g = np.concatenate((g,bottom),axis=0)

    #pdb.set_trace()

    return g
    
def compute_gab(g0a, g0b):
    """
    Creates a rigid body transform g_{ab} that converts between frame A and B
    given the coordinate frames A,B in relation to the origin
    
    Args:
    g0a - (4,4) ndarray : the rigid body transform from the origin to frame A
    g0b - (4,4) ndarray : the rigid body transform from the origin to frame B
    
    Returns:
    gab - (4,4) ndarray : the rigid body transform
    """
    # pdb.set_trace()

    gab = np.dot(linalg.inv(g0a), g0b)

    return gab
    
def find_omega_theta(R):
    """
    Given a rotation matrix R, finds the omega and theta such that R = exp(omega * theta)
    
    Args:
    R - (3,3) ndarray : the rotational component of the rigid body transform
    
    Returns:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    """

    theta = np.arccos((np.trace(R) - 1) / 2)

    r_vector1 = R[2,1] - R[1,2]
    r_vector2 = R[0,2] - R[2,0]
    r_vector3 = R[1,0] - R[0,1]

    r_vector = np.array([r_vector1, r_vector2, r_vector3])

    omega = (1/(2*np.sin(theta))) * r_vector

    return (omega, theta)
    
def find_v(omega, theta, trans):
    """
    Finds the linear velocity term of the twist (v,omega) given omega, theta and translation
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray of 3x1 list : the translation component of the rigid body transform
    
    Returns:
    v - (3,1) ndarray : the linear velocity term of the twist (v,omega)
    """

    '''#Test find_v
    arg1 = np.array([1.0, 2, 3])
    arg2 = 1
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array([[-0.1255],
       [ 0.0431],
       [ 0.0726]])
    array_func_test(find_v, func_args, ret_desired)'''

    p = trans
    p_normp = p / linalg.norm(p)

    R = rotation_3d(omega, theta)

    sumTest = sum(sum(R))						# if 3, true
    invTest = sum(sum ( linalg.inv(R) == R ) )	# if 9, true

    if 3 == sumTest:
    	if 9 == invTest:
    		flag = True
    	else:
    		flag = False
    else:
    	flag = False

    if flag == True:

    	top = np.concatenate((np.eye(3),p_normp),axis=1)
    	bottom = np.array([0,0,0,0])

     	xi_hat = np.concatenate((top,bottom),axis=0)

     	v = p_normp

    else: 	
    	Atemp = np.dot( (np.eye(3)-rotation_3d(omega, theta)), skew_3d(omega) )
        A = Atemp + np.dot(    np.outer(omega, np.transpose(omega)    ), theta)
    	# A = Atemp + np.dot(np.dot(omega, np.transpose(omega)), theta)
    	Ainv = linalg.inv(A)
    	vTemp = np.dot(Ainv, p)
    
        v = np.zeros((3,1))
        v[0] = vTemp[0]
        v[1] = vTemp[1]
        v[2] = vTemp[2]

    #pdb.set_trace()

    return v
    
#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')
        
def array_func_test_two_outputs(func_name, args, ret_desireds):
    ret_values = func_name(*args)
    for i in range(2):
        ret_value = ret_values[i]
        ret_desired = ret_desireds[i]
        if i == 0 and not isinstance(ret_value, np.ndarray):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
        elif i == 1 and not isinstance(ret_value, float):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a float')
        elif i == 0 and ret_value.shape != ret_desired.shape:
            print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
        elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
            print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
        else:
            print('[PASS] ' + func_name.__name__ + '() returned the argument %d value!' % i)

if __name__ == "__main__":
    print('Testing...')
    
    #Test quaternion_to_exp()
    arg1 = np.array([1.0, 2, 3, 0.1])
    func_args = (arg1,)
    ret_desired = (np.array([1.005, 2.0101, 3.0151]), 2.94125)
    array_func_test_two_outputs(quaternion_to_exp, func_args, ret_desired)
    
    #Test create_rbt()
    arg1 = np.array([1.0, 2, 3])
    arg2 = 2
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(create_rbt, func_args, ret_desired)
    
    #Test compute_gab(g0a,g0b)
    g0a = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    g0b = np.array(
      [[-0.6949,  0.7135,  0.0893,  0.5   ],
       [-0.192 , -0.3038,  0.9332, -0.5   ],
       [ 0.693 ,  0.6313,  0.3481,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    func_args = (g0a, g0b)
    ret_desired = np.array([[-0.6949, -0.192 ,  0.693 ,  0.    ],
       [ 0.7135, -0.3038,  0.6313,  0.    ],
       [ 0.0893,  0.9332,  0.3481,  0.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(compute_gab, func_args, ret_desired)
    
    #Test find_omega_theta
    R = np.array(
      [[ 0.4078, -0.6562,  0.6349 ],
       [ 0.8384,  0.5445,  0.0242 ],
       [-0.3616,  0.5224,  0.7722 ]])
    func_args = (R,)
    ret_desired = (np.array([ 0.2673,  0.5346,  0.8018]), 1.2001156089449496)
    array_func_test_two_outputs(find_omega_theta, func_args, ret_desired)
    
    #Test find_v
    arg1 = np.array([1.0, 2, 3])
    arg2 = 1
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array([[-0.1255],
       [ 0.0431],
       [ 0.0726]])
    array_func_test(find_v, func_args, ret_desired)
