#!/usr/bin/env python
"""Kinematic function skeleton code for Prelab 3.
Course: EE 106A, Fall 2015
Written by: CHRIS BERTHELET
Used by: EE106A, 9/11/15

This Python file is a code skeleton for Pre lab 3. You should fill in 
the body of the eight empty methods below so that they implement the kinematic 
functions described in the homework assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.

This code requires the NumPy and SciPy libraries. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import numpy as np
import scipy as sp
from scipy import linalg
import math 
import pdb

np.set_printoptions(precision=4,suppress=True)

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

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    rot = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    

    return rot

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

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = np.array([[0,-xi[2],xi[0]],[xi[2],0,xi[1]],[0,0,0]])

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    omega = np.array([xi[3],xi[4],xi[5]])
    omega_hat = skew_3d(omega)
    v_array = np.array([[xi[0]],[xi[1]],[xi[2]]])
    zeros = np.array([[0,0,0,0]])
    
    xi_hat = np.concatenate((omega_hat,v_array),axis=1)
    xi_hat = np.concatenate((xi_hat,zeros),axis=0)

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    rot_input = xi[2]*theta
    R = rotation_2d(rot_input)
    
    pmtx_1 = np.eye(2,dtype=int) - R
    pmtx_2 = np.array([[0,-1],[1,0]])
    pmtx_3 = np.array([[xi[0]/xi[2]],[xi[1]/xi[2]]])
    
    p_temp = pmtx_1.dot(pmtx_2)
    p = p_temp.dot(pmtx_3)
    bottom = np.array([[0,0,1]])
    
    g = np.concatenate((R,p),axis=1)
    g = np.concatenate((g,bottom),axis=0)

    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    '''
    omega = np.array([xi[3],xi[4],xi[5]])
    omega_T = omega.transpose()
    v = np.array([xi[0],xi[2],xi[3]])
    
    g_rotation = rotation_3d(omega,theta)
    
    g_translation = (1/linalg.norm(omega)) * (np.dot((np.eye(3,dtype=int) - g_rotation),(np.dot(skew_3d(omega),v))(np.dot(omega,omega_T,v)*theta)))'''
    
    g = linalg.expm(hat_3d(xi)*theta)

    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    g = np.eye(4)
    
    for i in range(0,theta.size):
        xi_current = np.array([xi[0,i],xi[1,i],xi[2,i],xi[3,i],xi[4,i],xi[5,i]])
        theta_current = theta[i]
        g = np.dot(g, homog_3d(xi_current, theta_current))

    '''p = np.array([0.7957,0.9965,0.3058])
    print(p)
    #g_0 = np.matrix([   [1, 0, 0, p[0]],   [0, 1, 0, p[1]],   [0, 0, 1, p[2]],   [0, 0, 0, 1]       ])
    #print([ 'g_0 is ', g_0])
    #print([   'g is ', g  ])

    newP = np.dot(g, p)'''

    return g

#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def mapping(theta):

    qstart = np.array([0,0,0])
    q1 = np.array([0.0635,0.2598,0.1188])
    q2 = np.array([0.1106,0.3116,0.3885])
    q3 = np.array([0.1827,0.3838,0.3881])
    q4 = np.array([0.3682,0.5684,0.3181])
    q5 = np.array([0.4417,0.6420,0.3177])
    q6 = np.array([0.6332,0.8337,0.3067])
    q7 = np.array([0.7152,0.9158,0.3063])
    qend = np.array([0.7957,0.9965,0.3058])

    omega1 = np.array([-0.0059,0.0113,0.9999])
    omega2 = np.array([-0.7077,0.7065,-0.0122])
    omega3 = np.array([0.7065,0.7077,-0.0038])
    omega4 = np.array([-0.7077,0.7065,-0.0122])
    omega5 = np.array([0.7065,0.7077,-0.0038])
    omega6 = np.array([-0.7077,0.7065,-0.0122])
    omega7 = np.array([0.7065,0.7077,-0.0038])

    v1 = np.cross(-omega1,q1)
    v2 = np.cross(-omega2,q2)
    v3 = np.cross(-omega3,q3)
    v4 = np.cross(-omega4,q4)
    v5 = np.cross(-omega5,q5)
    v6 = np.cross(-omega6,q6)
    v7 = np.cross(-omega7,q7)

    xi1 = np.concatenate([v1,omega1])
    xi2 = np.concatenate([v2,omega2])
    xi3 = np.concatenate([v3,omega3])
    xi4 = np.concatenate([v4,omega4])
    xi5 = np.concatenate([v5,omega5])
    xi6 = np.concatenate([v6,omega6])
    xi7 = np.concatenate([v7,omega7])

    #print(xi1.shape)

    xi1 = np.transpose(xi1)
    xi2 = np.transpose(xi2)
    xi3 = np.transpose(xi3)
    xi4 = np.transpose(xi4)
    xi5 = np.transpose(xi5)
    xi6 = np.transpose(xi6)
    xi7 = np.transpose(xi7)

    #print(xi1.shape)


    #pdb.set_trace()

    #xiF = np.concatenate([xi1,xi2])
    #xiF = np.concatenate([xiF,xi3])
    #xiF = np.concatenate([xiF,xi4])
    #xiF = np.concatenate([xiF,xi5])
    #xiF = np.concatenate([xiF,xi6])
    #xiF = np.concatenate([xiF,xi7])
    
    
    xiF = np.concatenate([[xi1], [xi2], [xi3], [xi4], [xi5], [xi6], [xi7]])
    xiF = np.transpose(xiF)

    #print(xiF.shape)
   # print(xiF)
    #pdb.set_trace()

    #xiF = np.reshape(xiF, (6,7))

    #xi_hat1 = kin_func_skeleton.hat_3d(xi1)
    #xi_hat2 = kin_func_skeleton.hat_3d(xi2)
    #xi_hat3 = kin_func_skeleton.hat_3d(xi3)
    #xi_hat4 = kin_func_skeleton.hat_3d(xi4)
    #xi_hat5 = kin_func_skeleton.hat_3d(xi5)
    #xi_hat6 = kin_func_skeleton.hat_3d(xi6)
    #xi_hat7 = kin_func_skeleton.hat_3d(xi7)

    #theta = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

    g_st = prod_exp(xiF, theta)

    '''   p = np.array([0.7957,0.9965,0.3058])

    g_0 = np.matrix([   [1, 0, 0, p[0]],   [0, 1, 0, p[1]],   [0, 0, 1, p[2]],   [0, 0, 0, 1]       ])
    print([ 'g_0 is ', g_0])
    print([ 'g_st is ', g_st  ])
    g_st1 = np.dot(g_st, g_0)'''

    return g_st


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

if __name__ == "__main__":
    print('Testing...')


    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3.,  0.,  1.],
                            [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                            [ 0.9198, -0.3924,  1.2348],
                            [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    print('Done!')
