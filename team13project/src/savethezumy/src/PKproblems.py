#!/usr/bin/env python
""" Paden Kahan Subproblem Code
Course: EE 106A, Fall 2016
Written by: Valmik Prabhu, 12/5/16
Used by: EE106A Project, 12/6/16

This code solves the three Paden Kahan subproblems.
It's transcribed from Valmik's HW3 Matlab code

"""

import numpy as np
import scipy as sp
from scipy import linalg
import math

def pk1(p, q, r, omega):
    """
    Solves the first Paden Kahan subproblem.
    Rotate the point p about axis xi until it rests on point q.
    r is a point on xi
    Essentially finds the intersection between a circle and a point
    
    Args:
    p - (3,1) ndarray: the initial position
    q - (3,1) ndarray: the desired position
    r - (3,1) ndarray: a point on the axis of rotation
    omega - (3,1) ndarray: the rotation vector
    
    Returns:
    theta: the angle which will achieve the transformation. Returns 0 if no solution (zero may also be the solution)
    """
    
    if not p.shape == (3,1):
        raise TypeError('p must be a 3x1-vector')

    if not q.shape == (3,1):
        raise TypeError('q must be a 3x1-vector')
        
    if not r.shape == (3,1):
        raise TypeError('r must be a 3x1-vector')

    if not omega.shape == (3,1):
        raise TypeError('omega must be a 3x1-vector')    
    

    try:
        u = p-r
        v = q-r
        u_prime = u - np.dot(np.dot(omega, omega.T), u)
        v_prime = v - np.dot(np.dot(omega, omega.T), v)

        theta = np.arctan2(np.dot(omega.T, np.cross(u_prime.T, v_prime.T).T), np.dot(u_prime.T, v_prime))

        for i in np.nditer(theta, op_flags=['readwrite']):
            if i > math.pi:
                i[...] = i - 2*math.pi
            elif i < -math.pi:
                i[...] = i + 2*math.pi

        theta = theta[0][0]

    except ValueError:
        theta = 0

    return theta

def pk2(p, q, r, omega_1, omega_2):
    """
    Solves the second Paden Kahan subproblem.
    Rotate the point p about axis xi_1, then rotate about xi_2 until it rests on point q.
    r is the intersection of xi_1 and xi_2
    Essentially finds the intersection between a two circles
    
    Args:
    p - (3,1) ndarray: the initial position
    q - (3,1) ndarray: the desired position
    r - (3,1) ndarray: the intersection of the two axes of rotation
    omega_1 - (3,1) ndarray: the first rotation vector
    omega_2 - (3,1) ndarray: the second rotation vector

    
    Returns:
    theta - (2,2): both sets of angles which will achieve the transformation. Or zeros if no solution (zero may also be the solution)
    """
    if not p.shape == (3,1):
        raise TypeError('p must be a 3x1-vector')

    if not q.shape == (3,1):
        raise TypeError('q must be a 3x1-vector')
        
    if not r.shape == (3,1):
        raise TypeError('r must be a 3x1-vector')

    if not omega_1.shape == (3,1):
        raise TypeError('omega_1 must be a 3x1-vector')

    if not omega_2.shape == (3,1):
        raise TypeError('omega_2 must be a 3x1-vector')    
    
    try: 
        u = p-r
        v = q-r
        
        alpha = np.dot(np.dot(np.dot(omega_1.T, omega_2), omega_2.T), u)
        alpha = alpha - np.dot(omega_1.T, v)
        alpha = float(alpha) / ((np.dot(omega_1.T, omega_2) ** 2) - 1)

        beta = np.dot(np.dot(np.dot(omega_1.T, omega_2), omega_1.T), v) 
        beta = beta - np.dot(omega_2.T, u)
        beta = float(beta) / ((np.dot(omega_1.T, omega_2) ** 2) - 1)

        gamma = (np.linalg.norm(u) ** 2)
        gamma = gamma - (alpha ** 2)
        gamma = gamma - (beta ** 2)
        gamma = gamma - 2*np.dot(np.dot(np.dot(alpha, beta), omega_1.T), omega_2) 
        gamma = float(gamma) / (np.linalg.norm(np.cross(omega_1.T, omega_2.T).T) ** 2)
        gamma = gamma ** 0.5

        z1 = alpha * omega_1 + beta * omega_2 + gamma * np.cross(omega_1.T, omega_2.T).T
        z2 = alpha * omega_1 + beta * omega_2 - gamma * np.cross(omega_1.T, omega_2.T).T

        c1 = z1 + r
        # print(c1)
        c2 = z2 + r
        # print(c2)

        theta1 = np.array([pk1(q, c1, r, -omega_1), pk1(p, c1, r, omega_2)])
        theta2 = np.array([pk1(q, c2, r, -omega_1), pk1(p, c2, r, omega_2)])

        theta = np.array([theta1, theta2])

        for i in np.nditer(theta, op_flags=['readwrite']):
            if i > math.pi:
                i[...] = i - 2*math.pi
            elif i < -math.pi:
                i[...] = i + 2*math.pi

    except ValueError:
        theta = np.array([0, 0])
        theta = np.array([theta, theta])

    return theta


# def pk2plus(p, q, r1, r2, omega_1, omega_2):
#     """
#     Solves the modified second Paden Kahan subproblem, which is the same as the second problem but with disjoint axes.
#     Rotate the point p about axis xi_1, then rotate about xi_2 until it rests on point q.
#     Take the common normal of the two axes d.
#     r1 is the intersection of xi_1 and d.
#     r2 is the intersection of xi_2 and d.
#     Kind of the double pendulum problem
    
#     Args:
#     p - (3,1) ndarray: the initial position
#     q - (3,1) ndarray: the desired position
#     r1 - (3,1) ndarray: the intersection of the two axes of rotation
#     omega_1 - (3,1) ndarray: the first rotation vector
#     omega_2 - (3,1) ndarray: the second rotation vector

    
#     Returns:
#     theta - (2,2): both sets of angles which will achieve the transformation. Or zeros if no solution (zero may also be the solution)
#     """
#     if not p.shape == (3,1):
#         raise TypeError('p must be a 3x1-vector')

#     if not q.shape == (3,1):
#         raise TypeError('q must be a 3x1-vector')
        
#     if not r1.shape == (3,1):
#         raise TypeError('r must be a 3x1-vector')

#     if not omega_1.shape == (3,1):
#         raise TypeError('omega_1 must be a 3x1-vector')

#     if not omega_2.shape == (3,1):
#         raise TypeError('omega_2 must be a 3x1-vector')    
    
#     try: 
#         r = r1
#         u = p-r
#         v = q-r
        
#         alpha = np.dot(np.dot(np.dot(omega_1.T, omega_2), omega_2.T), u)
#         alpha = alpha - np.dot(omega_1.T, v)
#         alpha = float(alpha) / ((np.dot(omega_1.T, omega_2) ** 2) - 1)

#         beta = np.dot(np.dot(np.dot(omega_1.T, omega_2), omega_1.T), v) 
#         beta = beta - np.dot(omega_2.T, u)
#         beta = float(beta) / ((np.dot(omega_1.T, omega_2) ** 2) - 1)

#         gamma = (np.linalg.norm(u) ** 2)
#         gamma = gamma - (alpha ** 2)
#         gamma = gamma - (beta ** 2)
#         gamma = gamma - 2*np.dot(np.dot(np.dot(alpha, beta), omega_1.T), omega_2) 
#         gamma = float(gamma) / (np.linalg.norm(np.cross(omega_1.T, omega_2.T).T) ** 2)
#         gamma = gamma ** 0.5

#         z1 = alpha * omega_1 + beta * omega_2 + gamma * np.cross(omega_1.T, omega_2.T).T
#         z2 = alpha * omega_1 + beta * omega_2 - gamma * np.cross(omega_1.T, omega_2.T).T

#         c1 = z1 + r
#         # print(c1)
#         c2 = z2 + r
#         # print(c2)

#         theta1 = np.array([pk1(q, c1, r, -omega_1), pk1(p, c1, r, omega_2)])
#         theta2 = np.array([pk1(q, c2, r, -omega_1), pk1(p, c2, r, omega_2)])

#         theta = np.array([theta1, theta2])

#         for i in np.nditer(theta, op_flags=['readwrite']):
#             if i > math.pi:
#                 i[...] = i - 2*math.pi
#             elif i < -math.pi:
#                 i[...] = i + 2*math.pi

#     except ValueError:
#         theta = np.array([0, 0])
#         theta = np.array([theta, theta])

#     return theta

def pk3(p, q, r, omega, delta):
    """
    Solves the third Paden Kahan subproblem.
    Rotate the point p about axis xi until it rests a distance d away from point q.
    r is a point on xi
    Essentially finds the intersection between a circle and a sphere
    
    Args:
    p - (3,1) ndarray: the initial position
    q - (3,1) ndarray: the position to which p should approach
    r - (3,1) ndarray: a point on the axis of rotation
    omega - (3,1) ndarray: the rotation vector
    delta: distance to between p-desired and q
    
    Returns:
    theta - (2, 1) ndarray: the two angles which will achieve the transformation. Returns 0 if no solution (zero may also be the solution)
    """
    if not p.shape == (3,1):
        raise TypeError('p must be a 3x1-vector')

    if not q.shape == (3,1):
        raise TypeError('q must be a 3x1-vector')
        
    if not r.shape == (3,1):
        raise TypeError('r must be a 3x1-vector')

    if not omega.shape == (3,1):
        raise TypeError('omega must be a 3x1-vector')
    
    try: 
        u = p-r
        v = q-r
        u_prime = u - np.dot(np.dot(omega, omega.T), u)
        v_prime = v - np.dot(np.dot(omega, omega.T), v)

        delta_prime = (delta ** 2) - (np.absolute(np.dot(omega.T, (p-q))) ** 2)
        delta_prime = float(delta_prime) ** 0.5

        theta_0 = np.arctan2(np.dot(omega.T, np.cross(u_prime.T, v_prime.T).T), np.dot(u_prime.T, v_prime))

        theta_0 = theta_0[0][0]

        theta_mod = np.linalg.norm(u_prime) ** 2
        theta_mod = theta_mod + (np.linalg.norm(v_prime) ** 2) - (delta_prime ** 2)
        theta_mod = float(theta_mod) / (2 * np.linalg.norm(u_prime) * np.linalg.norm(v_prime))
        theta_mod = np.arccos(theta_mod)

        theta = np.array([[theta_0 + theta_mod], [theta_0 - theta_mod]])

        for i in np.nditer(theta, op_flags=['readwrite']):
            if i > math.pi:
                i[...] = i - 2*math.pi
            elif i < -math.pi:
                i[...] = i + 2*math.pi


    except ValueError:
        theta = np.array([[0], [0]])
    finally:
        if np.any(np.isnan(theta)):
            theta = np.array([[0], [0]])

    

    return theta


if __name__ == "__main__":

    # p = np.array([[9], [3], [1]])
    # q = np.array([[8], [7], [2]])
    # r = np.array([[3], [4], [5]])
    # omega = np.array([[1], [2], [3]])

    # print(pk1(p, q, r, omega))

    # p2 = np.array([[1], [1], [1]])
    # q = np.array([[8], [7], [2]])
    # r2 = np.array([[1], [1], [1]])
    # omega = np.array([[1], [2], [3]])

    # print(pk1(p2, q, r2, omega))

    # p3 = np.array([[0], [0], [0]])
    # q3 = np.array([[3], [0], [0]])
    # r3 = np.array([[1], [0], [0]])
    # omega3 = np.array([[0], [0], [1]])
    # print(pk3(p, q, r, omega, 1))


    # Homework 3 Tests
    p = np.array([[0], [7], [5]])
    q = np.array([[0], [0], [5]])
    r = np.array([[0], [4], [5]])
    omega = np.array([[-1], [0], [0]])
    delta = 3
    print(pk3(p, q, r, omega, delta))
    print "This should be 2.301, -2.301"

    p = np.array([[0], [2], [2.764]])
    q = np.array([[-2], [2], [6]])
    r = np.array([[0], [0], [5]])
    omega1 = np.array([[0], [0], [1]])
    omega2 = np.array([[-1], [0], [0]])
    print(pk2(p, q, r, omega1, omega2))
    print "This should be [{-2.356, 2.640}, {0.785, -1.181}]"

    p = np.array([[0], [9], [5]])
    q = np.array([[1.414], [7.617], [6.273]])
    r = np.array([[0], [7], [5]])
    omega1 = np.array([[0], [0], [1]])
    omega2 = np.array([[-1], [0], [0]])
    print(pk2(p, q, r, omega1, omega2))
    print "This should be [{1.982, -2.452}, {-1.160, -0.690}]"

    p = np.array([[0], [9], [6]])
    q = np.array([[0.825], [9], [5.565]])
    r = np.array([[0], [7], [5]])
    omega = np.array([[0], [1], [0]])
    print(pk1(p, q, r, omega))
    print "This should be 0.970"


