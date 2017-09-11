#!/usr/bin/env python
""" Output to Zumy Code
Course: EE 106A, Fall 2016
Written by: Chris Berthelet, 12/5/16
Used by: EE106A Project, 12/5/16

This code takes the grid array of 1's and 0's and performs a Depth First Search type algoritm to find the optimal path
"""

import sys
import math
import numpy as np
import time 
# import scipy as sp

def create_path_DFS(gridArray):
    """
    Finds the optimal path for Zumy to travel using a Depth First Search type algorithm
        
    Args:
    gridArray - (4, 4) ndarray: results of grid testing in the form of 0's (foam) and 1's (wood)
    

    Returns:
    start_pos - int value between 1-4: the position for the Zumy to start at 
    correspond_pos_array - (n, 2) ndarray: array indexes of n number of ar tags that the Zumy must follow

    """

    gridMtx = np.zeros(16)
    newGridInd = [0,7,8,15,1,6,9,14,2,5,10,13,3,4,11,12]
    for i in range(0,len(gridArray)):
            gridMtx[i]=int(gridArray[newGridInd[15-i]])

    gridMtx = np.reshape(gridMtx,(4,4))
    
    gridMtx_pad = np.lib.pad(gridMtx,1,'constant')

    gridMtx_pad_temp = gridMtx_pad

    # raw_input()

    index_array = np.array([ [15, 14, 13, 12],
        [8, 9, 10, 11],
        [7, 6, 5, 4],
        [0, 1, 2, 3]])
    index_array = np.flipud(index_array.T)
    
    
    input_array = [np.array([[1,0]]), np.array([[0,1]]), np.array([[0,-1]])]
    paths_array = [np.array([[0,0]]), np.array([[0,0]]), np.array([[0,0]]), np.array([[0,0]])]
    found = 0

    try:

        for i in range(0,gridMtx.shape[1]):

            gridMtx_pad = gridMtx_pad_temp

            ind1 = 0
            ind2 = i

            if gridMtx_pad[ind1+1][ind2+1] == 0:

                paths_array[i] = np.array([[1000]])

            else:

                paths_array[i] = np.array([[ind1,ind2]])
                count = 1
                while ind1 < 3:
                    count = count+1
                    # If we get stuck this is an infeasible solution
                    if count > 100000:
                        paths_array[i] = np.array([[1000]])
                        break
                    for ii in range(0,3):

                        ind1 = ind1 + input_array[ii][0][0]
                        ind2 = ind2 + input_array[ii][0][1]

                        if gridMtx_pad[ind1+1][ind2+1] == 1:

                            a = paths_array[i]
                            b = np.array([[ind1,ind2]])
                            paths_array[i] = np.concatenate((a,b),axis=0)
                            found = 1
                            gridMtx_pad[ind1+1][ind2+1] = 0
                            break

                        else:

                            ind1 = ind1 - input_array[ii][0][0]
                            ind2 = ind2 - input_array[ii][0][1]


                    if found != 1:
                        paths_array[i] = np.array([[1000]])
                        break

        #NOW find the Shortest Path
        cost_best = 11 
        start_pos = 0

        for j in range(0,len(paths_array)):

            cost_temp = paths_array[j].shape[0]


            if paths_array[j][0][0]==1000:
                random = float('inf')

            elif cost_temp < cost_best:
                cost_best = cost_temp
                start_pos = j

        #print start_pos

        #Create array for Zumy
        array_of_indexes = paths_array[start_pos]

        correspond_pos_array = np.zeros(array_of_indexes.shape[0])

        for k in range(0,array_of_indexes.shape[0]):

            correspond_pos_array[k] = int(index_array[array_of_indexes[k][0]][array_of_indexes[k][1]])

        #print correspond_pos_array
        start_pos=start_pos+1
        return (start_pos,correspond_pos_array)

    except:
        # print("NO FEASIBLE PATH! Zumy is DOOOOOOOOOOOMED!")
        start_pos=100
        correspond_pos_array = [100,100]
    return start_pos,correspond_pos_array



if __name__ == '__main__': 
    #snake
    # gridArray = np.array([1,1,1,0,0,1,0,0,0,0,1,0,1,1,0,0])
    
    #2 possible
    # gridArray = np.array([1,1,1,0,0,1,0,0,0,0,1,0,1,1,1,1])


    #3 diff start
    # gridArray = np.array([1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1])

    # infeasible
    # gridArray = np.array([1,0,1,1,0,1,1,0,1,0,0,0,0,1,1,0])

    gridArray = np.load('gridWoodBlocks.npy')
    np.save('zumyDead',False)
    [startPos,pos] = create_path_DFS(gridArray)
    print pos
    np.save('zumyPath',pos)
    # np.save('zumyPathDiffStart',pos)
    if startPos == 100:
        np.save('zumyDead',True)
    print 'The Zumy should start at position ',startPos 
    
    