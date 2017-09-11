#!/usr/bin/env python
""" Output_To_Zumy Code

Course: EE 106A, Fall 2016
Written by: Eric Noordam, 12/5/16
Used by: EE106A Project, 12/5/16

TASKS:
1) Test the gui and run the zumy code to create output for GUI

FUNCTIONS:
    main()

VARIABLES:

Sources:

"""
import numpy as np
import time
import random
import subprocess

def main():

    gridStart = np.ones(16)*2
    np.save('gridWoodBlocks',gridStart)
    # proc = subprocess.Popen(['python','plotGUI.py'])
    zumyPath = np.array([99]) #np.array([0,1,2,3])
    np.save('zumyPath',zumyPath)
    np.save('zumyDead',False)
    # time.sleep(1)

    #All unknown
    # gridArray = np.ones(16)*2

    #snake
    # gridArray = np.array([1,1,1,0,0,1,0,0,0,0,1,0,1,1,0,0])
    
    #2 possible
    # gridArray = np.array([1,1,1,0,0,1,0,0,0,0,1,0,1,1,1,1])


    #3 diff start
    gridArray = np.array([1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1])

    #3 diff start no zumy
    gridArray = np.array([0,0,1,1,0,1,1,1,0,0,0,0,0,1,1,1])

    # infeasible
    # gridArray = np.array([1,0,1,1,0,1,1,0,1,0,0,0,0,1,1,0])

    proc = subprocess.Popen(['python','output_to_zumy.py'])

    # grid2 = np.array([0,1,1,2,2,0,1,1,0])
    # grid3 = np.array([0,1,0,1,2,2,1,2,1])
    while True:
        try:
            #for i in range(0,newgrid.size):
            #    newgrid[i]=1#int(random.random()+0.5)
            # zumyPath[2] = 4*int(random.random()+.5)
            # print(zumyPath[2])
            # np.save('zumyPath',zumyPath)
            np.save('gridWoodBlocks',gridArray)
            time.sleep(1)
        except Exception as e:
            raise

print('Starting')
main()
