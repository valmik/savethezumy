#!/usr/bin/env python
""" plotGUI Code

Course: EE 206A, Fall 2016
Written by: Eric Noordam, 12/10/16
Used by: EE206A Project, 12/10/16

TASKS:
1) Plot the squares with tag markings
2) Show their material
3) Show the Zumy's path or dead status

FUNCTIONS:

VARIABLES:
flipGUI is used to flip the GUI for sitting on other side of the grid

Sources:

"""
import pygame
import sys
import numpy as np
import time

def main():
    # Set this depending on which side of the grid you're facing
    flipGUI=True
    #Used to reorder all the points for which side of the grid you're on
    if flipGUI: 
        offset = 15
    else:
        offset = 0

    # Init pygame and the window
    pygame.init()
    clock = pygame.time.Clock()
    screenSize=1000
    screen = pygame.display.set_mode((screenSize,screenSize))
    pygame.display.set_caption('SaveTheZumy')

    # Define colors
    black = (0,0,0)
    brown = (100,70,40)
    offwhite = (230,230,230)
    unknown = (75,125,255)
    green = (0,255,0)
    red = (255,0,0)

    #Thickness of red/green path lines
    zumyLine = 10

    # Load tag numbers
    gridArrayIDs = np.load('gridArrayIDs.npy')

    # Load or init grid
    try:
    	grid = np.load('gridWoodBlocks.npy')
    except:
    	grid = np.ones(16)*2

    # init Zumy path to infeasible indices and death to False
    zumyPath = np.array([99999,9999])
    zumyDead = False

    # Grid indices have to be changed, init new grid
    newGridInd = [0,7,8,15,1,6,9,14,2,5,10,13,3,4,11,12]
    newGrid = np.zeros(16)

    # Define the geometry of boxes
    gridSize = np.sqrt(grid.size)
    const=screenSize/gridSize
    fontSize = int(const/2.25)

    # Font for questions and numbers
    quesfont = pygame.font.SysFont("monospace", int(fontSize))
    numfont = pygame.font.SysFont("monospace", int(fontSize/1.3))

    
    # Loop forever to keep updating the data
    while True:
        # Check for quit event  
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                 pygame.quit(); sys.exit();

        #Import and data
        try:
            grid = np.load('gridWoodBlocks.npy')
        except:
            pass

        # Reorder material data
        for i in range(0,len(grid)):
            newGrid[i]=grid[newGridInd[offset-i]]

        try:
            zumyPath = np.load('zumyPath.npy')
            zumyDead = np.load('zumyDead.npy')
        except:
            pass

        # Figure loop through the grid to make the figures
        for i in range(0,len(grid)):

            # Check if the current tag is in the zumyPath
            isInPath = False
            for j in range(0,len(zumyPath)):
                check = int(zumyPath[j])
                if check ==  newGridInd[offset-i]:
                    isInPath = True
            # Find x and y coordinates
            x=(i%gridSize)*const
            y=np.floor((i)/gridSize)*const
            width=1*const
            height=1*const
            
            # By default don't draw question marks
            drawQuestion=False

            # If wood, set to brown, foam off white, and unknown to blue
            if newGrid[i]==1:
                color = brown
            elif newGrid[i]==0:
                color = offwhite
            else:
                color = unknown
                drawQuestion = True

            # Draw the squares with the big green border if on Zumy's path
            if isInPath:
                pygame.draw.rect(screen, green, (x,y,width,height),0)
                pygame.draw.rect(screen, color, (x+zumyLine,y+zumyLine,width-2*zumyLine,height-2*zumyLine),0)
            # Draw the squares with the big red border if Zumy is dead
            elif zumyDead:    
                pygame.draw.rect(screen, red, (x,y,width,height),0)
                pygame.draw.rect(screen, color, (x+zumyLine,y+zumyLine,width-2*zumyLine,height-2*zumyLine),0)
            # Draw the squares with the normal border
            else:
                pygame.draw.rect(screen, color, (x,y,width,height), 0)
                pygame.draw.rect(screen, black, (x,y,width,height), 1)
            # Draw the question marks
            if drawQuestion:
                label = quesfont.render("?", 1, (255,255,0))
                screen.blit(label, (x+width/3, int(y)))

            # Add the AR tag numbers
            label = numfont.render(str(int(gridArrayIDs[newGridInd[offset-i]])), 1, black)
            screen.blit(label, (x+width/3.5, y+height/3))    
        #Update the display
        pygame.display.update()
if __name__ == '__main__':
    main()