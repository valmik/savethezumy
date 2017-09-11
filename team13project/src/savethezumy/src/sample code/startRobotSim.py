#!/usr/bin/env python
import os
# The goal of this is to make one file that can launch all of our roslaunches
os.system("roslaunch ur_gazebo ur5.launch limited:=true")
os.system("roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true")
os.system("roslaunch ur5_moveit_config moveit_rviz.launch config:=true")
