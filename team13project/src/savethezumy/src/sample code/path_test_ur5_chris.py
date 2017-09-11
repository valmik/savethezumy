#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import exp_quat_func as eqf
import numpy as np
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
# from baxter_interface import gripper as baxter_gripper

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    manipulator = moveit_commander.MoveGroupCommander('manipulator')
    manipulator.set_planner_id('RRTConnectkConfigDefault')
    manipulator.set_planning_time(10)
    #manipulator.allow_replanning(True)

    # # New functions :)
    # manipulator.set_goal_position_tolerance(0.1)
    # manipulator.set_goal_orientation_tolerance(0.1)
    # manipulator.allow_replanning(True)
    # manipulator.set_pose_reference_frame('base')
    # manipulator.set_num_planning_attempts(100)

    # Testing stuff
    # print "Planning frame: %s" % manipulator.get_planning_frame()
    # print "End effector: %s" % manipulator.get_end_effector_link()
    # print "Robot groups: %s" % robot.get_group_names()
    # print "Robot state: %s" % robot.get_current_state()

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    goal_1.pose.position.x = 0.729
    goal_1.pose.position.y = 0.099
    goal_1.pose.position.z = 0.3484
    
    #Orientation as a quaternion
    goal_1.pose.orientation.x = -0.5718
    goal_1.pose.orientation.y = -0.4117
    goal_1.pose.orientation.z =  0.5842
    goal_1.pose.orientation.w = -0.4026
    # rot = np.array([-.5718,-0.4117,0.5842,-0.4026])
    # print "Omega and Theta"
    # print eqf.quaternion_to_exp(rot)
    # #Set the goal state to the pose you just defined
    manipulator.set_pose_target(goal_1)

    #Set the start state for the left arm
    manipulator.set_start_state_to_current_state()

    #Plan a pathmanipulator
    print "Computing Path 1...."
    manip_plan = manipulator.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
    manipulator.execute(manip_plan)

    #Second goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_2 = PoseStamped()
    goal_2.header.frame_id = "base"

    #x, y, and z position
    goal_2.pose.position.x = .6516
    goal_2.pose.position.y = .2566
    goal_2.pose.position.z = 0.3484
    
    #Orientation as a quaternion
    goal_2.pose.orientation.x = -0.5718
    goal_2.pose.orientation.y = -0.4117
    goal_2.pose.orientation.z = 0.5842
    goal_2.pose.orientation.w = -0.4026

    #Set the goal state to the pose you just defined
    manipulator.set_pose_target(goal_2)

    #Set the start state for the left arm
    manipulator.set_start_state_to_current_state()

    #Create a path constraint for the arm
    #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    orien_const = OrientationConstraint()
    orien_const.link_name = "ee_link";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = -1.0;
    # orien_const.orientation.y = -0.4117;
    # orien_const.orientation.z =  0.5842;
    # orien_const.orientation.w = -0.4026;
    orien_const.absolute_x_axis_tolerance = 0.5;
    orien_const.absolute_y_axis_tolerance = 0.5;
    orien_const.absolute_z_axis_tolerance = 0.5;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    manipulator.set_path_constraints(consts)

    
    #Plan a path
    print "Computing Path 2...."
    manip_plan = manipulator.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 2: ')
    manipulator.execute(manip_plan)


    #Third goal pose -----------------------------------------------------
    rospy.sleep(2.0)
    goal_3 = PoseStamped()
    goal_3.header.frame_id = "base"

    #x, y, and z position
    goal_3.pose.position.x = .6516
    goal_3.pose.position.y = .2566
    goal_3.pose.position.z = .25
    
    #Orientation as a quaternion
    goal_3.pose.orientation.x = -0.5718
    goal_3.pose.orientation.y = -0.4117
    goal_3.pose.orientation.z =  0.5842
    goal_3.pose.orientation.w = -0.4026

    #Set the goal state to the pose you just defined
    manipulator.set_pose_target(goal_3)

    #Set the start state for the left arm
    manipulator.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "left_gripper";left_arm
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    # consts = Constraints()
    # consts.orientation_constraints = [orien_const]
    # left_arm.set_path_constraints(consts)

    #Plan a path
    print "Computing Path 3...."
    manip_plan = manipulator.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm to goal pose 3: ')
    manipulator.execute(manip_plan)

if __name__ == '__main__':
    main()
