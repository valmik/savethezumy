#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import tf
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from geometry_msgs.msg import Transform, Vector3, Wrench
import math
import numpy as np
import kinematics as kin
import exp_quat_func as eqf

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#Q1 = [-0.026, -1.91, -1.70, -1.17, 1.66, 0]

client = None
forceReading = None

def move1():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
        client.send_goal(g)
        time.sleep(3.0)
        print "Interrupting"
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

# def init_favorable_pos():
def test_z():

    #Input joint_angles
    #1x6 array of UR5 joint angles
    global forceReading

    #Create instance of force sensor subscriber
    force_sensor_sub = rospy.Subscriber("ati_ft_data", Wrench, force_sensor_callback)

    ##Solve for robot initial states
    #initial_pos = MTF.map_to_tool(joint_states)

    time.sleep(0.2) #This allows the subscriber to obtain the first force reading

    ##Solve XY IK solution
    #final_states = IK.solveXY(corner_tag,desired_pos,initial_pos)

    ##Set movement speed
    #This will be MUCH slower so we will set a velocity variable here to calculate time
    #z_dist = 2 in travel
    move_time_slow = 5.0
    move_time = 2.0

    conversion = pi/180

    desired_pos = np.dot([0,-140,-97,-31,90,0],conversion)

    ##SENSOR sample time == approx. 0.008 seconds

    #Get force value from sensor
    vector_force = forceReading.force
    #Calculate a force value from force vector
    #value_force = np.sqrt(np.dot(vector_force,vector_force))
    value_force_calib = vector_force.z

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try: 
        joint_states = rospy.wait_for_message("joint_states", JointState)
        above_block = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=above_block, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=desired_pos, velocities=[0]*6, time_from_start=rospy.Duration(move_time_slow))]
        
        #Do not insert a wait_for_result in order to allow for interrupts
        client.send_goal(g)
        #client.wait_for_result()
        start_time = time.time()

        value_force = abs(forceReading.force.z - value_force_calib)
        maxForce=0
        force_threshold = 400

        current_time = time.time()
        #listen to the FORCE value
        while abs(current_time - start_time) < move_time_slow:
            if value_force > force_threshold:
                print "Interrupting Wood Block Detected"
                break

            value_force = abs(forceReading.force.z - value_force_calib)
            if value_force > maxForce:
                maxForce=value_force
                print(maxForce)

            #update time
            current_time = time.time()

        joint_states = rospy.wait_for_message("joint_states", JointState)
        init_state = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=init_state, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=above_block, velocities=[0]*6, time_from_start=rospy.Duration(move_time))]
        client.send_goal(g)
        client.wait_for_result()

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

    #Move the UR5 to first corner of the grid by calling Valmik IK function

def force_sensor_callback(message):

    global forceReading
    forceReading = message

def test_ik():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    q_des = np.array([-.8, -.23, .018])
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        listener = tf.TransformListener()
        a = 0
        while a == 0:
            try:
                (trans, rot) = listener.lookupTransform('base','ee_link', rospy.Time(0))
                a=1
            except Exception as e: 
                a=0

        theta = kin.ik(q_des)

        print kin.rad2deg(theta)
        raw_input()


        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=theta, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
   
def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        #print str([Q1[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):

            test_ik()

        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
