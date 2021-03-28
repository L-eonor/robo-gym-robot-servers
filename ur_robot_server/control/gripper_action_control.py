#!/usr/bin/python
# coding: latin-1
# Control the gripper opening though action client/server messages
#   Max and Min pose are, respectively close_pose(0.8) and open_pose(0)
#   Max pose corresponds to the gripper closed (fingers together).
#   Min pose corresponds to the gripper opened (fingers away of each other)

close_pose=0.8 #stroke: 0.85, check it out @ https://robotiq.com/products/2f85-140-adaptive-robot-gripper
open_pose=0

#dealing with arguments passed through a command line
#https://docs.python.org/3/howto/argparse.html
import argparse

#import ros client for python
#http://ros.org/wiki/rospy
import rospy

# action control of a given node
# http://wiki.ros.org/actionlib
import actionlib

# messages and action messages for controlling nodes
# https://wiki.ros.org/control_msgs
import control_msgs.msg


# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

import time

#routine to send the action message 
def gripper_client(value):
    # Creates the SimpleActionClient, passing the type of the action
    # (GripperCommandAction-http://docs.ros.org/en/melodic/api/control_msgs/html/action/GripperCommand.html) to the constructor.
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()


    ''' Creates a goal to send to the action server.
     for a GripperCommand, the required parameters are the position and max_effort, both float64
     as defined here: http://docs.ros.org/en/melodic/api/control_msgs/html/msg/GripperCommand.html
     float64 position
     float64 max_effort '''
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # gap size in meters, from 0.0 to 0.8m
    goal.command.max_effort = -1  # effort exerted (in Newtons), Do not limit the effort

    # Sends the goal to the action server.
    client.send_goal(goal)

    ''' gripper doesn't have feedback messages, it can be stuck here for not reaching the goal
     Waits for the server to finish performing the action. (action server protocol)
     GripperCommand command
     ---
     float64 position  # The current gripper gap size (in meters)
     float64 effort    # The current effort exerted (in Newtons)
     bool stalled      # True iff the gripper is exerting max effort and not moving
     bool reached_goal # True iff the gripper position has reached the commanded setpoint
    #client.wait_for_result()'''
    #solution: wait 5s
    time.sleep(5)

    # Prints out the result of executing the action
    #return client.get_result()  #might not reach goal, won't return result
    return True

if __name__ == '__main__':
    try:
        # Get the gap size from the command line (gripper_value)
        parser = argparse.ArgumentParser()
        help_string='Value betwewen '+str(open_pose)+' (open) and '+str(close_pose)+' (closed)'
        parser.add_argument("--value", type=float, default="0.2",
                            help='Value betwewen '+str(open_pose)+' (open) and '+str(close_pose)+' (closed)')
        args = parser.parse_args()
        gripper_value = args.value

        if gripper_value >=open_pose or gripper_value<=close_pose:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            rospy.init_node('gripper_command')

            # Set the value to the gripper
            result = gripper_client(gripper_value)
            print (result)
        else:
            print ("invalid gap size!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")