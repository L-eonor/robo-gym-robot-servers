#!/usr/bin/python
# coding: latin-1
# Control the gripper opening though action client/server messages
#   Max and Min pose are, respectively close_pose(0.8) and open_pose(0)
#   Max pose corresponds to the gripper closed (fingers together).
#   Min pose corresponds to the gripper opened (fingers away of each other)



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


close_pose=0.8 #stroke: 0.85, check it out @ https://robotiq.com/products/2f85-140-adaptive-robot-gripper
open_pose=0

class GripperController:
    def __init__(self, open_pose=0, close_pose=0.8, wait_for_completion=3):
        self.open_pose=open_pose
        self.close_pose=close_pose
        self.wait_for_completion=wait_for_completion
        self.current_goal=None
        self.client=None
        self.init_gripper()
    
    def __create_goal(self, position, max_effort=-1):
        ''' 
        Creates a goal to send to the action server.
        for a GripperCommand, the required parameters are:
            *position
            *max_effort, both float64
        
        Command format:
        float64 position
        float64 max_effort
        
        More info @  http://docs.ros.org/en/melodic/api/control_msgs/html/msg/GripperCommand.html
        '''

        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = position   # gap size in meters, from 0.0 to 0.8m
        goal.command.max_effort = -1  # effort exerted (in Newtons), Do not limit the effort

        self.current_goal=goal

        return goal


    def __get_action_client(self):
        '''
        Creates the SimpleActionClient, passing the type of the action
        Also, waits for the server to be able to receive commands
        The action command tipe is GripperCommandAction

        More info @ http://docs.ros.org/en/melodic/api/control_msgs/html/action/GripperCommand.html)
        '''
        client = actionlib.SimpleActionClient(
            '/gripper_controller/gripper_cmd',  # namespace of the action topics
            control_msgs.msg.GripperCommandAction # action type
        )

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        self.client=client

        return client

    def __percentage_to_gap_size(self, percentage):
        '''
        Transforms the gap size percentage into mm
        Fully open gripper-> 100%=1
        Fully closed-> 0%=0
        '''

        #Scales 0%-100% to 0-1
        if percentage >1 and percentage <=100 and percentage >=0:
            absolute_percentage=percentage/100

        else:
            absolute_percentage=percentage

        #Calculates gap size in mm
        if self.close_pose > self.open_pose:
            gap_size=(1-absolute_percentage)*self.close_pose
        else:
            gap_size=absolute_percentage*self.open_pose

        return gap_size
        
    def __send_command(self):
        ''' gripper doesn't have feedback messages, it can be stuck here for not reaching the goal
        Waits for the server to finish performing the action. (action server protocol)
        GripperCommand command
        ---
        float64 position  # The current gripper gap size (in meters)
        float64 effort    # The current effort exerted (in Newtons)
        bool stalled      # True iff the gripper is exerting max effort and not moving
        bool reached_goal # True iff the gripper position has reached the commanded setpoint
        #client.wait_for_result()'''

        # Sends the goal to the action server.
        self.client.send_goal(self.current_goal)
        
        return True
            
    def command_gripper(self, desired_gap_size, percentage=False, max_effort=-1):
        self.__get_action_client()

        #gets gap size in mm
        if percentage:
            gap_size=self.__percentage_to_gap_size(desired_gap_size)
        else:
            gap_size=desired_gap_size

        self.__create_goal(gap_size, max_effort)

        self.__send_command()
        
        #solution: wait x seconds
        time.sleep(self.wait_for_completion)

        return True

    def open_gripper(self, max_effort=-1):
        desired_gap_size=self.open_pose
        self.command_gripper(desired_gap_size, max_effort=max_effort)

        return True

    def close_gripper(self, max_effort=-1):
        desired_gap_size=self.close_pose
        self.command_gripper(desired_gap_size, max_effort=max_effort)

        return True

    def init_gripper(self):
        
        self.open_gripper()

        rospy.loginfo("Gripper initialized as fully open")
        return True

