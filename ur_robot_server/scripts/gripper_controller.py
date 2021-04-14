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
import copy
import PyKDL
import numpy as np
from std_msgs.msg import Float32

#get link state
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState


class GripperController:
    def __init__(self, open_pose=0, close_pose=0.8, wait_for_completion=2):
        self.open_pose=open_pose
        self.close_pose=close_pose
        self.wait_for_completion=wait_for_completion
        self.current_goal=None
        self.client=None
        self.gripper_state=None
        self.init_gripper()

        #to request gripper position
        self.gripper_pose_service=rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    
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

    def __percentage_to_gap_size(self, desired_gap_size_percentage):
        '''
        Transforms the gap size percentage into mm
        Fully closed gripper-> 100%=1
        Fully open-> 0%=0
        '''

        #Scales 0%-100% to 0-1
        if desired_gap_size_percentage >1 and desired_gap_size_percentage <=100:
            relative_gap_size=desired_gap_size_percentage/100

        else:
            relative_gap_size=desired_gap_size_percentage

        #Calculates gap size in mm; adds the lowest value in case of offset
        if self.close_pose > self.open_pose:
            gap_size=self.open_pose + (self.close_pose-self.open_pose)*relative_gap_size
        else:
            gap_size=self.close_pose + (self.open_pose-self.close_pose)*(1-relative_gap_size)

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

    def __gap_size_to_state(self, desired_gap_size):
        '''
        Transforms the gap size into 0-1 value to use as gripper state
        0-> means open
        1-> means fully closed 
        '''

        if self.close_pose > self.open_pose:
            gap_size_topic=(desired_gap_size- self.open_pose  )/(self.close_pose - self.open_pose)

        else:
            gap_size_topic= (self.open_pose - desired_gap_size)/(self.open_pose  - self.close_pose)

        return np.float32(gap_size_topic)

    def __update_gripper_state(self, gap_size):
        """
        Updates internal vars
        """

        gripper_state=self.__gap_size_to_state(gap_size)
        self.gripper_state=gripper_state

        return self.gripper_state

    def command_gripper(self, desired_gap_size, percentage=False, max_effort=-1):
        """
        Commands gripper: sends the desired gap size, either in percentage (0%-100% or 0-1) or absolute distance (meters)
        Arguments:
            * desired_gap_size: gap size in one of these 3 forms:
                - 0%-100% (requires percentage=true). 0 is open, 100% is closed
                - 0-1 (requires percentage=true). 0 is open, 1 is closed
                - 0-0.085 m (percentage =false), this is the distance in mm.  0 is open, 0.8 is closed
        """
        self.__get_action_client()

        #gets gap size in mm
        if percentage:
            gap_size=self.__percentage_to_gap_size(desired_gap_size)
        else:
            gap_size=desired_gap_size

        self.__create_goal(gap_size, max_effort)

        self.__send_command()
        
        #solution: wait x seconds
        #time.sleep(self.wait_for_completion)

        self.__update_gripper_state(gap_size)

        return True

    def open_gripper(self, max_effort=-1):
        """
        Commands gripper: requests to open fingers
        """
        desired_gap_size=self.open_pose
        self.command_gripper(desired_gap_size, max_effort=max_effort)

        return True

    def close_gripper(self, max_effort=-1):
        """
        Commands gripper: requests to close fingers
        """
        desired_gap_size=self.close_pose
        self.command_gripper(desired_gap_size, max_effort=max_effort)

        return True

    def init_gripper(self):
        """
        Initialises gripper as fully open
        """
        
        self.open_gripper()

        rospy.loginfo("Gripper initialized as fully open")
        return True

    def get_gripper_pose(self):
        """
        returns the gripper pose (position + orientation), given by gazebo
        The return is a list in form: [x, y, z, r, p, y]
        """
        try:
            #to compute position
            rospy.wait_for_service('/gazebo/get_link_state')
            right_finger_pose = self.gripper_pose_service("robot::right_inner_finger", "world").link_state.pose

            #to compute position
            rospy.wait_for_service('/gazebo/get_link_state')
            left_finger_pose  = self.gripper_pose_service("robot::left_inner_finger",  "world").link_state.pose

            #gripper position estimation
            x = (right_finger_pose.position.x + left_finger_pose.position.x)/2
            y = (right_finger_pose.position.y + left_finger_pose.position.y)/2
            z = (right_finger_pose.position.z + left_finger_pose.position.z)/2


            #to compute orientation
            rospy.wait_for_service('/gazebo/get_link_state')
            wrist3_pose       = self.gripper_pose_service("robot::wrist_3_link",       "world").link_state.pose
            wrist3_orientation= wrist3_pose.orientation

            quaternion = PyKDL.Rotation.Quaternion(wrist3_orientation.x, wrist3_orientation.y, wrist3_orientation.z, wrist3_orientation.w)
            r,p,y = quaternion.GetRPY()

            gripper_pose = [x, y, z, r, p, y]

            return gripper_pose

        except rospy.ServiceException as e:
            print("Service call failed:" + e)