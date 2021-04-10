#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.srv import GetWorldProperties
import copy
import numpy as np

ground_plane_name="ground_plane"

class object_controller():
    """
    Allows to query the obstacles in the enviornment, return the environment's state and ask the state to change their position
    """

    def __init__(self, robo_name):
        self.robo_name=robo_name

        #to request world properties-> get to know the different objects
        self.world_prop_service=rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

        #to request model state/info
        self.get_model_state_service=rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        #to set model state
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #array to sabe objects' state
        self.state=[]

        #requests objects' model names
        self.__get_available_objects()
        

    def __get_available_objects(self):
        """
        Returns the available object's names, except ground plane and robot
        Object names saved in self.object names (list)
        """
        #waits for the model state to be available
        rospy.wait_for_service('/gazebo/get_model_state')
        try:    
            #requests model properties
            world_prop_result=self.world_prop_service()

            #evaluate whether the objects are different from gound plane and robot, saves names
            self.object_names=[model_name for model_name in world_prop_result.model_names if (model_name!=ground_plane_name and model_name!=self.robo_name) ]
            print(self.object_names)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def get_objects_state(self):
        """
        Retrieves the state of the available objects, except the ground_plane and the robot

        Arguments:
            None
        Returns:
            * self.state (np.array)- each column depicts an object (cube), first column is the name, second column is the pose. Pose is position (x, y, z) + orientation (x, y, z, w)
        """
        try:
            for model_name in self.object_names:
                #waits for sevice, requests info and saves i
                rospy.wait_for_service('/gazebo/get_model_state')
                model_state_result = self.get_model_state_service(model_name, "")
                        
                #observation: model name+pose
                pose=np.array([model_name, model_state_result.pose])
                        
                #updates state array
                if (len(self.state)==0):
                    self.state=np.array(pose)
                else:
                    self.state=np.vstack((self.state, pose))
            return self.state

        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def set_models_state(self):

        state_msg = ModelState()

        for object_name in self.object_names:
            print(object_name)
            state_msg.model_name = object_name
            state_msg.pose.position.x =  - 1
            state_msg.pose.position.y =  - 1
            state_msg.pose.position.z =  - 1
            state_msg.pose.orientation.x = 1
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0
            print(state_msg)

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_model_state_result = self.set_model_state_service(state_msg)
                print(set_model_state_result)

            except rospy.ServiceException as e:
                print("Service call failed:" + e)




"""
if __name__ == '__main__':
    objs=object_controller("robot")
    state=objs.get_objects_state()
    print(state)

    objs.set_models_state()

    
    
    state=get_models_state()

    print("********************************")
    print("Setting Cube State")
    print("********************************")

    state_msg = ModelState()

    state_msg.model_name = "cube"
    state_msg.pose.position.x = state.pose.position.x + 1
    state_msg.pose.position.y = state.pose.position.y + 1
    state_msg.pose.position.z = state.pose.position.z + 1
    state_msg.pose.orientation.x = 1
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    print(state_msg)

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state_result = set_model_state_service(state_msg)
        print(set_model_state_result)

    except rospy.ServiceException as e:
        print("Service call failed:" + e)
"""
