#!/usr/bin/python
# coding: latin-1



import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.srv import GetWorldProperties
import copy
import numpy as np
import PyKDL

ground_plane_name="ground_plane"
robot_name="robot"

class ObjectsController:
    """
    Allows to query the obstacles in the enviornment, return the environment's state and ask the state to change their position
    """

    def __init__(self):

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
            self.object_names=[model_name for model_name in world_prop_result.model_names if (model_name!=ground_plane_name and model_name!=robot_name) ]

        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def get_objects_state(self):
        """
        Retrieves the state of the available objects, except the ground_plane and the robot

        Arguments:
            None
        Returns:
            * self.state (np.array  (nr objects, 8) )- each line depicts an object (cube).  name index (i) + position (x, y, z) + orientation (x, y, z, w)
            * msg (list, 7)-> #0->id, #1-> x, #2->y, #3-> z, #4->r, #5->p, #6->y
        """
        
        try:

            self.state=[]
            msg=[]
            for model_id in range(len(self.object_names)):
                #waits for sevice, requests info and saves i
                rospy.wait_for_service('/gazebo/get_model_state')
                model_name=self.object_names[model_id]
                model_state_result = self.get_model_state_service(model_name, "")
                        
                #observation: model name+pose
                obj_pose=[model_id, model_state_result.pose.position.x, model_state_result.pose.position.y, model_state_result.pose.position.z, \
                          model_state_result.pose.orientation.x, model_state_result.pose.orientation.y, model_state_result.pose.orientation.z, model_state_result.pose.orientation.w]     
                
                #updates state array
                if (len(self.state)==0):
                    self.state=np.array(obj_pose, dtype=np.float32)
                    self.state.shape=(1, len(obj_pose))
                else:
                    self.state=np.vstack((self.state, obj_pose))


                #quaternion to rpy
                quaternion = PyKDL.Rotation.Quaternion(obj_pose[4],obj_pose[5],obj_pose[6], obj_pose[7])
                r,p,y = quaternion.GetRPY()
                target = obj_pose[0:4] + [r,p,y] #0->id, 1, 2, 3-> x, y, z

                msg.extend(target)
            

            #return self.state
            return msg

        except rospy.ServiceException as e:
            print("Service call failed:" + e)
