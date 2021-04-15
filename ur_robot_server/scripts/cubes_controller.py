#!/usr/bin/python
# coding: latin-1



import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.srv import GetLinkState
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

        #to request cube link position
        self.get_cube_link_service=rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        #array to sabe objects' state
        self.state=[]

        #requests objects' model names
        self.__get_available_objects()

        #no need to reset on object creation
        #self.reset_cubes_pos()

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
                model_name=self.object_names[model_id]
                #model state gives z top face idk why, trying link pose instead
                #rospy.wait_for_service('/gazebo/get_model_state')
                #model_state_result = self.get_model_state_service(model_name, "world")
                rospy.wait_for_service('/gazebo/get_link_state')
                model_state_result=self.get_cube_link_service(model_name+"::link", "world").link_state
                        
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

    def cubes_new_pose(self):
        
        """Get pose of a random point in the UR5 workspace.

        Returns:
            np.array: [x,y,z,alpha,theta,gamma] pose.

        """
        pose=np.zeros(6)

        singularity_area = True

        # check if generated x,y,z are in singularityarea
        while singularity_area:
            # Generate random uniform sample in semisphere taking advantage of the
            # sampling rule

            # UR5 workspace radius
            # Max d = 1.892
            R =  0.900 # reduced slightly

            #phi = np.random.default_rng().uniform(low= 0.0, high= 2*np.pi)
            phi = np.random.uniform(low= 0.0, high= 2*np.pi)
            #costheta = np.random.default_rng().uniform(low= 0.0, high= 1.0) # [-1.0,1.0] for a sphere
            costheta = 0
            #u = np.random.default_rng().uniform(low= 0.0, high= 1.0)
            u = np.random.uniform(low= 0.0, high= 1.0)

            theta = np.arccos(costheta)
            r = R * np.cbrt(u)

            x = r * np.sin(theta) * np.cos(phi)
            y = r * np.sin(theta) * np.sin(phi)
            z = r * np.cos(theta)

            if (x**2 + y**2) > 0.085**2:
                singularity_area = False

        pose[:3]=[x, y, z]

        return pose

    def set_models_state(self):

        state_msg = ModelState()

        for object_name in self.object_names:
            
            state_msg.model_name = object_name

            state_msg.pose.position.x =  1
            state_msg.pose.position.y =  1
            state_msg.pose.position.z =  0
            state_msg.pose.orientation.x = 1
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0
            #print(state_msg)

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_model_state_result = self.set_model_state_service(state_msg)
                #print(set_model_state_result)

            except rospy.ServiceException as e:
                print("Service call failed:" + e)

    def reset_cubes_pos(self):
        state_msg = ModelState()

        for object_name in self.object_names:
            
            state_msg.model_name = object_name

            pose=self.cubes_new_pose()
            rospy.wait_for_service('/gazebo/get_link_state')
            model_state_result=self.get_cube_link_service(object_name+"::link", "world").link_state

            state_msg.pose.position.x =  pose[0]
            state_msg.pose.position.y =  pose[1]
            state_msg.pose.position.z =  model_state_result.pose.position.z #do not change z, it has to be the center of mass!
            state_msg.pose.orientation.x = 1
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0
            #print(state_msg)

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_model_state_result = self.set_model_state_service(state_msg)
                #print(set_model_state_result)

            except rospy.ServiceException as e:
                print("Service call failed:" + e)



if __name__ == '__main__':
    objs=ObjectsController()
    objs.reset_cubes_pos()
