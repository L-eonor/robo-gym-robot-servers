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
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

            self.object_types_str=[model_name[0:-3] for model_name in self.object_names]
            self.object_types_int=[np.where(object_type==np.unique(self.object_types_str))[0][0] for object_type in self.object_types_str ]
            self.object_dimensions=np.reshape([list(map(int, (model_type[-8:].split("_", 3)))) for model_type in self.object_types_str], (-1, 3))/100.0

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
                #gets object dimensions, converts from cms to meters, depth, width, height
                model_dim=self.object_dimensions[model_id]

                #waits for sevice, requests info and saves i
                model_name=self.object_names[model_id]
                #model state gives z top face idk why, trying link pose instead
                rospy.wait_for_service('/gazebo/get_link_state')
                model_state_result=self.get_cube_link_service(model_name+"::link", "world").link_state
                        
                #observation: model name+pose
                obj_pose=[model_id, model_dim[0], model_dim[1], model_dim[2], \
                          model_state_result.pose.position.x, model_state_result.pose.position.y, model_state_result.pose.position.z, \
                          model_state_result.pose.orientation.x, model_state_result.pose.orientation.y, model_state_result.pose.orientation.z, model_state_result.pose.orientation.w]     
                
                #updates state array
                if (len(self.state)==0):
                    self.state=np.array(obj_pose, dtype=np.float32)
                    self.state.shape=(1, len(obj_pose))
                else:
                    self.state=np.vstack((self.state, obj_pose))


                #quaternion to rpy
                quaternion = PyKDL.Rotation.Quaternion(model_state_result.pose.orientation.x, model_state_result.pose.orientation.y, model_state_result.pose.orientation.z, model_state_result.pose.orientation.w)
                r,p,y = quaternion.GetRPY()
                target = obj_pose[0:7] + [r,p,y] #0->id, 1, 2, 3-> size, 4, 5, 6-> x, y, z
                #print(target)
                #print([model_state_result.pose.orientation.x, model_state_result.pose.orientation.y, model_state_result.pose.orientation.z, model_state_result.pose.orientation.w])
                #print([r, p, y])
                #print(quaternion.GetEulerZYX())
                #print(euler_from_quaternion([model_state_result.pose.orientation.x, model_state_result.pose.orientation.y, model_state_result.pose.orientation.z, model_state_result.pose.orientation.w], axes='sxyz'))
                msg.extend(target)        
            
            return msg

        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def cubes_new_pose(self, cube_order):
        
        """Get pose of a random point in the UR5 workspace.

        Returns:
            np.array: [x,y,z,alpha,theta,gamma] pose.

        """
        pose=np.zeros(6)

        singularity_area = True
        '''
        # check if generated x,y,z are in singularityarea
        while singularity_area:
            # Generate random uniform sample in semisphere taking advantage of the
            # sampling rule

            # UR5 workspace radius
            # Max d = 1.892
            #R =  0.900 # reduced slightly
            R =  0.800

            #phi = np.random.default_rng().uniform(low= 0.0, high= 2*np.pi)
            #phi = np.random.uniform(low= 0.0, high= 2*np.pi)
            phi = np.random.uniform(low= (3*np.pi/2), high= 2*np.pi)
            #phi = np.pi/self.number_of_objs * cube_order
            #u = np.random.default_rng().uniform(low= 0.0, high= 1.0)
            u = np.random.uniform(low= 0.1, high= 1.0)

            r = R * np.cbrt(u)

            x = r * np.cos(phi)
            y = r * np.sin(phi)
            z = 0

            if (x**2 + y**2) > 0.085**2:
                singularity_area = False
        '''
        x = np.random.uniform(low= 0.20, high= 0.6)
        y = np.random.uniform(low=-0.55, high= 0)
        z = 0

        #random position
        pose[:3]=[x, y, z]


        #random roll-> quaternion-> set object
        random_roll=np.random.random()*np.pi
        quaternion=quaternion_from_euler(0, 0, random_roll)
        
        return pose, quaternion

    def reset_cubes_pos(self):
        state_msg = ModelState()

        self.number_of_objs=len(self.object_names)
        i=-1
        for object_name in self.object_names:
            i+=1

            state_msg.model_name = object_name

            pose, quaternion=self.cubes_new_pose(i)
            rospy.wait_for_service('/gazebo/get_link_state')
            model_state_result=self.get_cube_link_service(object_name+"::link", "world").link_state

            #random posixion [x, y, z]
            state_msg.pose.position.x =  pose[0]
            state_msg.pose.position.y =  pose[1]
            state_msg.pose.position.z =  self.object_dimensions[i][-1]/2 #model_state_result.pose.position.z #do not change z, it has to be the center of mass!

            #print(random_roll)
            
            state_msg.pose.orientation.x = quaternion[0]
            state_msg.pose.orientation.y = quaternion[1]
            state_msg.pose.orientation.z = quaternion[2]
            state_msg.pose.orientation.w = quaternion[3]
            #print([state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w])

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_model_state_result = self.set_model_state_service(state_msg)
                #print(set_model_state_result)

            except rospy.ServiceException as e:
                print("Service call failed:" + e)



if __name__ == '__main__':
    objs=ObjectsController()
    objs.reset_cubes_pos()
