import random
from threading import Thread
import math
import rospy
import time
from gazebo_msgs.msg import ModelStates,LinkStates,ModelState
from geometry_msgs.msg import Twist


class ModelLocationInterface:
    """
    Client ROS class for identifying objects in the 3d model
    """
    
    def __init__(self,ns="/gazebo/"):
        self.ns=ns
        self.model_poses = {}
        self.model_states = {}
        self.model_twists = {}
        self._sub_model=rospy.Subscriber(ns+"model_states", ModelStates, self._cb_modeldata, queue_size=1)
        self._sub_links=rospy.Subscriber(ns+"link_states", LinkStates, self._cb_linkdata, queue_size=1)
        
    def _cb_modeldata(self, msg):
        for pos,item in enumerate(msg.name):
            twist = msg.twist[pos]
            pose = msg.pose[pos]
            self.model_twists[item] = twist
            self.model_poses[item] = pose
            ms = ModelState()
            ms.pose = pose
            ms.twist = twist
            ms.model_name = item
            # ms.reference_frame = ''
            self.model_states[item] = ms

    def _cb_linkdata(self, msg):
        for pos,item in enumerate(msg.name):
            twist = msg.twist[pos]
            pose = msg.pose[pos]
            self.model_twists[item] = twist
            self.model_poses[item] = pose
            ms = ModelState()
            ms.pose = pose
            ms.twist = twist
            ms.model_name = item
            # ms.reference_frame = ''
            self.model_states[item] = ms

    
   