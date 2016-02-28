import random
from threading import Thread
import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from gazebo_msgs.msg import ModelStates,LinkStates,ModelState
import tf
import numpy as np
from std_msgs.msg import *
import os
import tf_conversions.posemath as pm
import PyKDL
from ros_cqi.robot_interface import RobotInterface

class GazeboRobotInterface(RobotInterface, object):
    """
    Abstract Client ROS class for manipulating Robots in Gazebo.
    """
    
    def __init__(self):
        super(GazeboRobotInterface, self).__init__()
        # This publisher is used to hack grasping by fixing an object's location.
        self.fix_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.linked_objects = []
        self.fix_thread = Thread(target=self.enforce_object_links, kwargs={"link_update_frequency": 0.0})
        self.fix_thread.start()
        self.model_poses = {}
        self.model_states = {}
        self.model_twists = {}
        self._sub_model = rospy.Subscriber("/gazebo/"+"model_states", ModelStates, self._cb_modeldata, queue_size=1)
        self._sub_links = rospy.Subscriber("/gazebo/"+"link_states", LinkStates, self._cb_linkdata, queue_size=1)

    def _cb_modeldata(self, msg):
        self.data_to_publish = {}
        for pos, item in enumerate(msg.name):
            twist = msg.twist[pos]
            pose = msg.pose[pos]
            self.model_twists[item] = twist
            self.model_poses[item] = pose
            ms = ModelState()
            ms.pose = pose
            ms.twist = twist
            ms.model_name = item
            self.model_states[item] = ms
            op_str = item + ", pose"
            val_str = str(ms.pose)
            self.data_to_publish[op_str] = str(val_str)
            if item != self.name:
                continue
            self.pose = msg.pose[pos]

    def _cb_linkdata(self, msg):
        for pos, item in enumerate(msg.name):
            twist = msg.twist[pos]
            pose = msg.pose[pos]
            self.model_twists[item] = twist
            self.model_poses[item] = pose
            ms = ModelState()
            ms.pose = pose
            ms.twist = twist
            ms.model_name = item
            self.model_states[item] = ms

    def add_object_link(self,o1, o2, offset):
        self.linked_objects.append([o1,o2,offset])

    def release_object_link(self, o1, o2):
        for li in self.linked_objects:
            if li[0] == o1 and li[1] == o2:
                self.linked_objects.remove(li)

    def enforce_object_links(self, link_update_frequency=0.1):
        while True:
            for l in self.linked_objects:
                o1 = l[0]
                o2 = l[1]
                offset = l[2]
                o1_state = self.model_states[o1]
                o2_state = o1_state
                o2_state.pose = self.add_poses(o1_state.pose, offset)
                o2_state.model_name = o2
                self.fix_publisher.publish(o2_state)
            rospy.sleep(link_update_frequency)
            if float(link_update_frequency) == 0.0:
                break


