import random
from threading import Thread
import math
import rospy
import time
from gazebo_msgs.msg import ModelStates,LinkStates
# from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class ModelLocationInterface:
    """
    Client ROS class for identifying objects in the 3d model
    """
    
    def __init__(self,ns="/gazebo/"):
        self.ns=ns
        
        while True:
            self._sub_model=rospy.Subscriber(ns+"model_states",ModelStates,self._cb_modeldata,queue_size=1)
            rospy.sleep(5)

        
    def _cb_modeldata(self,msg):
        print "model data:"
        modelLocations = {}
        for pos,item in enumerate(msg.name):            
            # print "for " + item
            # print msg.twist[pos].linear
            # raw_input()
            # modelLocations[item] = msg.twist[pos]
            modelLocations[item] = msg.pose[pos]
            # modelLocations[item]["linear"] = msg.pose[pos].linear
            # modelLocations[item]["angular"] = msg.pose[pos].angular
        # print "locations"
        print modelLocations
        # print "\n"
        self._sub_model.unregister()

        
    
   