#!/usr/bin/env python
import rospy
import inspect

from ros_cqi.darwin_interface import DarwinInterface
from ros_cqi.gazebo_model import ModelLocationInterface
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from std_msgs.msg import *

# di = DarwinInterface()    



if __name__=="__main__":
 
    rospy.init_node("cqi")
    
    rospy.loginfo("Instantiating Darwin Interface Client")
    
    di = DarwinInterface()    
        
    rospy.spin()
    
    rospy.loginfo("Interface quit")

    


