#!/usr/bin/env python
import rospy
import inspect

from darwin_interface.darwin_interface import DarwinInterface
# from darwin_interface.gazebo_model import ModelLocationInterface
from geometry_msgs.msg import Twist,Pose,Quaternion,Point

if __name__=="__main__":
    rospy.init_node("darwin_interface")
    
    rospy.loginfo("Instantiating Darwin Interface Client")
    di=DarwinInterface()
    # rospy.loginfo("Instantiating Model Location Interface Client")
    # mli = ModelLocationInterface()
    
    rospy.sleep(1)

    destination = Pose()
    
    destination.orientation.x = 1
    destination.orientation.y = 1
    destination.orientation.z = 1
    destination.orientation.w = 1

    destination.position.x = 1
    destination.position.y = 1
    destination.position.z = 1

    di.walkToPose(destination)

    
 
    # rospy.loginfo("Darwin Interface Starting")

    # di.set_walk_velocity(0.5,0,0.5)

    # rospy.sleep(10)

    # di.set_walk_velocity(0,0,0)    
    
    # rospy.spin()
    
    rospy.loginfo("Interface quit")