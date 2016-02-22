#!/usr/bin/env python
import rospy
import inspect

from ros_cqi.darwin_interface import DarwinInterface
from ros_cqi.pr2_interface import PR2Interface

from ros_cqi.gazebo_model import ModelLocationInterface
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from std_msgs.msg import *

# di = DarwinInterface()    



if __name__=="__main__":
 
    rospy.init_node("cqi")
    # rospy.loginfo("Instantiating Darwin Interface Client")
    # di = DarwinInterface()
    # di.execCommand("moveToXY", ["4.3", "3"])
    # rospy.sleep(2.5)
    # di.execCommand("grasp", [])
    # rospy.sleep(4.5)
    # di.execCommand("moveToXY", ["1", "1"])
    # rospy.sleep(2.5)
    # di.execCommand("release", [])
    # rospy.sleep(2.5)
    # di.execCommand("raise_arms", [])
    # # rospy.sleep(2.5)
    # di.execCommand("moveToXY", ["-2", "3"])

    rospy.loginfo("Instantiating PR2 Interface Client")
    pr2i = PR2Interface()
    pr2i.execCommand("moveToXY", ["3.5", "1.5"])
    rospy.sleep(1.5)
    pr2i.execCommand("moveToXY", ["3.0", "1.4"])
    rospy.sleep(1.5)
    pr2i.execCommand("moveToXY", ["-0.7", "1"])
    rospy.sleep(1.5)
    pr2i.execCommand("moveToXY", ["-1.7", "0.7"])
    rospy.sleep(1.5)
    pr2i.execCommand("moveToXY", ["-1.7", "1.3"])
    rospy.sleep(1.5)
    # pr2i.execCommand("moveToXY", ["0.4", "-1"])
    # rospy.sleep(1.5)
    # rospy.sleep(2.5)
    # di.execCommand("grasp", [])
    # rospy.sleep(4.5)
    # di.execCommand("moveToXY", ["1", "1"])
    # rospy.sleep(2.5)
    # di.execCommand("release", [])
    # rospy.sleep(2.5)
    # di.execCommand("raise_arms", [])
    # # rospy.sleep(2.5)
    # di.execCommand("moveToXY", ["-2", "3"])

    pr2i.listen()

        
    rospy.spin()
    
    rospy.loginfo("Interface quit")

    


