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
    # By default, PR2 starts at (3.5, 6.5)
    # Move in front of counter
    pr2i.execCommand("moveToXY", ["3.5", "1.5"])
    pr2i.execCommand("moveToXY", ["3.0", "1.4"])
    pr2i.execCommand("moveToXY", ["-0.45", "0.6"])
    pr2i.execCommand("moveToXY", ["-0.5", "0.0"])
    pr2i.execCommand("moveToXY", ["-0.5", "-0.15"])
    pr2i.execCommand("moveToXY", ["-0.2", "-0.15"])
    pr2i.execCommand("moveToXY", ["0.15", "-0.15"])
    # Open grippers, move wrist downwards
    pr2i.execCommand("open_gripper", [])
    # Move another bit forward
    pr2i.execCommand("moveToXY", ["0.2", "-0.15"])
    # Grasp
    pr2i.execCommand("grasp", [])

    rospy.sleep(1.5)

    pr2i.listen()

        
    rospy.spin()
    
    rospy.loginfo("Interface quit")

    


