#!/usr/bin/env python
import rospy
import inspect

from ros_cqi.darwin_interface import DarwinInterface
from ros_cqi.pr2_interface import PR2Interface
from ros_cqi.robot_interface import RobotInterface

from ros_cqi.gazebo_model import ModelLocationInterface
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from std_msgs.msg import *

def exec_pr2_demo(pr2i):
    # By default, PR2 starts at (3.5, 6.5)

    # Move in front of counter
    pr2i.execCommand("move_to_pose", ["0.16", "-0.16", "0"])
    # Grasp
    pr2i.execCommand("grasp_object", ["soda_can"])
    # Move on to dining area
    pr2i.execCommand("move_to_pose", ["1.7", "5.8", "0"])
    # Release
    pr2i.execCommand("release", [])

def exec_darwin_demo(di):
    # Move to marker
    di.execCommand("move_to_pose", ["0.07", "5.7", "0"])
    # Gasp marker
    di.execCommand("grasp_object", ["blue_marker"])
    # Move to armchair
    di.execCommand("move_to_pose", ["-2.5", "6.3", "0"])
    # release marker
    di.execCommand("release", [])


if __name__ == "__main__":
 
    rospy.init_node("cqi")

    di = DarwinInterface()
    # exec_darwin_demo(di)
    # di.listen_for_console_input()

    # pr2i = PR2Interface()
    # exec_pr2_demo(pr2i)
    # pr2i.listen_for_console_input()

    rospy.spin()
    
    rospy.loginfo("Interface quit")


    


