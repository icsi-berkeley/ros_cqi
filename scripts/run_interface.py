#!/usr/bin/env python
import rospy
import inspect

from ros_cqi.darwin_interface import DarwinInterface
from ros_cqi.pr2_interface import PR2Interface
from ros_cqi.robot_interface import RobotInterface

from ros_cqi.gazebo_model import ModelLocationInterface
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
from std_msgs.msg import *

# di = DarwinInterface()    

def exec_pr2_demo(pr2i):
    # By default, PR2 starts at (3.5, 6.5)
    # Move in front of counter
    pr2i.execCommand("move_to_xy", ["3.5", "1.5"])
    pr2i.execCommand("move_to_xy", ["-0.45", "1.3"])
    pr2i.execCommand("move_to_xy", ["-0.7", "-0.18"])
    pr2i.execCommand("move_to_xy", ["-0.2", "-0.16"])
    pr2i.execCommand("move_to_xy", ["0.12", "-0.16"])

    # Open gripper
    pr2i.execCommand("open_gripper", [])

    # Move a little further
    pr2i.execCommand("move_to_xy", ["0.16", "-0.16"])

    # Grasp
    pr2i.execCommand("grasp_object", ["small beer"])

    # Move on to dining area
    pr2i.execCommand("move_to_xy", ["0.23", "0.6"])
    pr2i.execCommand("move_to_xy", ["0.5", "0.8"])
    pr2i.execCommand("move_to_xy", ["1.2", "0.9"])
    pr2i.execCommand("move_to_xy", ["3.5", "1.2"])
    pr2i.execCommand("move_to_xy", ["3.5", "5.8"])
    pr2i.execCommand("move_to_xy", ["1.7", "5.8"])

    pr2i.execCommand("release", [])

def exec_darwin_demo(di):
    di.init_pose()
    di.execCommand("move_to_xy", ["0.07", "5.7"])
    di.execCommand("grasp_object", ["blue marker"])
    di.execCommand("move_to_xy", ["-2.5", "6.3"])
    di.execCommand("release", [])
    # di.execCommand("move_to_xy", ["-2", "3"])


if __name__ == "__main__":
 
    rospy.init_node("cqi")

    # rospy.loginfo("Instantiating Darwin Interface Client")
    di = DarwinInterface()
    exec_darwin_demo(di)
    # di.listen()

    # rospy.loginfo("Instantiating PR2 Interface Client")
    # pr2i = PR2Interface()
    # exec_pr2_demo(pr2i)
    # pr2i.listen()

    rospy.spin()
    
    rospy.loginfo("Interface quit")


    


