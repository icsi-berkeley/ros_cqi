import random
from threading import Thread
import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from gazebo_msgs.msg import ModelStates,LinkStates
import tf
import numpy as np
from std_msgs.msg import *
from ros_cqi.gazebo_model import ModelLocationInterface
import os

class RobotInterface:
    """
    Abstract Client ROS class for manipulating Robots.
    """
    
    def __init__(self):
        self.name = ""
        self.joints=None
        self.angles=None
        self._sub_model=rospy.Subscriber("gazebo/model_states",ModelStates,self._cb_modeldata,queue_size=1)
        self._subscribe_joints()
        self.possible_comands = {}
        self.pose = None

        rospy.loginfo("Listening for commands")
        self._sub_nlu=rospy.Subscriber("/cqi/command",String,self.cb_nlu,queue_size=5)

    def _subscribe_joints(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def listen(self):
        while True:
            print("Enter one of the possible commands. " +
                  "Enter command name and arguments in parenthesis. " +
                  "Use empty parenthesis when no arguments are provided. " +
                  "Enter q to exit.")
            for comm, args in self.possible_comands.items():
                print(comm + "(" + str(args).replace("[", "").replace("]", "") + ")")

            txt_in = raw_input()
            if txt_in == "q":
                os._exit(1)
            if txt_in != "":
                comm, args = self.decode_command_input(txt_in)
                self.execCommand(comm, args)
            # # self.execCommand("lower_arms", [])
            # self.execCommand("moveToXY", ["4.3", "3"])
            # rospy.sleep(2.5)
            # self.execCommand("grasp", [])
            # rospy.sleep(4.5)
            # self.execCommand("moveToXY", ["1", "1"])
            # rospy.sleep(2.5)
            # self.execCommand("release", [])
            # rospy.sleep(2.5)
            # self.execCommand("raise_arms", [])
            # # rospy.sleep(2.5)
            # self.execCommand("moveToXY", ["-2", "3"])
            # # break

    @staticmethod
    def decode_command_input(comm_input):
        commandName = comm_input.split("(")[0]
        commandArgs = comm_input[:-1].split("(")[1].split(",")
        return commandName, commandArgs

    def cb_nlu(self,msg):
        print "received msg:"
        print msg
        commandName, commandArgs = self.decode_command_input(msg.data)
        self.execCommand(commandName, commandArgs)

    def _cb_modeldata(self,msg):
        # print "model data:"
        modelLocations = {}
        for pos,item in enumerate(msg.name):
            if item != self.name:
                continue
            self.pose = msg.pose[pos]

    def execCommand(self, commandName, commandArgs):
        if commandName not in self.possible_comands:
            rospy.logwarn("Invalid command given: {}".format(commandName))

        if commandName == "moveToXY":
            x = float(commandArgs[0])
            y = float(commandArgs[1])
            self.moveToXY(x, y)

        if commandName == "moveToPose":
            x = float(commandArgs[0])
            y = float(commandArgs[1])
            theta = float(commandArgs[2])
            self.moveToPose(x, y, theta)

        if commandName == "grasp":
            self.grasp()

        if commandName == "raise_arms":
            self.raise_arms()

        if commandName == "release":
            self.release()

        if commandName == "spread_arms":
            self.spread_arms()

        if commandName == "open_gripper":
            self.open_gripper()

        if commandName == "graspObject":
            self.grasp_object(commandArgs)

    def moveToXY(self, x, y):
        rospy.logerr("Virtual function needs to be overwritten.")

    def moveToPose(self, x, y, theta):
        rospy.logerr("Virtual function needs to be overwritten.")

    def grasp(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def raise_arms(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def release(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def release(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def spread_arms(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def open_gripper(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    def grasp_object(self):
        rospy.logerr("Virtual function needs to be overwritten.")

    @staticmethod
    def eulerFromQuarternion(pose):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    @staticmethod
    def quaternion2DFromAngle(angle):
        roll = 0
        pitch = 0
        yaw = angle
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    @staticmethod
    def interpolate(anglesa,anglesb,coefa):
        z={}
        joints=anglesa.keys()
        for j in joints:
            z[j]=anglesa[j]*coefa+anglesb[j]*(1-coefa)
        return z

    @staticmethod
    def get_distance(anglesa,anglesb):
        d=0
        joints=anglesa.keys()
        if len(joints)==0: return 0
        for j in joints:
            d+=abs(anglesb[j]-anglesa[j])
        d/=len(joints)
        return d

    @staticmethod
    def angle_between2d(a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        rads = math.atan2(dy,dx)
        # rads %= 2*math.pi
        return rads

    @staticmethod
    def unit_vector(vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    @staticmethod
    def angle_between(v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1_u = RobotInterface.unit_vector(v1)
        v2_u = RobotInterface.unit_vector(v2)
        angle = np.arccos(np.dot(v1_u, v2_u))
        if np.isnan(angle):
            if (v1_u == v2_u).all():
                return 0.0
            else:
                return np.pi
        return angle
