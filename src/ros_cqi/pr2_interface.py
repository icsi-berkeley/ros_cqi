import random
from threading import Thread
import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.msg import ModelStates,LinkStates
import tf
import numpy as np
from std_msgs.msg import *
from ros_cqi.gazebo_model import ModelLocationInterface
from ros_cqi.robot_interface import RobotInterface
from pr2_controllers_msgs.msg import Pr2GripperCommand


import os
# from pr2_controllers_msgs import Pr2GripperCommand


class PR2Interface(RobotInterface, object):
    """
    Client ROS class for manipulating Darwin OP in Gazebo
    """
    
    def __init__(self):
        super(PR2Interface, self).__init__()
        self.name = "pr2"
        self.possible_comands = {"moveToXY": ["x","y"],
                                 "moveToPose": ["x","y","ang(rad)"],
                                 "graspObject": ["label"],
                                 "grasp": [],
                                 "release": [],
                                 "raise_arms": [],
                                 "spread_arms": [],
                                 "open_gripper": []}
        self.pose = Pose()
        self.v_factor = 1

        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None: break
            rospy.sleep(0.1)
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        self.controllers = {"base_controller": Twist,
                            "base_odometry": String,
                            "head_traj_controller": String,
                            "l_arm_controller": JointTrajectory,
                            "l_gripper_controller": Pr2GripperCommand,
                            "r_arm_controller": JointTrajectory,
                            "r_gripper_controller": Pr2GripperCommand,
                            "torso_controller": String}
        self.joints = {}
        self.joints["r_arm_controller"] = ["r_shoulder_pan_joint",
                                           "r_shoulder_lift_joint",
                                           "r_upper_arm_roll_joint",
                                           "r_elbow_flex_joint",
                                           "r_forearm_roll_joint",
                                           "r_wrist_flex_joint",
                                           "r_wrist_roll_joint"]

        rospy.loginfo("Creating controller command publishers")
        self._pub_controllers = {}
        for c,v in self.controllers.items():
            p=rospy.Publisher("/"+c+"/command",v,queue_size=1)
            self._pub_controllers[c]=p

        # self._pub_cmd_vel = rospy.Publisher("/base_controller/command", Twist, queue_size=1)

    def _subscribe_joints(self):
        self._sub_joints = rospy.Subscriber("/joint_states", JointState, self._cb_joints, queue_size=1)

    def set_move_velocity(self, x, y, t):
        v_factor = self.v_factor
        msg = Twist()
        msg.linear.x = x * v_factor
        msg.linear.y = y * v_factor
        msg.angular.z = t * v_factor
        self._pub_controllers["base_controller"].publish(msg)
        
    def _cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def moveTo3DPose(self, destination):
        rospy.loginfo("Staggering to given destination {}".format(str(destination)))
        while not self.pose:
            print "Pose has not yet been initialized, can not move."
            rospy.sleep(2)

        locationPos = np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        destinationPos = np.array([destination.position.x,destination.position.y,destination.position.z])
        delta = destinationPos - locationPos
        # print delta

        destinationPos2d = np.array([destination.position.x,destination.position.y])

        euler = RobotInterface.eulerFromQuarternion(self.pose)
        heading = euler[2]

        # print " My heading is " + str(heading/math.pi*180) + " deg."

        locationPos2d = [self.pose.position.x,self.pose.position.y]

        # destAng = angle_between(locationPos2d,destinationPos2d)        
        destAng = RobotInterface.angle_between2d(destinationPos2d,locationPos2d)


        # print " My destination is at " + str(destAng/math.pi*180)  + " deg."

        turnAng = destAng - heading

        if turnAng > math.pi:
            turnAng -= (2*math.pi)
        if turnAng < -math.pi:
            turnAng += (2*math.pi)

        # print " I have to turn by " + str(turnAng/math.pi*180) + " deg."

        angThreshold = 0.1
        distThreshold = 0.4

        while True:
            heading = RobotInterface.eulerFromQuarternion(self.pose)[2]
            # # locationPos2d = np.array([self.pose.position.x,self.pose.position.y])        
            locationPos2d = [self.pose.position.x,self.pose.position.y]
            destAng = RobotInterface.angle_between2d(destinationPos2d,locationPos2d)

            turnAng = destAng - heading
            if turnAng > math.pi:
                turnAng -= (2*math.pi)
            if turnAng < -math.pi:
                turnAng += (2*math.pi)

            angAbs = abs(turnAng)
            if turnAng < 0 :
                turnDir = -1
            else:
                turnDir = 1
            
            turnV = angAbs / math.pi
            maxTurnV = 0.5
            minTurnV = 0.00
            turnV = max(min(math.pow(turnV,1)/2,maxTurnV),minTurnV)
            # turnV = min(angAbs,0.5)
            angV = turnV * turnDir

            distAbs = abs(np.linalg.norm(destinationPos2d - locationPos2d))

            fwdV = min(math.pow(distAbs,1.5),1)
            
            if angAbs > (math.pi/2):
                fwdV = 0
            else:
                fwdV *= (1-(angAbs/(math.pi/2)))

            if (abs(turnAng) < angThreshold):
                angV = 0

            if (abs(distAbs) < distThreshold):
                fwdV = 0

            # print "heading: " + str(heading) + " -- destAng: " + str(destAng) + " -- turnAng: " + str(turnAng) + " -- angV: " + str (angV) + " -- dist: " + str(distAbs) + " -- fwdV: " + str(fwdV)
            # break
            
            self.set_move_velocity(fwdV,0,angV)

            if (abs(turnAng) < angThreshold and abs(distAbs) < distThreshold):
                self.set_move_velocity(0,0,0)
                rospy.loginfo("Arrived at destination")
                break

            rospy.sleep(0.2)

    def moveToXY(self, x, y):
        destination = Pose()
        destination.orientation.x = 0
        destination.orientation.y = 0
        destination.orientation.z = 0
        destination.orientation.w = 0
        destination.position.x = x
        destination.position.y = y
        destination.position.z = 0
        self.moveTo3DPose(destination)

    def moveToPose(self, x, y, theta):
        destination = Pose()
        quaternion = RobotInterface.quaternion2DFromAngle(theta)
        destination.position.x = x
        destination.position.y = y
        destination.position.z = 0
        destination.orientation.x = quaternion[0]
        destination.orientation.y = quaternion[1]
        destination.orientation.z = quaternion[2]
        destination.orientation.w = quaternion[3]
        self.moveTo3DPose(self, destination)

    def spread_arms(self):
        rospy.logwarn("PR2 spread arms not implemented")
        # jl = "j_shoulder_l"
        # jr = "j_shoulder_r"
        # vl = math.pi/2
        # vr = -math.pi/2
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # rospy.sleep(2.5)
        # jl = "j_high_arm_l"
        # jr = "j_high_arm_r"
        # vl = -1.4
        # vr = -1.4
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # rospy.sleep(2.5)
        # self.set_walk_velocity(-0.1,0,0)
        # rospy.sleep(2.5)
        # self.set_walk_velocity(0.1,0,0)
        # rospy.sleep(0.5)
        # self.set_walk_velocity(0,0,0)

    def open_gripper(self):
        ra = "r_arm_controller"
        joint_values = {"r_wrist_flex_joint": -0.0,
                        "r_shoulder_lift_joint": -0.00,
                        "r_forearm_roll_joint": 0.0,
                        "r_wrist_roll_joint": 0.0,
                        "r_elbow_flex_joint": -0.0}

        self.set_joint_values(joint_values, ra, 1)

        rg = "r_gripper_controller"
        v = 0.1
        eff = 100.0
        msg = Pr2GripperCommand()
        msg.position = v
        msg.max_effort = eff
        self._pub_controllers[rg].publish(msg)
        rospy.sleep(5.6)

    def grasp(self):
        rg = "r_gripper_controller"
        v = 0.0
        eff = 15.0
        msg = Pr2GripperCommand()
        msg.position = v
        msg.max_effort = eff
        self._pub_controllers[rg].publish(msg)
        rospy.sleep(4)
        # Now raise the shoulder a little bit:
        ra = "r_arm_controller"
        joint_values = {"r_wrist_flex_joint": -0.0,
                        "r_shoulder_lift_joint": -0.001,
                        "r_forearm_roll_joint": 0.0,
                        "r_wrist_roll_joint": 0.0,
                        "r_elbow_flex_joint": -0.0}
        self.set_joint_values(joint_values, ra, 1)

    def set_joint_values(self,joint_values, controller, duration=1):
        msg = JointTrajectory()
        joints = self.joints[controller]
        msg.joint_names = joints
        point = JointTrajectoryPoint
        points = [point]
        for p in points:
            p.positions = []
            p.velocities = []
            p.accelerations = []
            p.effort = []
            point.time_from_start = rospy.Duration(duration)
            for j in joints:
                p.velocities.append(0.0)
                if j in joint_values.keys():
                    p.positions.append(joint_values[j])
                    continue
                p.positions.append(0)
        msg.points = points
        self._pub_controllers[controller].publish(msg)
        rospy.sleep(duration)

    def raise_arms(self):
        rospy.logwarn("PR2 raise arms not implemented")
        # jl = "j_shoulder_l"
        # jr = "j_shoulder_r"
        # vl = 0
        # vr = 0
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # rospy.sleep(2.5)
        # jl = "j_high_arm_l"
        # jr = "j_high_arm_r"
        # vl = -1.4
        # vr = -1.4
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # pass

    def release(self):
        rospy.logwarn("PR2 realease action not implemented")
        # jl = "j_shoulder_l"
        # jr = "j_shoulder_r"
        # vl = math.pi/2
        # vr = -math.pi/2
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # rospy.sleep(2.5)
        # jl = "j_high_arm_l"
        # jr = "j_high_arm_r"
        # vl = 0
        # vr = 0
        # self._pub_joints[jl].publish(vl)
        # self._pub_joints[jr].publish(vr)
        # self.set_walk_velocity(-0.5,0,0)
        # rospy.sleep(1.5)
        # self.set_walk_velocity(0,0,0)
        # pass

