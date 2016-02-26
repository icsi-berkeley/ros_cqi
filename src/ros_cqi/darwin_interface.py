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
from ros_cqi.robot_interface import RobotInterface
import os


class DarwinInterface(RobotInterface, object):
    """
    Client ROS class for manipulating Darwin OP in Gazebo
    """
    
    def __init__(self):
        super(DarwinInterface, self).__init__()
        self.name = "darwin"
        self.possible_comands = {"move_to_xy": ["x","y"],
                                 "move_to_pose": ["x","y","ang(rad)"],
                                 "grasp_object": ["label"],
                                 "grasp": [],
                                 "release": [],
                                 "raise_arms": [],
                                 "spread_arms": []
                                 }
        self.pose = Pose()
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None: break
            rospy.sleep(0.1)            
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        
        
        rospy.loginfo("Creating joint command publishers")
        self._pub_joints={}
        for j in self.joints:
            p=rospy.Publisher("/darwin/"+j+"_position_controller/command",Float64,queue_size=1)
            self._pub_joints[j]=p

        self._pub_cmd_vel = rospy.Publisher("/darwin/cmd_vel", Twist, queue_size=1)

    def _subscribe_joints(self):
        self._sub_joints = rospy.Subscriber("/darwin/joint_states", JointState, self._cb_joints, queue_size=1)

    def set_walk_velocity(self, x, y, t):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        print "Set move velocity to x: {} y: {}, theta:{}".format(x, y, t)
        self._pub_cmd_vel.publish(msg)
        
    def _cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None: return None
        if self.angles is None: return None
        return dict(zip(self.joints,self.angles))

    def set_angles(self,angles):
        for j,v in angles.items():
            if j not in self.joints:
                rospy.logerror("Invalid joint name "+j)
                continue
            self._pub_joints[j].publish(v)

    def set_angles_slow(self,stop_angles,delay=2):
        start_angles=self.get_angles()
        start=time.time()
        stop=start+delay
        r=rospy.Rate(100)
        while not rospy.is_shutdown():
            t=time.time()
            if t>stop: break
            ratio=(t-start)/delay            
            angles=RobotInterface.interpolate(stop_angles,start_angles,ratio)
            self.set_angles(angles)
            r.sleep()

    def move_to_xy(self, x, y):
        destination = Pose()
        destination.orientation.x = 0
        destination.orientation.y = 0
        destination.orientation.z = 0
        destination.orientation.w = 0
        destination.position.x = x
        destination.position.y = y
        destination.position.z = 0
        self.walkTo3DPose(destination)

    def move_to_pose(self, x, y, theta):
        destination = Pose()
        quaternion = RobotInterface.quaternion2DFromAngle(theta)
        destination.position.x = x
        destination.position.y = y
        destination.position.z = 0
        destination.orientation.x = quaternion[0]
        destination.orientation.y = quaternion[1]
        destination.orientation.z = quaternion[2]
        destination.orientation.w = quaternion[3]
        self.walkTo3DPose(self, destination)

    def walkTo3DPose(self, destination):
        rospy.loginfo("Staggering to given destination {}".format(str(destination)))
        while not self.pose:
            print "Pose has not yet been initialized, can not move."
            rospy.sleep(2)

        # locationPos = np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        # destinationPos = np.array([destination.position.x,destination.position.y,destination.position.z])
        # delta = destinationPos - locationPos
        # print delta

        destinationPos2d = np.array([destination.position.x,destination.position.y])
        euler = RobotInterface.eulerFromQuarternion(self.pose)
        heading = euler[2]

        locationPos2d = [self.pose.position.x,self.pose.position.y]

        destAng = RobotInterface.angle_between2d(destinationPos2d,locationPos2d)
        turnAng = destAng - heading

        if turnAng > math.pi:
            turnAng -= (2*math.pi)
        if turnAng < -math.pi:
            turnAng += (2*math.pi)

        angThreshold = 0.15
        distThreshold = 0.4
        maxTurnV = 0.2
        minTurnV = 0.00
        maxFwdV = 1
        minFwdV = 0

        while True:
            heading = RobotInterface.eulerFromQuarternion(self.pose)[2]
            locationPos2d = [self.pose.position.x,self.pose.position.y]
            destAng = RobotInterface.angle_between2d(destinationPos2d,locationPos2d)

            # Determine turning speed
            turnAng = destAng - heading
            if turnAng > math.pi:
                turnAng -= (2*math.pi)
            if turnAng < -math.pi:
                turnAng += (2*math.pi)

            angAbs = abs(turnAng)
            if turnAng < 0:
                turnDir = -1
            else:
                turnDir = 1
            
            turnV = angAbs / math.pi * 3
            turnV = max(min(math.pow(turnV, 2), maxTurnV), minTurnV)
            angV = turnV * turnDir

            # Determine forward speed
            distAbs = abs(np.linalg.norm(destinationPos2d - locationPos2d))
            fwdV = min(math.pow(distAbs, 1.5), 1)
            fwdV *= (1 - turnV/maxTurnV)
            fwdV = max(min(fwdV, maxFwdV), minFwdV)
            
            # if angAbs > (math.pi/8):
            #     fwdV = 0
            # else:
            # fwdV *= (1-(angAbs/(math.pi/2)))

            # if (abs(turnAng) < angThreshold):
            #     angV = 0

            if (abs(distAbs) < distThreshold):
                fwdV = 0
            
            self.set_walk_velocity(fwdV, 0, angV)

            if (abs(turnAng) < angThreshold and abs(distAbs) < distThreshold):
                self.set_walk_velocity(0, 0, 0)
                rospy.loginfo("Arrived at destination")
                break

            rospy.sleep(0.2)

    def grasp(self):
        jr = "j_gripper_r"
        vr = -math.pi/2
        angles = {}
        angles[jr] = vr
        duration = 0.1
        self.set_angles_slow(angles,delay=duration)
        rospy.sleep(duration)

    def open_gripper(self):
        jr = "j_gripper_r"
        vr = 0
        angles = {}
        angles[jr] = vr
        duration = 1
        self.set_angles_slow(angles, delay=duration)
        rospy.sleep(duration)

    def get_on_ground(self):
        angles = {}
        # Knees
        angles["j_tibia_l"] = math.radians(-130)
        angles["j_tibia_r"] = math.radians(130)
        # Feet pitch angles
        angles["j_ankle1_l"] = math.radians(-80)
        angles["j_ankle1_r"] = math.radians(80)
        # Hip pitch angles
        angles["j_thigh2_l"] = math.radians(50)
        angles["j_thigh2_r"] = math.radians(-50)

        self.set_angles_slow(angles, delay=2)
        rospy.sleep(1)
        # Feet pitch angles
        angles["j_ankle1_l"] = math.radians(-70)
        angles["j_ankle1_r"] = math.radians(70)
        # Knees
        angles["j_tibia_l"] = math.radians(-100)
        angles["j_tibia_r"] = math.radians(100)
        # Hips
        angles["j_thigh2_l"] = math.radians(90)
        angles["j_thigh2_r"] = math.radians(-90)

        self.set_angles_slow(angles, delay=8)
        rospy.sleep(5)

    def stand_up(self):
        angles = {}

        # # Feet
        # angles["j_ankle1_l"] = math.radians(-70)
        # angles["j_ankle1_r"] = math.radians(70)
        #
        # # Knees
        # angles["j_tibia_l"] = math.radians(-130)
        # angles["j_tibia_r"] = math.radians(130)
        #
        # # angles["j_low_arm_l"] = math.radians(-45)
        # # angles["j_low_arm_r"] = math.radians(45)
        #
        # self.set_angles_slow(angles, delay=0.1)
        #
        # rospy.sleep(0.2)
        # Feet
        angles["j_ankle1_l"] = math.radians(-60)
        angles["j_ankle1_r"] = math.radians(60)
        # Knees
        angles["j_tibia_l"] = math.radians(-150)
        angles["j_tibia_r"] = math.radians(150)

        # Hips
        angles["j_thigh2_l"] = math.radians(80)
        angles["j_thigh2_r"] = math.radians(-80)

        self.set_angles_slow(angles, delay=6.5)

        rospy.sleep(1)
        # Hips
        angles["j_tibia_l"] = math.radians(0)
        angles["j_tibia_r"] = math.radians(0)

        # Feet
        angles["j_ankle1_l"] = math.radians(-15)
        angles["j_ankle1_r"] = math.radians(15)

        # Hip
        angles["j_thigh2_l"] = math.radians(0)
        angles["j_thigh2_r"] = math.radians(0)

        self.set_angles_slow(angles, delay=1.8)

        rospy.sleep(0.5)
        # Feet
        # angles["j_ankle1_l"] = math.radians(10)
        # angles["j_ankle1_r"] = math.radians(-10)
        # self.set_angles_slow(angles, delay=1.8)
        #
        # rospy.sleep(1)
        # Feet
        angles["j_ankle1_l"] = math.radians(0)
        angles["j_ankle1_r"] = math.radians(0)
        self.set_angles_slow(angles, delay=1.8)


    def grasp_object(self, label):
        self.open_gripper()
        x = 0.06
        y = -0.001
        z = -0.1
        w = 1
        offset = RobotInterface.define_pose(x, y, z, 0, 0, 0.0, w)
        self.lower_arms()
        self.get_on_ground()
        rospy.sleep(4)
        self.add_object_link('darwin::MP_ARM_GRIPPER_FIX_R', label, offset)
        self.enforce_object_links(link_update_frequency=0.0)
        self.grasp()
        self.enforce_object_links(link_update_frequency=0.0)
        self.stand_up()
        rospy.sleep(5)

    def grasp_both_arms(self):
        self.spread_arms()
        angles = {}
        duration = 1
        angles["j_shoulder_l"] = math.pi/2
        angles["j_shoulder_r"] = -math.pi/2
        self.set_angles_slow(angles,delay=duration)
        angles["j_high_arm_l"] = -1.4
        angles["j_high_arm_r"] = -1.4
        self.set_angles_slow(angles,delay=duration)
        rospy.sleep(duration)
        self.set_walk_velocity(-0.1,0,0)
        rospy.sleep(2.5)
        self.set_walk_velocity(0.1,0,0)
        rospy.sleep(0.5)
        self.set_walk_velocity(0,0,0)

    def lower_arms(self):
        angles = {}
        duration = 1
        jl = "j_shoulder_l"
        jr = "j_shoulder_r"
        vl = -1.3
        vr = 1.3
        angles[jl] = vl
        angles[jr] = vr
        jl = "j_high_arm_l"
        jr = "j_high_arm_r"
        vl = 1.4
        vr = 1.4
        angles[jl] = vl
        angles[jr] = vr
        jl = "j_low_arm_l"
        jr = "j_low_arm_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr
        jl = "j_wrist_l"
        jr = "j_wrist_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr
        self.set_angles_slow(angles,delay=duration)

    def raise_arms(self):
        angles = {}
        duration = 1
        jl = "j_shoulder_l"
        jr = "j_shoulder_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr
        jl = "j_high_arm_l"
        jr = "j_high_arm_r"
        vl = -1.4
        vr = -1.4
        angles[jl] = vl
        angles[jr] = vr
        self.set_angles_slow(angles,delay=duration)

    def init_pose(self):
        angles = {}
        # Knees
        angles["j_tibia_l"] = math.radians(0)
        angles["j_tibia_r"] = math.radians(0)
        # Feet pitch angles
        angles["j_ankle1_l"] = math.radians(0)
        angles["j_ankle1_r"] = math.radians(0)
        # Feet roll angles
        angles["j_ankle2_l"] =math.radians(0)
        angles["j_ankle2_r"] = math.radians(0)
        # Hip yaw angles
        angles["j_thigh1_l"] = math.radians(0)
        angles["j_thigh1_r"] = math.radians(0)
        # Hip pitch angles
        angles["j_thigh2_l"] = math.radians(0)
        angles["j_thigh2_r"] = math.radians(0)
        self.set_angles_slow(angles, delay=2)

    def spread_arms(self):
        angles = {}
        duration = 1
        jl = "j_shoulder_l"
        jr = "j_shoulder_r"
        vl = math.pi/2
        vr = -math.pi/2
        angles[jl] = vl
        angles[jr] = vr

        jl = "j_high_arm_l"
        jr = "j_high_arm_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr

        jl = "j_low_arm_l"
        jr = "j_low_arm_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr

        jl = "j_wrist_l"
        jr = "j_wrist_r"
        vl = 0
        vr = 0
        angles[jl] = vl
        angles[jr] = vr
        self.set_angles_slow(angles,delay=duration)

    def release(self):
        self.open_gripper()
        for l in self.linked_objects:
            if l[0] == 'darwin::MP_ARM_GRIPPER_FIX_R':
                self.release_object_link(l[0], l[1])
        # pass

