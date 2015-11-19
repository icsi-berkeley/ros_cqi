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

class DarwinInterface:
    """
    Client ROS class for manipulating Darwin OP in Gazebo
    """
    
    def __init__(self,ns="/darwin/"):
        self.ns=ns
        self.joints=None
        self.angles=None
        self._sub_model=rospy.Subscriber("gazebo/model_states",ModelStates,self._cb_modeldata,queue_size=1)
        self._sub_joints=rospy.Subscriber(ns+"joint_states",JointState,self._cb_joints,queue_size=1)
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
            p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64)
            self._pub_joints[j]=p
        
        rospy.sleep(1)
        
        self._pub_cmd_vel=rospy.Publisher(ns+"cmd_vel",Twist)
        

    def set_walk_velocity(self,x,y,t):
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=t
        self._pub_cmd_vel.publish(msg)
        
    def _cb_joints(self,msg):
        if self.joints is None:
            self.joints=msg.name
        self.angles=msg.position
        
    
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
            angles=interpolate(stop_angles,start_angles,ratio)                        
            self.set_angles(angles)
            r.sleep()

    def walkToPose(self,destination=Pose()):
        print " I am at "
        print self.pose
        print " I must go to "
        print destination
        print " I have to go this way:"

        locationPos = np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        destinationPos = np.array([destination.position.x,destination.position.y,destination.position.z])
        delta = destinationPos - locationPos
        print delta

        locationPos2d = np.array([self.pose.position.x,self.pose.position.y])
        destinationPos2d = np.array([destination.position.x,destination.position.y])
        delta = destinationPo2d - locationPos2d

            

        
    def _cb_modeldata(self,msg):
        # print "model data:"
        modelLocations = {}
        for pos,item in enumerate(msg.name):            
            if item != "darwin":
                continue
            self.pose = msg.pose[pos]   

def interpolate(anglesa,anglesb,coefa):
    z={}
    joints=anglesa.keys()
    for j in joints:
        z[j]=anglesa[j]*coefa+anglesb[j]*(1-coefa)
    return z

def get_distance(anglesa,anglesb):
    d=0
    joints=anglesa.keys()
    if len(joints)==0: return 0
    for j in joints:
        d+=abs(anglesb[j]-anglesa[j])
    d/=len(joints)
    return d

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))
    if np.isnan(angle):
        if (v1_u == v2_u).all():
            return 0.0
        else:
            return np.pi
    return angle