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
            p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64,queue_size=1)
            self._pub_joints[j]=p
        
        rospy.sleep(1)
        
        self._pub_cmd_vel=rospy.Publisher(ns+"cmd_vel",Twist,queue_size=1)

        rospy.loginfo("Listening for commands")
        self._sub_nlu=rospy.Subscriber("/cqi/command",String,self.cb_nlu,queue_size=5)

        # self.testBeer()
        

        
    def cb_nlu(self,msg):

        print "received msg:"
        print msg
        commandName = msg.data.split("(")[0]
        commandArgs = msg.data[:-1].split("(")[1].split(",")
        self.execCommand(commandName,commandArgs)

    def execCommand(self,commandName,commandArgs):
        if commandName == "moveToXY":
            x = float(commandArgs[0])
            y = float(commandArgs[1])
            destination = Pose()    
            destination.position.x = x
            destination.position.y = y
            self.walkToPose(destination)
        

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

    def walkToPose(self,destination):
        rospy.loginfo("Staggering to given destination")
        # print " I am at "
        # print self.pose
        # print " I must go to "
        # print destination
        
        # print " I have to go this way:"

        locationPos = np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        destinationPos = np.array([destination.position.x,destination.position.y,destination.position.z])
        delta = destinationPos - locationPos
        # print delta

        destinationPos2d = np.array([destination.position.x,destination.position.y])

        euler = eulerFromQuarternion(self.pose)
        heading = euler[2]

        # print " My heading is " + str(heading/math.pi*180) + " deg."

        locationPos2d = [self.pose.position.x,self.pose.position.y]

        # destAng = angle_between(locationPos2d,destinationPos2d)        
        destAng = angle_between2d(destinationPos2d,locationPos2d)


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
            heading = eulerFromQuarternion(self.pose)[2]
            # # locationPos2d = np.array([self.pose.position.x,self.pose.position.y])        
            locationPos2d = [self.pose.position.x,self.pose.position.y]
            destAng = angle_between2d(destinationPos2d,locationPos2d)

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
            
            self.set_walk_velocity(fwdV,0,angV)

            if (abs(turnAng) < angThreshold and abs(distAbs) < distThreshold):
                self.set_walk_velocity(0,0,0)
                rospy.loginfo("Arrived at destination")
                break

            rospy.sleep(0.2)

        # print delta

        
    def _cb_modeldata(self,msg):
        # print "model data:"
        modelLocations = {}
        for pos,item in enumerate(msg.name):            
            if item != "darwin":
                continue
            self.pose = msg.pose[pos]  

    def testBeer(self):

        destination = Pose()    
        destination.orientation.x = 1
        destination.orientation.y = 1
        destination.orientation.z = 1
        destination.orientation.w = 1
        destination.position.x = 1
        destination.position.y = 1
        destination.position.z = 1
        rospy.loginfo("Instantiating Model Location Interface Client")
        mli = ModelLocationInterface()
        rospy.sleep(1)
        for item,pose in mli.modelPoses.items():
            print item
            if item == "beer":
                destination = pose
                print "There is beer!!!"

        self.walkToPose(destination) 

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

def angle_between2d(a,b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    rads = math.atan2(dy,dx)
    # rads %= 2*math.pi
    return rads

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
