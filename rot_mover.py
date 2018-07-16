#!/bin/python
import numpy as np
import math
from math import sin, cos, pi
import rospy
import tf
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped, TransformStamped, PoseStamped, PointStamped
import struct
import tf2_ros
import tf2_geometry_msgs


BASE_FRAME= "base_footprint"
MAP_FRAME="map"
UPDATE_PERIOD = 0.5
MAX_ERROR = 0.05

def YAW_SPEED_MAP(val):
    val=np.linalg.norm(val)
    if val < 0.2 : 
        return 0.05
    if val < 0.4: 
        return 0.09
    if val < 0.6: 
        return 0.12
    return 0.18

def controllerLoop(event):
    global controller_done
    if controller_done:
        return
    vel = getVelocity() #Yaw, Pitch, Roll
    tgt = getTargetYaw()
    error = np.linalg.norm(tgt[0])
    print(np.linalg.norm(tgt[0]))
    if np.linalg.norm(tgt[0]) < MAX_ERROR:
        print("reached target. Stopping..")
        sendMovement(0)
        controller_done = True
        return
    if vel[0] == 0:
        print("error to large. Updating movement command:")
        mvmt = computeMovement(tgt)
        sendMovement(mvmt)
        print(mvmt)
    if np.linalg.norm(vel[0]) > YAW_SPEED_MAP(getTargetDistanceRad(tgt)) or vel[0]*tgt[0] < 0 : #last condition catches movement in the wrong direction
        print("Exceeded speed limit. Updating cmd_vel:")
        mvmt = computeMovement(tgt)
        sendMovement(mvmt)
        print(mvmt)
    pass

def computeMovement(targetYaw):
    distance = targetYaw[0]
    vel = YAW_SPEED_MAP(distance)*distance/np.linalg.norm(distance) # diretion
    return vel
def sendMovement(move):
    mvmt = Twist()
    mvmt.angular = Vector3(0, 0, move)
    velPub.publish(mvmt)
def getTargetDistanceRad(tgt):
    return tgt[0]

def getVelocity(): # in robot frame
    #transf = tfBuffer.lookup_transform(MAP_FRAME, BASE_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    v = Vector3Stamped()
    v.vector = last_cmdvel.angular
    v.header.frame_id = BASE_FRAME
    #vel = tf2_geometry_msgs.do_transform_vector3(v, transf)
    vel = np.array([v.vector.z, 0, 0])
    return vel
def getTargetYaw(): #in robot base frame
    transf = tfBuffer.lookup_transform(BASE_FRAME, MAP_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    tgt = tf2_geometry_msgs.do_transform_pose(target, transf)
    quaternion = [tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return np.array([yaw, 0, 0])

def targetUpdate(data): # store target in static map frame
    transf = tfBuffer.lookup_transform(MAP_FRAME, data.header.frame_id, data.header.stamp, timeout=rospy.Duration(2))
    data = tf2_geometry_msgs.do_transform_pose(data, transf)
    global target
    target = data
    global controller_done
    controller_done = False
    print("received new target: (map_frame)"+str(target))
    print("euler: "+str(getTargetYaw()))
    pass
def velocityUpdate(data):
    global last_cmdvel
    last_cmdvel = data
    pass

global last_cmdvel
last_cmdvel = Twist()
global tfBuffer
global velPub
global target
global controller_done
controller_done = True
rospy.init_node('map_cartesian_mover')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
rospy.Subscriber("/direct_move/rot_target", PoseStamped, targetUpdate)
rospy.Subscriber("/cmd_vel", Twist, velocityUpdate)
rospy.Timer(rospy.Duration(UPDATE_PERIOD), controllerLoop)
rospy.spin()
