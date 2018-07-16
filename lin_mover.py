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
MAX_ERROR = 0.1

def DISTANCE_SPEED_MAP(val):
    if val < 0.1 : 
        return 0.02
    if val < 0.2: 
        return 0.05
    if val < 0.5: 
        return 0.1
    return 0.14

def controllerLoop(event):
    global controller_done
    if controller_done:
        return
    #position = last_position.pose.position
    #position = np.array([position.x, position.y, 0])
    position = np.array([0,0,0]) # we're using the base frame as reference frame
    map_vel = getVelocity()
    tgt = getTarget()
    error = estimateTargetDeviation(position, map_vel, tgt)
    print("estimated error: " + str(error))
    print("error/distance: " + str(error/getTargetDistance(position, tgt)))
    if getTargetDistance(position, tgt) < MAX_ERROR:
        print("reached target. stopping")
        controller_done = True
        sendMovement([0,0,0])
        return
    if np.linalg.norm(map_vel) == 0 or error > MAX_ERROR and error/getTargetDistance(position, tgt)<0.5:
        print("error to large. Updating movement command:")
        mvmt = computeMovement(position, tgt)
        sendMovement(mvmt)
        print(mvmt)
    if np.linalg.norm(map_vel) > DISTANCE_SPEED_MAP(getTargetDistance(position, tgt)):
        print("Exceeded speed limit. Updating cmd_vel:")
        mvmt = computeMovement(position, tgt)
        sendMovement(mvmt)
        print(mvmt)
    pass

def computeMovement(position, tgt):
    direction = position - tgt
    distance = getTargetDistance(position, tgt)
    #yaw = getBaseYaw()
    #direction = np.array([direction[0]*cos(yaw)-direction[0]*sin(yaw),direction[1]*sin(yaw)+direction[1]*cos(yaw),0]) #rotate to base frame
    direction = np.array([direction[0],direction[1],0])
    vel = direction/np.linalg.norm(direction)*DISTANCE_SPEED_MAP(distance)
    return vel

def sendMovement(move):
    mvmt = Twist()
    mvmt.linear = Vector3(move[0], move[1], 0)
    velPub.publish(mvmt)
def getTargetDistance(point, target):
    return np.linalg.norm(target-point)

def estimateTargetDeviation(position, v, target):
    distance = np.linalg.norm(np.cross(position-target, v))/np.linalg.norm(v)
    return distance

def getBaseYaw():
    transf = tfBuffer.lookup_transform(MAP_FRAME, BASE_FRAME, rospy.Time(), timeout=rospy.Duration(2))
    quat = transf.transform.rotation
    euler = tf.transformations.euler_from_quaternion(np.array([quat.x, quat.y, quat.z, quat.w]))
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return yaw

def getVelocity(): # in robot frame
    #transf = tfBuffer.lookup_transform(MAP_FRAME, BASE_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    v = Vector3Stamped()
    v.vector = last_cmdvel.linear
    v.header.frame_id = BASE_FRAME
    #vel = tf2_geometry_msgs.do_transform_vector3(v, transf)
    vel = np.array([v.vector.x, v.vector.y, 0])
    return vel
def getTarget(): #in robot base frame
    transf = tfBuffer.lookup_transform(BASE_FRAME, target.header.frame_id, rospy.Time(),timeout=rospy.Duration(2))
    tgt = tf2_geometry_msgs.do_transform_point(target, transf)
    return np.array([tgt.point.x, tgt.point.y, 0])

def targetUpdate(data):
    global target
    target = data
    global controller_done
    controller_done = False
    print("received new target: (map_frame)"+str(target))
    pass
def positionUpdate(data):
    global last_position
    last_position = data
    pass
def velocityUpdate(data):
    global last_cmdvel
    last_cmdvel = data
    pass

global last_cmdvel
last_cmdvel = Twist()
global tfBuffer
global velPub
global last_position
global target
global controller_done
controller_done = True
last_position = PoseStamped()
target = np.array([0,0,0])
rospy.init_node('map_cartesian_mover')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
rospy.Subscriber("/direct_move/target", PointStamped, targetUpdate)
rospy.Subscriber("/slam_out_pose", PoseStamped, positionUpdate)
rospy.Subscriber("/cmd_vel", Twist, velocityUpdate)
rospy.Timer(rospy.Duration(UPDATE_PERIOD), controllerLoop)
rospy.spin()
