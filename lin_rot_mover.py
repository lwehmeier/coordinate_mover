#!/usr/bin/python
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
UPDATE_PERIOD = 0.1
MAX_ERROR = 0.01
MAX_YAW_ERROR = 0.03
def DISTANCE_SPEED_MAP(val):
    if val < 0.05:
        return 0.015
    if val < 0.1 : 
        return 0.02
    if val < 0.2: 
        return 0.05
    if val < 0.5: 
        return 0.1
    return 0.14
def YAW_SPEED_MAP(val):
    val=np.linalg.norm(val)
    if val < 0.25 :
        return 0.02
    if val < 0.4:
        return 0.09
    if val < 0.65:
        return 0.12
    return 0.18

def controllerLoop(event):
    global controller_done
    if controller_done:
        return
    #position = last_position.pose.position
    #position = np.array([position.x, position.y, 0])
    position = np.array([0,0,0]) # we're using the base frame as reference frame
    map_vel = getVelocity()
    tgt = getTarget()
    print("target"+str(tgt))
    error = estimateTargetDeviation(position, map_vel, tgt)
    print("estimated error: " + str(error))
    print("error/distance: " + str(error/getTargetDistance(position, tgt)))
    yaw = getTargetYaw()
    mvmt = np.array([0,0,0])
    if getTargetDistance(position, tgt) < MAX_ERROR and np.linalg.norm(yaw) < MAX_YAW_ERROR:
        print("reached target. stopping")
        controller_done = True
        sendMovement([0,0,0])
        return
    if np.linalg.norm(map_vel) == 0 or error > MAX_ERROR and error/getTargetDistance(position, tgt)>0.15:
        print("error to large. Updating movement command:")
        mvmt = mvmt + computeMovementLin(position, tgt) + computeMovementRot()
        print(mvmt)
        sendMovement(mvmt)
    if np.linalg.norm(map_vel) > DISTANCE_SPEED_MAP(getTargetDistance(position, tgt)) or np.linalg.norm(getYawSpeed()) > YAW_SPEED_MAP(np.linalg.norm(getYawSpeed())):
        print("Exceeded speed limit. Updating cmd_vel:")
        mvmt = computeMovementLin(position, tgt) + computeMovementRot()
        sendMovement(mvmt)
        print(mvmt)
    pass

def computeMovementLin(position, tgt):
    if getTargetDistance(np.array([0,0,0]), getTarget()) < MAX_ERROR:
        return np.array([0,0,0])
    direction = tgt - position
    distance = getTargetDistance(position, tgt)
    direction = np.array([direction[0],direction[1],0])
    vel = direction/np.linalg.norm(direction)*DISTANCE_SPEED_MAP(distance)
    return vel
def computeMovementRot():
    if np.linalg.norm(getTargetYaw()[2])<MAX_YAW_ERROR:
        return np.array([0,0,0])
    vel = np.array([0,0,0])
    yaw = getTargetYaw()[2]
    vel[2] = YAW_SPEED_MAP(np.linalg.norm(yaw))*yaw/np.linalg.norm(yaw) #direction
    return vel

def sendMovement(move):
    mvmt = Twist()
    mvmt.linear = Vector3(move[0], move[1], 0)
    mvmt.angular = Vector3(0,0,move[2])
    velPub.publish(mvmt)
def getTargetDistance(point, target):
    return np.linalg.norm(target-point)

def estimateTargetDeviation(position, v, target):
    if v[0] != 0.0 or v[1] != 0.0 and position.any() != 0:
        distance = np.linalg.norm(np.cross(position-target, v))/np.linalg.norm(v)
    else:
        distance = np.linalg.norm(position-target)
    return distance

def getTargetYaw(): #in robot base frame
    transf = tfBuffer.lookup_transform(BASE_FRAME, MAP_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    tgt = tf2_geometry_msgs.do_transform_pose(target, transf)
    quaternion = [tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return np.array([yaw, 0, 0])
def getVelocity(): # in robot frame
    #transf = tfBuffer.lookup_transform(MAP_FRAME, BASE_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    v = Vector3Stamped()
    v.vector = last_cmdvel.linear
    v.header.frame_id = BASE_FRAME
    #vel = tf2_geometry_msgs.do_transform_vector3(v, transf)
    vel = np.array([v.vector.x, v.vector.y, 0])
    return vel
def getYawSpeed():
    return last_cmdvel.angular.z
def getTarget(): #in robot base frame
    transf = tfBuffer.lookup_transform(BASE_FRAME, MAP_FRAME, rospy.Time(),timeout=rospy.Duration(2))
    print(transf)
    tgt = tf2_geometry_msgs.do_transform_pose(target, transf)
    return np.array([tgt.pose.position.x, tgt.pose.position.y, 0])

def targetUpdate(data): #transforms target to map frame as that is the only frame guaranteed to be static
    global target
    transf = tfBuffer.lookup_transform(MAP_FRAME, data.header.frame_id, data.header.stamp, timeout=rospy.Duration(2))
    data = tf2_geometry_msgs.do_transform_pose(data, transf)
    target = data
    target.header.frame_id=MAP_FRAME
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
rospy.init_node('cartesian_mover_lin_rot')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/direct_move/lin_rot_target", PoseStamped, targetUpdate)
rospy.Subscriber("/slam_out_pose", PoseStamped, positionUpdate)
rospy.Subscriber("/cmd_vel", Twist, velocityUpdate)
rospy.Timer(rospy.Duration(UPDATE_PERIOD), controllerLoop)
rospy.Rate(1/UPDATE_PERIOD)
rospy.spin()
