#!/usr/bin/python
import math
from math import sin, cos, pi
import rospy
import tf
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
import struct
import tf2_ros
import tf2_geometry_msgs
import numpy as np
BASE_FRAME= "base_footprint"
UPDATE_PERIOD = 0.1
global speed_pub
global direction_pub
global go_pub
global tfBuffer
global last_odom
global target_point
last_odom = Odometry()
target_point = None
global speed
global direction
speed=0
direction=0.0

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
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def callback_odom(data):
    global last_odom
    last_odom = data
def callback_speed(data):
    global speed
    speed = data.data
def callback_direction(data):
    global direction
    direction = float(data.data)/180.0*pi
def callback_target(data):
    global target_point
    target_point = data

def my_callback(event):
    global speed
    global target_point
    if target_point is None:
        return
    try:
        if speed == 0:
            speed = 50
            speed_pub.publish(Int16(15))
            go_pub.publish(Int16(1))
            print("starting movement towards target")
        target = np.array([target_point.x, target_point.y, target_point.z])
        position = np.array([last_odom.pose.pose.position.x,last_odom.pose.pose.position.y,last_odom.pose.pose.position.z])
        quaternion = last_odom.pose.pose.orientation
        quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        distance = np.linalg.norm(target-position)
        velocity = np.array([speed*cos(direction+yaw), speed*sin(direction+yaw), 0])
        if speed != 0:
            projected_mvmt = position + distance/np.linalg.norm(velocity) * velocity
        else:
            projected_mvmt = np.array([0,0,0])
        error = np.linalg.norm(target-projected_mvmt)
        print("projected: " + str(projected_mvmt))
        print("estimated position: " + str(position))
        print("target: "+str(target))
        print("velocity"+str(velocity))
        print("error/distance: " + str(error/distance))
        if distance < 0.05:
            print("reached target")
            target_point = None
            go_pub.publish(Int16(0))
        elif speed > 10 and distance < 0.1:
            print("getting close to target. reducing speed")
            speed_pub.publish(Int16(10))
        elif speed > 25 and distance < 0.25:
            print("getting close to target. reducing speed")
            speed_pub.publish(Int16(25))
        if error/distance > 0.15:
            print("adjusting movement")
            new_direction = direction + angle_between(target-position, velocity)
            new_direction = new_direction/pi*180
            print(new_direction)
            direction_pub.publish(Int16(int(new_direction)))
    except Exception as e:
        print(e)
        pass

rospy.init_node('simple_rel_mover')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
r = rospy.Rate(1) # 10hz
speed_pub = rospy.Publisher("/platform/speed", Int16, queue_size=3)
direction_pub = rospy.Publisher("/platform/direction", Int16, queue_size=3)
go_pub = rospy.Publisher("/platform/go", Int16, queue_size=3)
rospy.Timer(rospy.Duration(1), my_callback)
rospy.Subscriber("/odom", Odometry, callback_odom)
rospy.Subscriber("/target", Point, callback_target)
rospy.Subscriber("/platform/direction", Int16, callback_direction)
rospy.Subscriber("/platform/speed", Int16, callback_speed)
rospy.spin()
