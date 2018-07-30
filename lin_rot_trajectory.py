#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, PoseArray

def nearTargetCB(data):
    global trajectory
    print("near current waypoint")
    if data.data and len(trajectory) > 0:
        print("setting next target")
        posPub.publish(trajectory[0])
        trajectory = trajectory[1:]

def trajectoryCB(data):
    print("received new trajectory: ")
    print(data)
    global trajectory
    trajectory = []
    for pose in data.poses:
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = data.header.frame_id
        trajectory.append(ps)
    posPub.publish(trajectory[0])
    print(trajectory[0])
    trajectory = trajectory[1:]
def targetAbort(msg):
    if msg.data:
        global trajectory
        trajectory = []
if __name__ == "__main__":
   rospy.init_node("lin_rot_trajectory") 
   rospy.Subscriber("/direct_move/near_target", Bool, nearTargetCB)
   rospy.Subscriber("/direct_move/trajectory", PoseArray, trajectoryCB)
   posPub = rospy.Publisher("/direct_move/lin_rot_target", PoseStamped, queue_size=1)
   rospy.Subscriber("/direct_move/abort", Bool, targetAbort)
   rospy.spin()
