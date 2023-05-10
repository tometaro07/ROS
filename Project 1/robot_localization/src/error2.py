#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseArray,Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import numpy as np


def quaternion_to_euler(x,y,z,w):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def callback(data):
    global position2,orientation2,covariance

    position2=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z]
    orientation2=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    covariance=data.pose.covariance

position2=orientation2=covariance=None

rospy.init_node("error")
tf_pose=tf.TransformListener()
sub=rospy.Subscriber("/odometry/filtered_odom",Odometry,callback).callback
rospy.Rate(10).sleep()

n=0
while not rospy.is_shutdown():
    
    now1=rospy.Time.now()
    while not tf_pose.canTransform("/map","/mocap_laser_link",now1):
        now1=tf_pose.getLatestCommonTime("/map","/mocap_laser_link")

    (position1,orientation1)=tf_pose.lookupTransform("/map","/mocap_laser_link",now1)

    pose1=Pose()
    pose2=Pose()
    pose3=Pose()
    head=Header()

    sub

    pose1.position.x=position1[0]-position2[0]
    pose2.position.x=pose1.position.x+covariance[0]
    pose3.position.x=pose1.position.x-covariance[0]

    pose1.position.y=position1[1]-position2[1]
    pose2.position.y=pose1.position.y+covariance[7]
    pose3.position.y=pose1.position.y-covariance[7]



    angle1=quaternion_to_euler(orientation1[0],orientation1[1],orientation1[2],orientation1[3])[2]
    angle2=quaternion_to_euler(orientation2[0],orientation2[1],orientation2[2],orientation2[3])[2]
    if angle2*angle1<0 and abs(angle1)>math.pi/2 and abs(angle2)>math.pi/2:
        if angle1<0:
            angle1=angle1+2*math.pi
        else:
            angle1=angle1-2*math.pi

    pose1.orientation.z=angle2-angle1
    pose2.orientation.z=pose1.orientation.z+covariance[35]
    pose3.orientation.z=pose1.orientation.z-covariance[35]

    head.frame_id="map"
    head.seq=n
    head.stamp=now1

    array=[pose1,pose2,pose3]
    rospy.Publisher("/error_data",PoseArray,queue_size=1).publish(PoseArray(header=head,poses=array))

    rospy.Rate(30).sleep()