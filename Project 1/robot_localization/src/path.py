#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

rospy.init_node("path",anonymous=True)
tf_pose=tf.TransformListener()
frame=rospy.get_param('~frame_of_pose',"/base_link")
decay=rospy.get_param('~path_decay',True)
decay_len=rospy.get_param('~path_decay_len',100)

pub_estimate=rospy.Publisher("/path_walked_by_"+frame[1:],Path,queue_size=1)

array=[]
n=0

rospy.Rate(0.1).sleep()
while not rospy.is_shutdown():
    
    now=rospy.Time.now()
    while not tf_pose.canTransform("/map",frame,now):
        now=tf_pose.getLatestCommonTime("/map",frame)

    (position,orientation)=tf_pose.lookupTransform("/map",frame,now)

    pose=Pose()
    head=Header()

    pose.position.x=position[0]
    pose.position.y=position[1]

    pose.orientation.z=orientation[2]
    pose.orientation.w=orientation[3]

    
    head.frame_id="map"
    head.seq=n
    head.stamp=now
    array+=[PoseStamped(header=head,pose=pose)]
    if decay and len(array)>decay_len:
        array.pop(0)
    pub_estimate.publish(Path(header=head,poses=array))

    rospy.Rate(30).sleep()