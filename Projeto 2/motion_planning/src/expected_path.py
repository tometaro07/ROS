#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from nav_msgs.srv import GetPlan, GetPlanRequest

rospy.init_node("expected_path",anonymous=True)
tf_pose=tf.TransformListener()
rospy.Rate(10).sleep()

pub_plan=rospy.Publisher("/path_planned",Path,queue_size=1)
path=rospy.get_param("/da_wae")
waypoints=rospy.get_param("/waypoints")
service_subscriber=rospy.ServiceProxy("/move_base/NavfnROS/make_plan",GetPlan)
array=[]
n=0
if not rospy.is_shutdown():

    now=rospy.Time.now()
    while not tf_pose.canTransform("/map","/base_link",now):
        now=tf_pose.getLatestCommonTime("/map","/base_link")

    (position,orientation)=tf_pose.lookupTransform("/map","/base_link",now)

    initial_pose=Pose()
    head=Header()

    initial_pose.position.x=position[0]
    initial_pose.position.y=position[1]

    initial_pose.orientation.z=orientation[2]
    initial_pose.orientation.w=orientation[3]

    head.frame_id="map"
    head.seq=n
    head.stamp=now

    initial_pose=PoseStamped(header=head,pose=initial_pose)

    array=[]
    for point in path:
        final_pose=Pose()

        final_pose.position.x=waypoints[point]["x"]
        final_pose.position.y=waypoints[point]["y"]
        final_pose.orientation.z=waypoints[point]["yawn"]
        final_pose.orientation.w=(1-waypoints[point]["yawn"]**2)**0.5
        final_pose=PoseStamped(header=head,pose=final_pose)

        rospy.wait_for_service("/move_base/NavfnROS/make_plan")
        path=service_subscriber(GetPlanRequest(start=initial_pose,goal=final_pose,tolerance=0))

        array=array[:]+path.plan.poses[:]
        path=None

        initial_pose=final_pose

    pub_plan.publish(Path(header=head,poses=array))