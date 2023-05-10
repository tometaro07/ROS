#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib

from math import sqrt

class My_way(object):
    def __init__(self):
        rospy.init_node("we_know_da_wae")
                
        self.waiting_list = []
        self.goal = None
        
        
        self.sequence=0
        self.goal_achieved = False
        self.task_concluded = False
        self.problemo = False
        self.pose=None


        # Create an action client called "almost_there" with action definition file "MoveBaseAction"
        self.client_goals=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tf_pose=tf.TransformListener()


        self.rate = rospy.Rate(10)
        
    
    def set_up(self,waypoints,da_wae,n):
        self.waiting_list = da_wae[:]
        self.goal = self.waiting_list[0]

        self.client_goals.wait_for_server()
        self.publish_goal(n,waypoints,self.client_goals)

    def run(self,waypoints):
        way=(waypoints[self.goal]["x"],waypoints[self.goal]["y"])
        self.is_goal_achieved(self.client_goals.get_state(),self.pose,way)

        if self.goal_achieved:
            self.waiting_list.pop(0)
            rospy.set_param("/da_wae",self.waiting_list)
            if len(self.waiting_list) == 0:
                self.task_concluded = True
                rospy.loginfo("Welcome Home, brudahs!")

            else:
                rospy.loginfo("A leader is one who knows da wae, goes da wae and shows da wae")
            self.goal_achieved = False
            
            self.rate.sleep()
                

    def is_goal_achieved(self,data,feedback,way):
        if (feedback[0]-way[0])**2+(feedback[1]-way[1])**2 <0.3**2 and len(self.waiting_list)>1:
            self.goal_achieved=True
        elif data == 3:
            self.goal_achieved = True
        elif data == 4:
            rospy.logerr("Mission aborted")
            self.problemo=True
        elif data == 5:
            rospy.logerr("You don't know da wae")
            self.problemo=True

    def publish_goal(self,n,waypointse,client):
        way=None
        for key in waypointse:
            if self.goal==key:
                way=waypointse[key].copy()
                way.pop("weight",None)
                pass
     
        if not way:
            rospy.logerr("Spit on the fake queen!")
            self.problemo=True

        point = Point(x=way["x"],y=way["y"],z=0.0)
        quaternion = Quaternion(x=0.0,y=0.0,z=way["yawn"],w=sqrt(1-way["yawn"]**2))
        now = rospy.Time.now()

        new_pose= Pose(position=point, orientation=quaternion)        
        new_header= Header(seq=n, stamp=now, frame_id= "map")

        new_simple_goal=PoseStamped(header=new_header,pose=new_pose)
        new_move_base_goal= MoveBaseGoal(target_pose=new_simple_goal)       

        # Sends the goal to the action server.
        client.send_goal(new_move_base_goal)

        self.sequence+=1

    def get_pose(self):

        now=rospy.Time.now()
        while not self.tf_pose.canTransform("/map","/base_link",now):
            now=self.tf_pose.getLatestCommonTime("/map","/base_link")

        (posi,orientation)=self.tf_pose.lookupTransform("/map","/base_link",now)

        self.pose=(posi[0],posi[1])


if __name__ == '__main__':

    param_waypoints=None
    while not rospy.is_shutdown() and param_waypoints==None:
        param_waypoints=rospy.get_param("/waypoints",None)
    
    param_da_wae=[]
    my_obj=My_way()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        my_obj.get_pose()
        new_param_wae=rospy.get_param("/da_wae",[])
        if new_param_wae!=[] and my_obj.pose:
            if param_da_wae!=new_param_wae:
                if my_obj.task_concluded or my_obj.problemo:
                    my_obj.task_concluded=False
                    my_obj.problemo=False

                my_obj.set_up(param_waypoints, new_param_wae, my_obj.sequence)
                param_da_wae=new_param_wae

            if not my_obj.task_concluded and not my_obj.problemo:
                my_obj.run(param_waypoints)

        my_obj.rate.sleep()