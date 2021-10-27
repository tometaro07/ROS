#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib

from math import sqrt

class My_way(object):
    def __init__(self, path):
        rospy.init_node("we_know_da_wae")
                
        self.waiting_list = path[:]
        self.goal = self.waiting_list[0]
        

        self.goal_achieved = False
        self.task_concluded = False
        
        self.rate = rospy.Rate(10)
        
        
    def run(self, waypoints):
        # Create an action client called "almost_there" with action definition file "MoveBaseAction"
        client_goals = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        # Waits until the action server has started up and started listening for goals.
        client_goals.wait_for_server()

        n=0

        self.publish_goal(n,waypoints,client_goals)

        while not rospy.is_shutdown() and not self.task_concluded:

            self.is_goal_achieved(client_goals.get_state())

            if self.goal_achieved:
                self.waiting_list.pop(0)
                if len(self.waiting_list) == 0:
                    self.task_concluded = True
                    rospy.loginfo("Welcome to Uganda, brudahs!")
                    rospy.delete_param("/da_wae")

                else:
                    rospy.loginfo("A leader is one who knows da wae, goes da wae and shows da wae")
                    n +=1
                    self.goal = self.waiting_list[0]
                    client_goals.wait_for_server()
                    self.publish_goal(n,waypoints,client_goals)         
                    self.goal_achieved = False
                
            self.rate.sleep()
        
                

    def is_goal_achieved(self,data):
        if data == 3:
            self.goal_achieved = True
        elif data == 4:
            rospy.loginfo("Mission aborted")
            pass
        elif data == 5:
            rospy.loginfo("You don't know da wae")
            pass

    def publish_goal(self,n,waypointse,client):
        way=None
        counter=0
        found=False
        while not found and counter <=len(waypointse)-1:
            if self.goal==waypointse[counter]["name"]:
                way=waypointse[counter]["pose"]
                found=True
            counter+=1
     
        if not way:
            rospy.loginfo("Spit on the fake queen!")
            pass

        point = Point(x=way["x"],y=way["y"],z=0.0)
        quaternion = Quaternion(x=0.0,y=0.0,z=way["yawn"],w=sqrt(1-way["yawn"]**2))
        now = rospy.Time.now()

        new_pose= Pose(position=point, orientation=quaternion)        
        new_header= Header(seq=n, stamp=now, frame_id= "map")

        new_simple_goal=PoseStamped(header=new_header,pose=new_pose)
        new_move_base_goal= MoveBaseGoal(target_pose=new_simple_goal)       

           # Sends the goal to the action server.
        client.send_goal(new_move_base_goal)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        if rospy.has_param("/da_wae"):
            my_obj = My_way(rospy.get_param("/da_wae"))
            my_obj.run(rospy.get_param("/waypoints"))
        my_obj.rate.sleep()
