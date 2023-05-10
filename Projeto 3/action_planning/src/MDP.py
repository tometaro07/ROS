#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray
import random as rd
from actions import Actions
from numpy import inf as oo

class Agent:
    def __init__(self, grid_pose, goal):
        self.comulative_reward=0
        self.pose=grid_pose
        self.goals=goal
        self.goal_reached=False
        self.act=Actions()

        self.pub=rospy.Publisher("/policy", PoseArray,queue_size=1)

    def policy_maker(self,waypoints,size):
        gama=0.75
        x=size[1]-size[0]+1
        y=size[3]-size[2]+1

        new_grid=None
        new_old_grid=[[0]*y for n in range(x)]

        while new_old_grid!=new_grid:
            array=[]
            if new_grid:
                new_old_grid=[l[:] for l in new_grid]
            new_grid=[[0]*y for k in range(x)]
            for i in range(x):
                for j in range(y):
                    down=up=right=left=-oo

                    key_s=str(i+size[0])+","+str(j+size[2])
                    keyu=str(i+1+size[0])+","+str(j+size[2])
                    keyd=str(i-1+size[0])+","+str(j+size[2])
                    keyr=str(i+size[0])+","+str(j-1+size[2])
                    keyl=str(i+size[0])+","+str(j+1+size[2])

                    if not key_s in waypoints:
                        continue
                    R=waypoints[key_s]["weight"]

                    isd=keyd in waypoints 
                    isu=keyu in waypoints
                    isr=keyr in waypoints
                    isl=keyl in waypoints

                    if not isd:
                        if not isr and not isl:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.05*new_old_grid[i][j])
                        elif not isr:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i][j+1])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i+1][j])
                        elif not isl:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i][j-1])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i+1][j])
                        else:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.017*new_old_grid[i][j]+0.0165*new_old_grid[i][j-1]+0.0165*new_old_grid[i][j+1])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i+1][j]+0.01*new_old_grid[i][j+1])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i+1][j]+0.01*new_old_grid[i][j-1])

                    elif not isu:
                        if not isr and not isl:
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.05*new_old_grid[i][j])
                        elif not isr:
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i][j+1])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i-1][j])
                        elif not isl:
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i][j-1])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.025*new_old_grid[i][j]+0.025*new_old_grid[i-1][j])
                        else:
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.017*new_old_grid[i][j]+0.0165*new_old_grid[i][j-1]+0.0165*new_old_grid[i][j+1])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i-1][j]+0.01*new_old_grid[i][j+1])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i-1][j]+0.01*new_old_grid[i][j-1])

                    else:
                        if not isr and isl:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.03*new_old_grid[i][j]+0.02*new_old_grid[i-1][j])
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.03*new_old_grid[i][j]+0.02*new_old_grid[i+1][j])
                        elif not isr:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i][j+1]+0.01*new_old_grid[i-1][j])
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i][j+1]+0.01*new_old_grid[i+1][j])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.017*new_old_grid[i][j]+0.0165*new_old_grid[i+1][j]+0.0165*new_old_grid[i-1][j])
                        elif not isl:
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i][j-1]+0.01*new_old_grid[i-1][j])
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.02*new_old_grid[i][j]+0.02*new_old_grid[i][j-1]+0.01*new_old_grid[i+1][j])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.017*new_old_grid[i][j]+0.0165*new_old_grid[i+1][j]+0.0165*new_old_grid[i-1][j])
                        else:
                            down=R+gama*(0.95*new_old_grid[i-1][j]+0.005*new_old_grid[i+1][j]+0.015*new_old_grid[i][j]+0.015*new_old_grid[i][j+1]+0.015*new_old_grid[i][j-1])
                            up=R+gama*(0.95*new_old_grid[i+1][j]+0.005*new_old_grid[i-1][j]+0.015*new_old_grid[i][j]+0.015*new_old_grid[i][j+1]+0.015*new_old_grid[i][j-1])
                            right=R+gama*(0.95*new_old_grid[i][j-1]+0.005*new_old_grid[i][j+1]+0.015*new_old_grid[i][j]+0.015*new_old_grid[i-1][j]+0.015*new_old_grid[i+1][j])
                            left=R+gama*(0.95*new_old_grid[i][j+1]+0.005*new_old_grid[i][j-1]+0.015*new_old_grid[i][j]+0.015*new_old_grid[i-1][j]+0.015*new_old_grid[i+1][j])

                    new_grid[i][j]=max(down,left,up,right)

                    point=Pose()
                    point.position.x=waypoints[key_s]["x"]
                    point.position.y=waypoints[key_s]["y"]

                    if waypoints[key_s]["weight"]!=1:
                        if new_grid[i][j]==up:
                            point.orientation.z=0
                            point.orientation.w=1
                            array+=[point]
                        if new_grid[i][j]==down:
                            point.orientation.z=1
                            point.orientation.w=0
                            array+=[point]
                        if new_grid[i][j]==right:
                            point.orientation.z=-0.7
                            point.orientation.w=0.714
                            array+=[point]
                        if new_grid[i][j]==left:
                            point.orientation.z=0.7
                            point.orientation.w=0.714
                            array+=[point]

        for i in range(x):
            for j in range(y):
                key=str(i+size[0])+","+str(j+size[2])
                if key in waypoints:
                    waypoints[key]["weight"]=new_grid[i][j]
                    head=Header()
                    head.stamp=rospy.Time.now()
                    head.frame_id="map"

        self.pub.publish(PoseArray(header=head, poses=array))

        rospy.set_param("/waypoints",waypoints)
        rospy.logwarn("Policy was calculated")
                    

    def policy_maker_not_conservative(self,waypoints,size):
        gama=0.7
        x=size[1]-size[0]+1
        y=size[3]-size[2]+1

        new_grid=None
        new_old_grid=[[0]*y for n in range(x)]

        while new_old_grid!=new_grid:
            array=[]
            if new_grid:
                new_old_grid=[l[:] for l in new_grid]
            new_grid=[[0]*y for k in range(x)]
            for i in range(x):
                for j in range(y):
                    down=up=right=left=-oo

                    key_s=str(i+size[0])+","+str(j+size[2])
                    keyu=str(i+1+size[0])+","+str(j+size[2])
                    keyd=str(i-1+size[0])+","+str(j+size[2])
                    keyr=str(i+size[0])+","+str(j-1+size[2])
                    keyl=str(i+size[0])+","+str(j+1+size[2])

                    if not key_s in waypoints:
                        continue
                    R=waypoints[key_s]["weight"]

                    isd=keyd in waypoints 
                    isu=keyu in waypoints
                    isr=keyr in waypoints
                    isl=keyl in waypoints

                    up= R+gama*new_old_grid[i+1][j] if isu else -oo
                    down= R+gama*new_old_grid[i-1][j] if isd else -oo
                    right= R+gama*new_old_grid[i][j-1] if isr else -oo
                    left= R+gama*new_old_grid[i][j+1] if isl else -oo

                    new_grid[i][j]=max(down,left,up,right)

                    if waypoints[key_s]["weight"]!=1:
                        if new_grid[i][j]==up:
                            point=Pose()
                            point.position.x=waypoints[key_s]["x"]
                            point.position.y=waypoints[key_s]["y"]
                            point.orientation.z=0
                            point.orientation.w=1
                            array+=[point]
                        elif new_grid[i][j]==down:
                            point=Pose()
                            point.position.x=waypoints[key_s]["x"]
                            point.position.y=waypoints[key_s]["y"]
                            point.orientation.z=1
                            point.orientation.w=0
                            array+=[point]
                        elif new_grid[i][j]==right:
                            point=Pose()
                            point.position.x=waypoints[key_s]["x"]
                            point.position.y=waypoints[key_s]["y"]
                            point.orientation.z=-0.7
                            point.orientation.w=0.714
                            array+=[point]
                        elif new_grid[i][j]==left:
                            point=Pose()
                            point.position.x=waypoints[key_s]["x"]
                            point.position.y=waypoints[key_s]["y"]
                            point.orientation.z=0.7
                            point.orientation.w=0.714
                            array+=[point]

        for i in range(x):
            for j in range(y):
                key=str(i+size[0])+","+str(j+size[2])
                if key in waypoints:
                    waypoints[key]["weight"]=new_grid[i][j]
                    head=Header()
                    head.stamp=rospy.Time.now()
                    head.frame_id="map"

                    self.pub.publish(PoseArray(header=head, poses=array))

        rospy.set_param("/waypoints",waypoints)
        rospy.logwarn("Policy was calculated")


    def decision_maker(self,waypoints):

        if self.goals==self.pose:
            self.comulative_reward+=waypoints[self.goals]["weight"]
            rospy.loginfo("Got best policy to get to the goal")
            self.goal_reached=True
        else:
            self.act.pose=self.pose.split(",")
            possibilities=self.act.available_actions(waypoints)

            choice=max([x[0] for x in possibilities.values()])

            self.comulative_reward+=choice
            
            for key in possibilities:
                if possibilities[key][0] != choice:
                    possibilities[key]=None
            
            possibilities=[key for key,x in possibilities.items() if x!=None]


            choosen_action=rd.choice(possibilities)

            if choosen_action=="up":
                self.act.up()
            elif choosen_action=="down":
                self.act.down()
            elif choosen_action=="left":
                self.act.left()
            elif choosen_action=="right":
                self.act.right()
            
            self.pose=rospy.get_param("/wae")[-1][1]

    def decision_apllyer(self,coords):
        self.act.apply_action(coords)
        self.act.relative_angle=0

        if rospy.get_param("/wae",[])==[]:
            self.act.inprocessx= False