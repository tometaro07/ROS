#!/usr/bin/env python

import rospy
import random as rd
from std_msgs.msg import Header
from actions import Actions
from geometry_msgs.msg import Pose, PoseArray

class Learner:
    def __init__(self,initial_pose):
        self.init_pose=initial_pose
        self.act=Actions()
        self.q_state=None
        self.goal_reached=False

        self.pub=rospy.Publisher("/policy", PoseArray,queue_size=1)

    def trainer_q(self,waypoints,alpha,gamma,epsilon,nr_tries):
        q_state={}

        for key in waypoints:
            self.act.pose=key.split(",")
            available=self.act.available_actions(waypoints)
            q_state[key]={}
            q_state[key]["reward"]=waypoints[key]["weight"]

            q_state[key]["actions_q"]={}
            q_state[key]["actions_next"]={}
            for action in available:
                q_state[key]["actions_q"][action]=0
                q_state[key]["actions_next"][action]=available[action][1]

        for _ in range(nr_tries):
            reward=0
            done=False
            state=self.init_pose

            while not done:
                if rd.random()< epsilon:
                    action=rd.choice(q_state[state]["actions_q"].keys())
                else:
                    action=rd.choice([key for key,x in q_state[state]["actions_q"].items() if x==max(q_state[state]["actions_q"].values())])

                next_state=q_state[state]["actions_next"][action]
                reward=q_state[state]["reward"]
                done= reward==1

                old_value= q_state[state]["actions_q"][action]
                next_max=max(q_state[next_state]["actions_q"].values())

                new_value=(1-alpha)*old_value+alpha*(reward+gamma*next_max)
                q_state[state]["actions_q"][action]=new_value

                state=next_state

        self.q_state=q_state

        array=[]
        for stat in q_state:
            xi=waypoints[stat]["x"]
            yi=waypoints[stat]["y"]
            actions=[key for key,x in q_state[stat]["actions_q"].items() if x==max(q_state[stat]["actions_q"].values())]

            for act in actions:
                pose=Pose()
                pose.position.x=xi
                pose.position.y=yi
                if act=="up":
                    pose.orientation.z=0
                    pose.orientation.w=1
                elif act=="down":
                    pose.orientation.z=1
                    pose.orientation.w=0
                elif act=="right":
                    pose.orientation.z=-0.7
                    pose.orientation.w=0.714
                elif act=="left":
                    pose.orientation.z=0.7
                    pose.orientation.w=0.714

                array+=[pose]

        head= Header()
        head.stamp=rospy.Time.now()
        head.frame_id="map"
        self.pub.publish(PoseArray(header=head,poses=array))



    def trainer_SARAS(self,waypoints,alpha,gamma,epsilon,nr_tries):

        def choose_action(status):
            if rd.random()< epsilon:
                return rd.choice(q_state[status]["actions_q"].keys())
            else:
                return rd.choice([key for key,x in q_state[status]["actions_q"].items() if x==max(q_state[status]["actions_q"].values())])
                

        q_state={}

        for key in waypoints:
            self.act.pose=key.split(",")
            available=self.act.available_actions(waypoints)
            q_state[key]={}
            q_state[key]["reward"]=waypoints[key]["weight"]

            q_state[key]["actions_q"]={}
            q_state[key]["actions_next"]={}
            for action in available:
                q_state[key]["actions_q"][action]=0
                q_state[key]["actions_next"][action]=available[action][1]

        for _ in range(nr_tries):
            reward=0
            done=False

            state1=rd.choice(q_state.keys())
            action1= choose_action(state1)

            step=0

            while not done and step<100:
                state2=q_state[state1]["actions_next"][action1]
                action2=choose_action(state2)

                reward=q_state[state1]["reward"]
                done= reward==1

                old_value= q_state[state1]["actions_q"][action1]

                new_value=(1-alpha)*old_value+alpha*(reward+gamma*q_state[state2]["actions_q"][action2])

                q_state[state1]["actions_q"][action1]=new_value

                state1=state2
                action1=action2
                step+=1

                array=[]

        self.q_state=q_state

        for stat in q_state:
            xi=waypoints[stat]["x"]
            yi=waypoints[stat]["y"]
            actions=[key for key,x in q_state[stat]["actions_q"].items() if x==max(q_state[stat]["actions_q"].values())]

            for act in actions:
                pose=Pose()
                pose.position.x=xi
                pose.position.y=yi
                if act=="up":
                    pose.orientation.z=0
                    pose.orientation.w=1
                elif act=="down":
                    pose.orientation.z=1
                    pose.orientation.w=0
                elif act=="right":
                    pose.orientation.z=-0.7
                    pose.orientation.w=0.714
                elif act=="left":
                    pose.orientation.z=0.7
                    pose.orientation.w=0.714

                array+=[pose]

        head= Header()
        head.stamp=rospy.Time.now()
        head.frame_id="map"
        self.pub.publish(PoseArray(header=head,poses=array))
        print(array)

    def tester(self,nr,waypoints,alpha,gamma,epsilon,nr_tries):

        total_epochs,total_penalties=0,0
        episodes=nr

        for _ in range(episodes):
            self.trainer(waypoints,alpha,gamma,epsilon,nr_tries)

            state=self.init_pose
            reward=0

            epochs=0

            done=False
            while not done:
                action=rd.choice([key for key,x in self.q_state[state]["actions_q"].items() if x==max(self.q_state[state]["actions_q"].values())])
                state=self.q_state[state]["actions_next"][action]

                reward=self.q_state[state]["reward"]
                done= reward==1

                if reward ==-1:
                    total_penalties+=1

                epochs+=1
            total_epochs+=epochs


        rospy.loginfo("After being tested "+str(nr)+" times, with "+str(nr_tries)+" learning steps, the average minimum path was "+str(float(total_epochs)/float(episodes))+" and it made "+
        str(float(total_penalties)/float(episodes))+" penalties")

    def decision_maker(self):

        state=self.init_pose
        reward=self.q_state[state]["reward"]
        penalties=0

        while reward!=1:
            if reward==-1:
                penalties+=1

            
            pose=state.split(",")
            action=rd.choice([key for key,x in self.q_state[state]["actions_q"].items() if x==max(self.q_state[state]["actions_q"].values())])

            self.act.pose=pose

            if action=="up":
                self.act.up()
            elif action=="down":
                self.act.down()
            elif action=="left":
                self.act.left()
            elif action=="right":
                self.act.right()

            state=self.q_state[state]["actions_next"][action]
            reward=self.q_state[state]["reward"]
            
        rospy.loginfo("Got the best path. It will commit "+str(penalties)+" penalties")
        self.goal_reached=True

    def decision_apllyer(self,coords):
        self.act.apply_action(coords)
        self.act.relative_angle=0

        if rospy.get_param("/wae",[])==[]:
            self.act.inprocessx= False
