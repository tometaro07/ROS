#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time

class Actions:
    def __init__(self):
        self.pose=None
        self.aplly_rotation=False
        self.inprocessx=False
        self.relative_angle=None

        self.pub_vel=rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def up(self):
        old_wae=rospy.get_param("/wae",[])
        
        new_pose=str(int(self.pose[0])+1)+","+self.pose[1]

        new_wae=old_wae+[("up",new_pose)]
        rospy.set_param("/wae",new_wae)

    def down(self):
        old_wae=rospy.get_param("/wae",[])
        
        new_pose=str(int(self.pose[0])-1)+","+self.pose[1]

        new_wae=old_wae+[("down",new_pose)]
        rospy.set_param("/wae",new_wae)

    def right(self):
        old_wae=rospy.get_param("/wae",[])
        
        new_pose=self.pose[0]+","+str(int(self.pose[1])-1)

        new_wae=old_wae+[("right",new_pose)]
        rospy.set_param("/wae",new_wae)

    def left(self):
        old_wae=rospy.get_param("/wae",[])
        
        new_pose=self.pose[0]+","+str(int(self.pose[1])+1)

        new_wae=old_wae+[("left",new_pose)]
        rospy.set_param("/wae",new_wae)

    def apply_action(self,coords):
        actions=rospy.get_param("/wae")
        next_action=actions[0]
        vel=Twist()
        angle=quaternion_to_euler(0,0,coords[0],coords[1])[2]

        if not self.inprocessx:
            if next_action[0]=="up":
                self.relative_angle=angle

            if next_action[0]=="down":
                self.relative_angle=angle-math.pi if angle>0 else math.pi+angle

            if next_action[0]=="left":
                self.relative_angle=angle-math.pi/2 if angle>0 else (-math.pi/2+angle if abs(angle)<math.pi/2 else angle+2*math.pi-math.pi/2)

            if next_action[0]=="right":
                self.relative_angle=angle+math.pi/2 if angle<0 else (math.pi/2+angle if abs(angle)<math.pi/2 else angle-2*math.pi+math.pi/2)


            vel.angular.z=-self.relative_angle/abs(self.relative_angle)*0.9
            self.pub_vel.publish(vel)
            time.sleep(abs(self.relative_angle)/0.9)
            self.pub_vel.publish(Twist())

        if abs(self.relative_angle)<1e-1:
            self.inprocessx=True
            wae_wae=rospy.get_param("/da_wae",[])[:]+[next_action[1]]
            rospy.set_param("/da_wae",wae_wae)
            next=rospy.get_param("/wae",[])
            next.pop(0)
            rospy.set_param("/wae",next)



    def available_actions(self,waypoints):
        actions={"up": str(int(self.pose[0])+1)+","+self.pose[1],"down": str(int(self.pose[0])-1)+","+self.pose[1],"right": self.pose[0]+","+str(int(self.pose[1])-1),"left":self.pose[0]+","+str(int(self.pose[1])+1)}
        for key in actions:
            if actions[key] in waypoints:
                actions[key]=(waypoints[actions[key]]["weight"], actions[key])
            else:
                actions[key]=None
        
        actions={key:x for key,x in actions.items() if x}

        return actions



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