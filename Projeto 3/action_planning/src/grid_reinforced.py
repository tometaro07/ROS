#!/usr/bin/env python

from grido import Greed
import rospy
from reinforcement_learning import Learner


if __name__ == "__main__":
    try:
        myObject=Greed()
        old_connects=[0,0,0,0]
        myObject.run()

        param_goal=None
        param_obs=None


        start=False
        griddddd=False

        safe=rospy.get_param('~safe',False)

        while not rospy.is_shutdown() and rospy.has_param("/waypoints"):
            new_param_goal=rospy.get_param("/goal_waypoints",[])
            new_param_obs=rospy.get_param("/obs_waypoints",[])

            connections=[myObject.pub_greedmap.get_num_connections(),myObject.pub_greed.get_num_connections(),
                        myObject.pub_obs_cells.get_num_connections(),myObject.pub_goal_cells.get_num_connections()]

            if connections[0]>old_connects[0] or connections[1]>old_connects[1]:
                myObject.pub_greed.publish(myObject.greed)
                myObject.pub_greedmap.publish(myObject.greedmap)

            if param_obs!=new_param_obs or param_goal!=new_param_goal:
                myObject.weight_giver(myObject.waypoint ,new_param_obs,new_param_goal)

                param_goal=new_param_goal
                param_obs=new_param_obs

            if connections[2]!=old_connects[2] or connections[3]!=old_connects[3]:
                myObject.pub_goal_cells.publish(myObject.goal)
                myObject.pub_obs_cells.publish(myObject.obs)

            if connections!=old_connects:
                old_connects=connections[:]

            myObject.subs_clicker

            myObject.get_pose()

            if not start and new_param_goal!=[] and myObject.pose and not safe:
                rein=Learner(myObject.pose)
                rein.trainer_q(rospy.get_param("/waypoints"),0.1,0.7,1,1000)
                start=True
                if rospy.has_param("/wae"):
                    rospy.delete_param("/wae")

            if not start and new_param_goal!=[] and myObject.pose and safe:
                rein=Learner(myObject.pose)
                rein.trainer_SARAS(rospy.get_param("/waypoints"),0.2,0.8,0.2,1000)
                start=True
                if rospy.has_param("/wae"):
                    rospy.delete_param("/wae")

            myObject.get_pose()

            if start and not rein.goal_reached:
                rein.decision_maker()


            if start and rospy.get_param("/wae",[])!=[]:
                rein.decision_apllyer(myObject.coords)


            if start and myObject.reached:
                myObject.reached=False
                start=False

            if start and myObject.reached:
                myObject.reached=False
                start=False

            if param_obs!=new_param_obs or param_goal!=new_param_goal:
                param_goal=new_param_goal
                param_obs=new_param_obs

            myObject.rate.sleep()


    finally:
        if rospy.has_param("goal_waypoints"):
            rospy.delete_param("goal_waypoints")
        if rospy.has_param("obs_waypoints"):
            rospy.delete_param("obs_waypoints")
        if rospy.has_param("waypoints"):
            rospy.delete_param("waypoints")