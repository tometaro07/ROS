#!/usr/bin/env python

import rospy
import tf
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from MDP import Agent
from numpy import inf as oo

class Greed:
    def __init__(self):
        rospy.init_node("greedo")

        self.pub_greedmap=rospy.Publisher('greedmap', OccupancyGrid,queue_size=1)
        self.pub_greed=rospy.Publisher('greed', GridCells,queue_size=1)
        self.pub_obs_cells=rospy.Publisher('obs_cells', GridCells,queue_size=1)
        self.pub_goal_cells=rospy.Publisher('goal_cells', GridCells,queue_size=1)
        self.service_subscriber=rospy.ServiceProxy("/static_map",GetMap)

        self.subs_clicker=rospy.Subscriber('clicked_point', PointStamped,self.cell_finder)
        self.tf_pose=tf.TransformListener()

        self.pose=None
        self.waypoint={}
        self.reached=False
        self.last_msg=None

        self.rate=rospy.Rate(10)
        self.greed_size=None



    def callback(self,msg):
        self.info=msg.map
        self.data=self.info.data
        self.params={"height":self.info.info.height,"width":self.info.info.width,"resolution":self.info.info.resolution}
        self.size=int(round(0.3/self.params["resolution"],0))

        self.origin=self.info.info.origin

        self.matrix=[[0 for i in range(self.params["width"])][:] for i in range(self.params["height"])]

        for i in range(self.params["height"]):
            for j in range(self.params["width"]):
                self.matrix[i][j]=self.data[j+i*self.params["width"]]

        self.grid=[0]*int(self.params["width"]/self.size)*int(self.params["height"]/self.size)

        waypoints={}
        points=[]

        maxx=-oo
        maxy=-oo
        minx=oo
        miny=oo

        for i in range(int(self.params["height"]/self.size)):
            entryi= i*int(self.params["width"]/self.size)
            for j in range(int(self.params["width"]/self.size)):
                neg=0
                clean=0
                occup=0
                for k in range(self.size):
                    for n in range(self.size):
                        trlo=self.matrix[i*self.size+k][j*self.size+n]
                        if trlo<0:
                            neg+=1
                        elif trlo>0:
                            occup+=1
                        else:
                            clean+=1
                

                if occup>=self.size:
                    self.grid[entryi+j]=100
                elif clean==max(occup,clean,neg):
                    maxx=max(maxx,j)
                    maxy=max(maxy,i)
                    minx=min(minx,j)
                    miny=min(miny,i)

                    self.grid[entryi+j]=-1
                    xp=(j+0.5)*0.3+self.origin.position.x
                    yp=(i+0.5)*0.3+self.origin.position.y
                    points+=[Point(x=xp,y=yp,z=0)]
                    waypoints[str(j)+","+str(i)]={"x":xp,"y":yp,"yawn":0,"weight":0}
                else:
                    self.grid[entryi+j]=-1

        now=rospy.Time.now()
        map=[Header(seq=0,stamp= now, frame_id="map"),
            MapMetaData (map_load_time=now ,resolution=0.3,width=int(self.params["width"]/self.size),height=int(self.params["height"]/self.size),origin=self.origin),
            self.grid]

        self.greedmap=OccupancyGrid(header=map[0],info=map[1],data=map[2])
        self.greed=GridCells(header=map[0],cell_width=0.3,cell_height=0.3,cells=points)
        self.waypoint=waypoints
        self.greed_size=(minx,maxx,miny,maxy)

        rospy.set_param("/waypoints",waypoints)

    def run(self):
        rospy.wait_for_service('/static_map')
        self.callback(self.service_subscriber(GetMapRequest()))

    def obs_cells(self, obs, waypoints):
        way=waypoints.copy()

        points=[]
        for point in obs:
            if point in way:
                points+=[Point(x=way[point]["x"],y=way[point]["y"],z=0)]
                way[point]["weight"]=-1

        now=rospy.Time.now()
        map2=GridCells(header=Header(seq=0,stamp= now, frame_id="map"),
            cell_width=0.3, cell_height=0.3, cells=points)

        rospy.set_param("/waypoints",way)

        self.pub_obs_cells.publish(map2)
        self.obs=map2

    def goal_cells(self,goals,waypoint):
        way=waypoint.copy()

        gle=[]
        if len(goals)>1:
            gle=goals[1:]
        
        rospy.set_param("/next_goal_waypoints",gle)


        point=[]
        if goals!=[]:
            way[goals[0]]["weight"]=1
            point+=[Point(x=way[goals[0]]["x"], y=way[goals[0]]["y"],z=0)]

        now=rospy.Time.now()
        map1=GridCells(header=Header(seq=0,stamp= now, frame_id="map"),
            cell_width=0.3, cell_height=0.3, cells=point)

        rospy.set_param("/waypoints",way)

        self.pub_goal_cells.publish(map1)
        self.goal=map1

    def weight_giver(self,waypoints,obs,goals):
        rospy.set_param("/waypoints",waypoints)
        new_waypoints={key: waypoints[key].copy() for key in waypoints}

        self.obs_cells(obs,new_waypoints)
        rospy.logwarn("You shall not pass!")

        self.goal_cells(goals,new_waypoints)
        rospy.logwarn("New Goal has been set")

    def get_pose(self):

        now=rospy.Time.now()
        while not self.tf_pose.canTransform("/map","/base_link",now):
            now=self.tf_pose.getLatestCommonTime("/map","/base_link")
        (pose,orientation)=self.tf_pose.lookupTransform("/map","/base_link",now)

        cell=False
        waypoints=self.waypoint

        for key in waypoints:
            if abs(waypoints[key]["x"]-pose[0])<=0.15 and abs(waypoints[key]["y"]-pose[1])<=0.15:
                cell=True
                self.pose=key
                self.coords=(orientation[2],orientation[3])
                
                goaal= rospy.get_param("/goal_waypoints",[])
                if key in goaal:
                    if key == goaal[0]:
                        self.reached=True
                        rospy.set_param("/goal_waypoints",rospy.get_param("/next_goal_waypoints",[]))
                        print(rospy.get_param("/next_goal_waypoints",[]))
                        rospy.logwarn("You reached your goal")
                        print(rospy.get_param("/goal_waypoints",[]))

    
        if not cell:
            rospy.loginfo("That point doesn't belong to any cell")


    def cell_finder(self,data):
    
        if self.last_msg!=data:
            self.last_msg=data
            coords=data
            coords=(coords.point.x,coords.point.y)
            cell=False
            waypoints=rospy.get_param("/waypoints")

            for key in waypoints:
                if abs(waypoints[key]["x"]-coords[0])<=0.15 and abs(waypoints[key]["y"]-coords[1])<=0.15:
                    rospy.loginfo("This is the cell "+key+" and it's weight is "+str(waypoints[key]["weight"]))
                    cell=True
                    pass

            if not cell:
                rospy.loginfo("That point doesn't belong to any cell")


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
                markov=Agent(myObject.pose,new_param_goal[0])
                markov.policy_maker_not_conservative(rospy.get_param("/waypoints"),myObject.greed_size)
                if rospy.has_param("/wae"):
                    rospy.delete_param("/wae")
                start=True

            if not start and new_param_goal!=[] and myObject.pose and safe:             
                markov=Agent(myObject.pose,new_param_goal[0])
                markov.policy_maker(rospy.get_param("/waypoints"),myObject.greed_size)
                if rospy.has_param("/wae"):
                    rospy.delete_param("/wae")
                start=True          

            myObject.get_pose()

            if start and not markov.goal_reached:
                markov.decision_maker(rospy.get_param("/waypoints"))
            
            if start and rospy.get_param("/wae",[])!=[]:
                    markov.decision_apllyer(myObject.coords)

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