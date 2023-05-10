#!/usr/bin/env python

from actions import Actions
import rospy
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import OccupancyGrid, MapMetaData, GridCells, Odometry
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from MDP import Agent
#from numpy import inf as oo

class Greed:
    def __init__(self):
        rospy.init_node("greedo")

        self.pub_greedmap=rospy.Publisher('greedmap', OccupancyGrid,queue_size=1)
        self.pub_greed=rospy.Publisher('greed', GridCells,queue_size=1)
        self.pub_obs_cells=rospy.Publisher('obs_cells', GridCells,queue_size=1)
        self.pub_goal_cells=rospy.Publisher('goal_cells', GridCells,queue_size=1)
        self.service_subscriber=rospy.ServiceProxy("/static_map",GetMap)

        self.subs_pose=rospy.Subscriber("/odom",Odometry,self.get_pose)
        self.subs_clicker=rospy.Subscriber('clicked_point', PointStamped,self.cell_finder)

        self.pose=None

        self.rate=rospy.Rate(10)



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

        gridpoints=["waypoints:\n"]
        waypoints={}
        points=[]

#        maxx=0
#        maxy=0
#        minx=oo
#        miny=oo

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
#                    maxx=max(maxx,j)
#                    maxy=max(maxy,i)
#                    minx=min(minx,j)
#                    miny=min(miny,i)

                    self.grid[entryi+j]=0
                    xp=(j+0.5)*0.3+self.origin.position.x
                    yp=(i+0.5)*0.3+self.origin.position.y
                    points+=[Point(x=xp,y=yp,z=0)]
                    waypoints[str(j)+","+str(i)]={"x":xp,"y":yp,"yawn":0,"weight":-1}
                    gridpoints+= ["  "+str(j)+","+str(i)+":\n","    x: "+str(xp)+"\n","    y: "+str(yp)+"\n","    yawn: "+str(0)+"\n","    weight: "+str(-1)+"\n"]
                else:
                    self.grid[entryi+j]=-1

        now=rospy.Time.now()
        map=[Header(seq=0,stamp= now, frame_id="map"),
            MapMetaData (map_load_time=now ,resolution=0.3,width=int(self.params["width"]/self.size),height=int(self.params["height"]/self.size),origin=self.origin),
            self.grid]

        self.greedmap=OccupancyGrid(header=map[0],info=map[1],data=map[2])
        self.greed=GridCells(header=map[0],cell_width=0.3,cell_height=0.3,cells=points)
        self.waypoint=waypoints

        f = open("/home/tometaro/catkin_ws/src/action_planning/config/gridpoints.yaml", "w")
        f.writelines(gridpoints)
        f.close()
        rospy.set_param("/waypoints",waypoints)

    def run(self):
        rospy.wait_for_service('/static_map')
        self.callback(self.service_subscriber(GetMapRequest()))

    def obs_cells(self, obs, waypoints):
        way=waypoints.copy()

        points=[]
        for point in obs:
            for key in way:
                if point==key:
                    points+=[Point(x=way[key]["x"],y=way[key]["y"],z=0)]
                    way[key]["weight"]=-100

        now=rospy.Time.now()
        map2=GridCells(header=Header(seq=0,stamp= now, frame_id="map"),
            cell_width=0.3, cell_height=0.3, cells=points)

        rospy.set_param("/waypoints",way)

        self.pub_obs_cells.publish(map2)
        self.obs=map2

    def goal_cells(self,obs,goals,waypoint):
        way=waypoint.copy()

        if len(goals)>1:
            rospy.set_param("/next_goal_waypoints",goals[1:])

        way[goals[0]]["weight"]=100

        l=[goals[0]]
        pk=[]
        i=1

        while l!=[]:
            new_l=[]
            for key in l:
                [x,y]=key.split(",")
                for k in range(-1,2,2):
                    if str(int(x)+k)+","+y not in new_l:
                        new_l+=[str(int(x)+k)+","+y]
                    if x+","+str(int(y)+k) not in new_l:
                        new_l+=[x+","+str(int(y)+k)]
               
            for j in range(len(new_l)):
                if new_l[j] in pk:
                    new_l[j]=None
                if obs!=None:
                    if new_l[j] in obs:
                        new_l[j]=None

            new_l=[prol for prol in new_l if prol!=None]
            pk=l[:]
            l=[]
            for po in new_l:
                if po in way:
                    way[po]["weight"]=-(1.005)**i
                    l+=[po]
            i+=1       
        rospy.loginfo("The furthest cell from the goal is at least "+str(i)+" actions away")         

        now=rospy.Time.now()
        map1=GridCells(header=Header(seq=0,stamp= now, frame_id="map"),
            cell_width=0.3, cell_height=0.3, cells= [Point(x=way[goals[0]]["x"], y=way[goals[0]]["y"],z=0)])

        rospy.set_param("/waypoints",way)


        self.pub_goal_cells.publish(map1)
        self.goal=map1

    def weight_giver(self,waypoints,obs,goals):
        rospy.set_param("/waypoints",waypoints)
        new_waypoints={key: waypoints[key].copy() for key in waypoints}
        if obs!=None:
            self.obs_cells(obs,new_waypoints)
            rospy.logwarn("You shall not pass!")
        if goals!=None:
            self.goal_cells(obs,goals,new_waypoints)
            rospy.logwarn("New Goal has been set")

    def get_pose(self,data):
        coords=data
        coords=(coords.pose.pose.position.x,coords.pose.pose.position.y,coords.pose.pose.orientation.z,coords.pose.pose.orientation.w)
        cell=False
        waypoints=self.waypoint

        for key in waypoints:
            if abs(waypoints[key]["x"]-coords[0])<=0.15 and abs(waypoints[key]["y"]-coords[1])<=0.15:
                cell=True
                self.pose=key
                self.coords=(coords[2],coords[3])
                pass
        if not cell:
            rospy.loginfo("That point doesn't belong to any cell")


    def cell_finder(self,data):
        global last_msg
    
        if last_msg!=data:
            last_msg=data
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

        last_msg=None #definir sempre que queremos usar cel

        start=False

        while not rospy.is_shutdown():
            new_param_goal=rospy.get_param("/goal_waypoints",None)
            new_param_obs=rospy.get_param("/obs_waypoints",None)

            connections=[myObject.pub_greedmap.get_num_connections(),myObject.pub_greed.get_num_connections(),
                        myObject.pub_obs_cells.get_num_connections(),myObject.pub_goal_cells.get_num_connections()]

            if connections[0]>old_connects[0] or connections[1]>old_connects[1]:
                myObject.pub_greed.publish(myObject.greed)
                myObject.pub_greedmap.publish(myObject.greedmap)

            if (param_obs!=new_param_obs and new_param_obs!=None) or (param_goal!=new_param_goal and new_param_goal!=None):
                myObject.weight_giver(myObject.waypoint ,new_param_obs,new_param_goal)

                param_goal=new_param_goal
                param_obs=new_param_obs

            if connections[2]>old_connects[2] or connections[3]>old_connects[3]:
                if new_param_goal!=None:
                    myObject.pub_goal_cells.publish(myObject.goal)
                if new_param_obs!=None:
                    myObject.pub_obs_cells.publish(myObject.obs)

            if connections!=old_connects:
                old_connects=connections[:]

            myObject.subs_clicker

            myObject.subs_pose

            if not start and new_param_goal!=None and myObject.pose:
                
                markov=Agent(myObject.pose,None,new_param_goal[0])
                start=True

            if start:
                markov.decision_maker(rospy.get_param("/waypoints"))
                if rospy.get_param("/wae",[])!=[]:
                    markov.decision_apllyer(myObject.coords)

            myObject.rate.sleep()



    finally:
        if rospy.has_param("goal_waypoints"):
            rospy.delete_param("goal_waypoints")
        if rospy.has_param("obs_waypoints"):
            rospy.delete_param("obs_waypoints")
        if rospy.has_param("waypoints"):
            rospy.delete_param("waypoints")