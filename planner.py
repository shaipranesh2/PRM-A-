#!/usr/bin/env python
import rospy
from bot.msg import Obstacle
import random

rospy.init_node("planner")
start=[0.0,0.0]
end=[6.0,6.0]
neighbours=5
number=50

class Planner:
    def __init__(self):
        global start
        global end
        self.nodes=[start,end]
        self.explored=[]
        self.near=[]
        self.obs=[]
        self.sub = rospy.Subscriber('obstacle',Obstacle, self.map_populate)
        self.pub = rospy.Publisher('controller',Obstacle, queue_size=1)

    def map_populate(self, msg):
        if msg.end==0:
            self.obs.append([msg.x,msg.y])
        else:
            self.generate()
    
    def generate(self):
        print(self.obs)
        global number
        count=1
        while 1:
            self.x=random.uniform(0,6)
            self.y=random.uniform(0,6)
            if self.free(self.x,self.y):#[self.x,self.y] not in self.explored:
                self.nodes.append([self.x,self.y])
                if count>=number:
                    break
                else:
                    count=count+1
            else:
                continue
        
        self.graphing()
        self.search()
    
    def graphing(self): 
        for i in self.nodes:
            neigh_dict={}
            for j in self.nodes:
                if not (i[0]==j[0] and i[1]==j[1]):
                    value=self.dist(i,j)
                    if not (value<0):
                        neigh_dict[value]=j
            sorted_dict = sorted(neigh_dict.items(), key = lambda kv: kv[0])
            
            self.near.append([])
            for k in range(0,neighbours):
                try:
                    self.near[-1].append(sorted_dict[k][1])
                except:
                    break
            print(self.near[self.nodes.index([0.0,0.0])])

    def dist(self,i,j):
        slope=(i[1]-j[1])/(i[0]-j[0])
        for obs in self.obs:
            normal=abs(obs[1]-slope*obs[0]+slope*i[0]-i[1])/((1+(slope)**2)**0.5)
            x=((obs[0]/slope)+slope*i[0]-i[1]+obs[1])/((1/slope)+slope)
            y=slope*(x-i[0])+i[1]
            if normal<0.35 and ((y<=i[1] and y>=j[1]) or (y>=i[1] and y<=j[1]) or (x<=i[0] and x>=j[0]) or (x>=i[0] and x<=j[0])):
                return -1
        return ((j[1]-i[1])**2 + (j[0]-i[0])**2)
    
    def free(self,x,y):
        for point in self.obs:
            if ((x-point[0])**2 + (y-point[1])**2 <=0.35) or ([x,y]  in self.nodes):
                return 0
        return 1
    
    def search(self):
        global start
        global end
        h=1
        visited=[[start,((end[1]-start[1])**2 + (end[0]-start[0])**2)**0.5]]
        history=[start]
        explorer=[]
        
        
        while 1:
            parent=visited[0][0]
            if end[0]==parent[0] and end[1]==parent[1]:
                break
            for coord in self.near[self.nodes.index(parent)]:
                if coord not in history:
                    dist=((end[1]-coord[1])**2 + (end[0]-coord[0])**2)**0.5
                    g=h+dist
                    explorer.append([coord,g,parent])
                    history.append(coord)
            try:
                explorer.sort(key = lambda x: x[1])
                visited.insert(0,explorer[0])
                explorer.pop(0)
                h=h+1
            except:
                return

        final=[]
        current=end
        for i in visited:
            if not(current[0]==start[0] and current[1]==start[1]):
                if (i[0][1]==current[1] and i[0][0]==current[0]):
                    final.insert(0,i[0])
                    current=i[2]
            else:
                points=Obstacle()
                points.end=0
                final.insert(0,start)
                rate=rospy.Rate(15)
                print(final)
                
                for l in final:
                    if l[0]==end[0] and l[1]==end[1]:
                        points.end=1
                    points.x=l[0]
                    points.y=l[1]
                    self.pub.publish(points)
                    rate.sleep()
                return

        



bot=Planner()
rospy.spin()