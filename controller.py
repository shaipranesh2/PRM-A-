#!/usr/bin/python
import rospy
from bot.msg import Obstacle
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

rospy.init_node('pid_controller')

class controller:
    def __init__(self):
        self.kp=0.1
        self.kd=0.5
        self.map=[]
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
        self.init=Twist()
        self.init.linear.x=0.000000000000000000000
        self.init.linear.y=0.000000000000000000000
        self.init.linear.z=0.000000000000000000000
        self.init.angular.x=0.00000000000000000000
        self.init.angular.y=0.00000000000000000000
        self.init.angular.z=0.00000000000000000000
        self.pub.publish(self.init)
        self.sub = rospy.Subscriber('/controller',Obstacle, self.map_populate)
        
    def map_populate(self,msg):

        self.map.append([msg.x,msg.y])
        #print(self.map)
        if msg.end==1:
            self.pid_sub = rospy.Subscriber('/odom',Odometry, self.pose_estimate)

    def pose_estimate(self,msg):

        orientation_q=msg.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll,pitch,yaw)=euler_from_quaternion(orientation_list)
        try:
            trajectory_z=math.atan(float(self.map[0][1]-self.map[1][1])/float(self.map[0][0]-self.map[1][0]))
            #trajectory_z=(3.14159254/2)-trajectory_z
        except:
            if ((self.map[0][1]-self.map[1][1]) <0) or ((self.map[0][0]-self.map[1][0]) <0):
                trajectory_z=-3.14159254
            else:
                trajectory_z=3.14159254
        #print(self.map[0][0]-self.map[1][0])
        #dest=self.map[1]

        err_x=(float((msg.pose.pose.position.x-self.map[1][0])**2+(msg.pose.pose.position.y-self.map[1][1])**2))**0.5
        print(err_x)
        err_z=float(trajectory_z-yaw)
        if abs(err_z)>=0.01:
            self.init.linear.x=0.00000000
            self.init.angular.z=((float(self.kp*err_z-(self.kd*msg.twist.twist.angular.z))))
            self.pub.publish(self.init)
        else:
            self.init.angular.z=0.00000000
            if abs(err_x)>=0.01:
                self.init.linear.x=2*((float(self.kp*err_x-(self.kd*msg.twist.twist.linear.x))))
            else:
                self.init.linear.x=0.00000000
                self.map.pop(0)

            self.pub.publish(self.init)
            
bot=controller()
rospy.spin()