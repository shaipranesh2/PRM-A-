#!/usr/bin/env python
import rospy
from bot.msg import Obstacle
rospy.init_node('obstacle_publisher')

pub = rospy.Publisher('obstacle', Obstacle,queue_size=1)
rate = rospy.Rate(10)
msg=Obstacle()
rate.sleep()
#while not rospy.is_shutdown():
for i in range(0,60,15): 
    for j in range(0,60,15):
        rate.sleep()
        msg.x=float(i)/10
        msg.y=float(j)/10
        msg.end=0
        if not( msg.x==0 and msg.y==0):
            print([msg.x,msg.y])
            pub.publish(msg)
        rate.sleep()
msg.end=1
pub.publish(msg)