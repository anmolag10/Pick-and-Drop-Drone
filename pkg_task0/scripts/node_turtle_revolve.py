#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import time
flag = 0 
final_x = 5.53
final_y = 5.54
x = 0
y = 0

def callback(dat):
    global final_x, final_y, x, y
    x = round(dat.x, 2) 
    y = round(dat.y, 2)  
    print(final_x,x, final_y, y)
    



rospy.init_node('turtle_rotate', disable_signals=True)

rospy.Subscriber("/turtle1/pose", Pose, callback)
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
speed = Twist()
r = rospy.Rate(100)

while not rospy.is_shutdown():
    
    if final_x == x and final_y == y:
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        break

    speed.linear.x = 0.4
    speed.angular.z = 0.4
    pub.publish(speed)

    r.sleep()
