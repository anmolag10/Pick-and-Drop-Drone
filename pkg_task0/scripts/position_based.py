#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

flag = 0 
final_x = 0
final_y = 0
x = 0
y = 0

def turtle_pose(pose):
    global final_x, final_y, x, y, flag
    
    x = round(pose.x, 1) 
    y = round(pose.y, 1)

    if flag == 0:
        time.sleep(0.5)
        final_x = x
        final_y = y
        flag = 1

def turtle_revolve():
	rospy.init_node('node_turtle_revolve', anonymous=True)

	rospy.Subscriber("/turtle1/pose", Pose, turtle_pose)

	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
	vel = Twist()

	rate = rospy.Rate(100)
	init_time = rospy.get_time()

	while not rospy.is_shutdown():
	    
	    if final_x == x and final_y == y:
		if(flag == 1):
		    vel = Twist()
		    pub.publish(vel)
		    rospy.loginfo("goal reached")
		    break
		
	    rospy.loginfo("Moving in a circle\n"+ str(rospy.get_time() - init_time))
	    vel.linear.x = 2.0
	    vel.angular.z = 2.0
	    pub.publish(vel)

	    rate.sleep()

if __name__ == '__main__':
	turtle_revolve() 
