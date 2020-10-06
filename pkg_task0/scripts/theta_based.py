#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

theta=0

def turtle_pose(pose):
	global theta
	theta = pose.theta if pose.theta>=0 else pose.theta + 2*math.pi

def turtle_revolve():
	rospy.init_node('node_turtle_revolve', anonymous=True)

	rospy.Subscriber("turtle1/pose", Pose, turtle_pose)

	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
	vel = Twist()
	vel.linear.x = 1.0
	vel.angular.z = 1.0
	radius = vel.linear.x / vel.angular.z

	rate = rospy.Rate(100)

	while theta < 2*math.pi - 0.04: #
		rospy.loginfo("Moving in a circle\n"+ str(radius*theta))
		pub.publish(vel)
		rate.sleep()
	
	vel=Twist()
	pub.publish(vel)
	rospy.loginfo("goal reached")

	rospy.spin()

if __name__ == '__main__':
	turtle_revolve() 
