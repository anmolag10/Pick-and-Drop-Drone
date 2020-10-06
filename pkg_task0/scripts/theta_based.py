#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

theta=0 #global variable so it can be accessed in turtle_revolve function

def turtle_pose_callback(pose):
	global theta
	theta = pose.theta if pose.theta>=0 else pose.theta + 2*math.pi #Makes arc angle theta range from 0 to 2*pi radians

def turtle_revolve():
	rospy.init_node('node_turtle_revolve', anonymous=True) #Initialising the node

	rospy.Subscriber("turtle1/pose", Pose, turtle_pose_callback) #Subscribing to the pose of the turtle

	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10) #Specifying the topic where velocity is to be published
	vel = Twist() #Initialising velocity message
	vel.linear.x = 1.0 #Assigning 1 m/s as linear velocity 
	vel.angular.z = 1.0 #Assigning 1 rad/s as angular velocity
	radius = vel.linear.x / vel.angular.z 

	rate = rospy.Rate(100)

	while theta < 2*math.pi - 0.02: #Move while angle of arc travelled is less than 2*pi radians (considering a buffer of 0.02 radians to make sure turtle stops at the appropriate position)
		rospy.loginfo("Moving in a circle\n"+ str(radius*theta)) #Printing distance info on terminal 
		pub.publish(vel) #Publishing velocity
		rate.sleep()
	
	vel=Twist() #Reinitialising velocity message to 0 so that the turtle stops
	pub.publish(vel)
	rospy.loginfo("goal reached")

	rospy.spin()

if __name__ == '__main__':
	turtle_revolve() #Calling the function to make the turtle revolve
