#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist #Message Type required for turtle velocity
import math

def turtle_revolve():

	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1) #specifying topic where turtle velocity is to be published
	vel = Twist() #Initialising velocity message

	rospy.init_node('node_turtle_revolve', anonymous=True)	#Initialising node

	init_time = rospy.get_time() #Initial time before turtle starts moving

	while rospy.get_time() - init_time < (360/(1*180))*math.pi: #move while elapsed Time is less than the time required for the turtle to move in a circle at 1 rad/s

		rospy.loginfo("Moving in a circle\n"+ str(rospy.get_time() - init_time)) #Info for terminal

		vel.linear.x = 1.0 #Linear velocity 1 m/s
		vel.angular.z = 1.0 #Angular velocity 1 rad/s
		pub.publish(vel)  #Publishing velocity
	
	vel = Twist() #Reinitialising velocity message to 0 so turtle stops moving
	pub.publish(vel)
	
	rospy.loginfo("goal reached")
	exit()

if __name__ == '__main__':
	turtle_revolve() #Calling function to make turtle revolve
