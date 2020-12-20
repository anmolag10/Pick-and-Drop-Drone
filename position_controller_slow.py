#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import numpy as np
from vitarana_drone.srv import Gripper
import math
import time


flag = 0
class Position():

	def __init__(self):
		# initializing ros node with name position_controller
		rospy.init_node('node_position_controller')

		# Location at which the drone spawns
		self.spawnloc = np.array([0, 0, 0])
		# Location of package to pickup
		self.pickuploc = np.array([18.9990965928,72.0000664814, 11.75])
		# Numpy array for current GPS location
		self.currentloc = np.array([0.0, 0.0, 0.0])
		# Current Location of drone in XYZ coordinate system
		self.currentlocxy = np.array([0, 0, 0])
		# Drone Command message of type edrone_cmd and initialisation
		self.setpoint_rpy = edrone_cmd()
		self.setpoint_rpy.rcRoll = 0
		self.setpoint_rpy.rcPitch = 0
		self.setpoint_rpy.rcYaw = 0
		self.setpoint_rpy.rcThrottle = 0

		self.side = 0
		self.iterator = 0
		self.counter = 0

		# Numpy array for PID gains : [x, y, z] * coefficient ratio
		self.Kp = np.array([325, 325, 225]) * 0.6
		self.Ki = np.array([0, 0, 4]) * 0.008
		self.Kd = np.array([1825, 1825, 465]) * 0.3

		# For storing error term for PID
		self.error = np.array([0, 0, 0])
		# For storing previous error for derivative term
		self.prev_values = np.array([0.0, 0.0, 0.0])
		# For storing sum of error for integral term
		self.integral = np.array([0.0, 0.0, 0.0])
		# Maximum and Minimum values for roll, pitch and throttle commands
		self.max_value = 2000
		self.min_value = 1000

		# PID sampling rate and time
		self.sample_rate = 10  # in Hz
		self.sample_time = 0.1  # in seconds

		# Flag to confirm detection
		self.detectconf = False
		# Flag to switch state from pickup to delivery
		self.start_detection_flag = 0
		self.detection_flag = 0
		# Waypoint for trajectory
		self.waypoint = np.array([0, 0, 0])
		# Other variables needed to generate waypoints
		self.dt = 0
		self.t = 0
		self.flag=0

		# Detected coordinate from QR code
		self.detectedcoord = [0.0, "0.0", "0.0"]
		# Ranges from range finder top
		self.ranges = np.array([26, 26, 26, 26, 26])

		# To check gripper state from callback
		self.gripperState = False

		# Publishing /edrone/drone_command
		self.setpoint_pub = rospy.Publisher(
			'/edrone/drone_command', edrone_cmd, queue_size=1)

		# Subscribers
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/range_finder_top',LaserScan, self.laser_callback)
		# ------------------------------------------------------------------------------------------------------------

	# Callback for Laser sensor ranges
	def laser_callback(self, msg):
		self.ranges = np.array(msg.ranges)

	# Callback for GPS location
	def gps_callback(self, msg):
		if self.flag==0 and (msg.latitude is not 0):
			self.spawnloc =  np.array([msg.latitude, msg.longitude, msg.altitude])
			self.flag=1
		self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

	# For convering latitude to X coordinate
	def lat_to_x(self, input_latitude):
		return 110692.0702932625 * (input_latitude - 19)

	# For converting longitude to Y coordinate
	def long_to_y(self, input_longitude):
		return -105292.0089353767 * (input_longitude - 72)

	# For generating waypoints between (lat1, long1) and (lat2, long2)
	# dist is the distance between waypoints / step size
	def waypoint_generator(self, lat1, lon1, lat2, lon2, dist):
		x0, y0 = self.lat_to_x(lat1), self.long_to_y(lon1)
		x1, y1 = self.lat_to_x(lat2), self.long_to_y(lon2)
		# Distance between coordinates
		d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
		# Distance of next waypoint from (lat1, long1)
		self.dt = self.dt + dist if (self.dt + dist) < d else d
		# Ratio of waypoint distance to total distance (is equal to 1 when at
		# final goal)
		self.t = self.dt / d
		# Waypoint [x,y,z], z = 25.16 which is 3 m above the spawn location
		waypoint = np.array(
			[((1 - self.t) * x0 + self.t * x1), ((1 - self.t) * y0 + self.t * y1), 25.16])
		return waypoint

	# Function for checking limits of PID output
	def checkLimits(self, drone_command):
		if drone_command > self.max_value:
			return self.max_value
		elif drone_command < self.min_value:
			return self.min_value
		else:
			return drone_command

	# Callback for detected coordinates
	def confirm_cordinates(self, data):
		strdata = data.data.split(',')
		if strdata[0] == 'True' :
			self.detectconf = True
			self.detectedcoord[1] = strdata[1]
			self.detectedcoord[2] = strdata[2]

	def Search_pattern(self):
		if(self.iterator % 2 == 0):
			self.side += 4
		self.iterator += 1

		if self.counter == 0:
			self.waypoint[2] = self.pickuploc[2] + 4
		if self.counter % 4 == 0:
			self.waypoint[1] = self.currentlocxy[1] + self.side
			print("move up {}m".format(self.side))
		elif self.counter % 4 == 1:
			self.waypoint[0] = self.currentlocxy[0] + self.side
			print("move right {}m".format(self.side))
		elif self.counter % 4 == 2:
			self.waypoint[1] = self.currentlocxy[1] - self.side
			print("move down {}m".format(self.side))
		elif self.counter % 4 == 3:
			self.waypoint[0] = self.currentlocxy[0] - self.side
			print("move left {}m".format(self.side))
		self.counter += 1


	# Function for PID control
	def pid(self):
		# Calculating XYZ coordinates
		self.currentlocxy = np.array([self.lat_to_x(
		self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])
		# Calculating error, derivative and integral
		self.error = np.round((self.waypoint - self.currentlocxy), 7)
		derivative = np.round(
		((self.error - self.prev_values) / self.sample_time), 7)
		self.integral = np.round(
		((self.integral + self.error) * self.sample_time), 7)
		# PID output
		output = np.round(
		((self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)
		# Final values for publishing after checking limits
		throttle = self.checkLimits(1500.0 + output[2])
		pitch = self.checkLimits(1500.0 - output[1])
		roll = self.checkLimits(1500.0 + output[0])
		# Assigning previous value with error for next iteration
		self.prev_values = self.error
		# Publishing the commands
		self.setpoint_rpy.rcRoll = roll
		self.setpoint_rpy.rcPitch = pitch
		self.setpoint_rpy.rcYaw = 1500
		self.setpoint_rpy.rcThrottle = throttle
		self.setpoint_pub.publish(self.setpoint_rpy)

	# Function for pickup state
	def pickup(self):
		# Initially, GPS location is not updated,i.e., [0,0,0]
		# To avoid undesired motion due to this the following is used.
		# It returns to the main function if any values are 0
		if not np.any(self.currentloc):
			return


		if abs(self.error[0]) < 1 and abs(self.error[1]) < 1 and abs(self.error[2]) < 0.1 and self.t != 1:
			self.waypoint = self.waypoint_generator(self.spawnloc[0], self.spawnloc[1], self.pickuploc[0], self.pickuploc[1], 5)

		# Call PID function for publishing control commands

		elif abs(self.error[0]) > 0.1 and abs(self.error[1]) > 0.1 and abs(self.error[2]) > 0.1 and self.t == 1:
			pass

		# self.t == 1 implies that the current setpoint is the final goal
		elif self.t == 1:
			if abs(self.error[0]) > 0.1 and abs(self.error[1]) > 0.1 and abs(self.error[2]) > 0.1:
				return

			elif abs(self.waypoint[2] - 25.16) < 0.1:
				self.waypoint[2] = self.pickuploc[2]

			# Hover 0.7 m above until coordinates on box are detected
			elif abs(self.error[2]) < 0.1:
				self.detectconf = False
				self.detectedcoord = [0,"0.0","0.0"]
				self.start_detection_flag = 1
				self.detection_flag = 0

		self.pid()
		

	def detection(self):
		global flag
		print(self.detectconf, self.detection_flag, self.detectedcoord[1])
		if ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(self.error[2]) < 0.1)) and self.detectconf is False:
			rospy.Subscriber('/detect_confirm', String, self.confirm_cordinates)
			self.Search_pattern()

		elif self.detectconf is True and self.detection_flag == 0 and self.detectedcoord[1]!="inf" and self.detectedcoord[1]!="-inf" and self.detectedcoord[1]!='0.0':
			print(self.detectedcoord)
			self.waypoint[0] = self.currentlocxy[0] + float(self.detectedcoord[1])
			self.waypoint[1] = self.currentlocxy[1] + float(self.detectedcoord[2])
			self.detection_flag = 1

		elif ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(self.error[2]) < 0.1)):

			self.t = 0
			self.dt = 0
			self.detectconf = False
			self.spawnloc = self.currentloc
			self.start_detection_flag = 0
			self.counter = 0
			self.side = 4
			if flag == 0:
				print("build 2")
				self.pickuploc = np.array([18.9990965925, 71.9999050292, 23.2])
				flag += 1
			elif flag == 1:
				print("build 3")
				self.pickuploc = np.array([18.9993675932, 72.0000569892, 11.7])
				flag += 1
			elif flag == 2:
				#some error will come up here but i just want to go down after reaching.
				self.waypoint[2] = self.currentlocxy -2
		self.pid()
# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

	e_drone_position = Position()
	# Defining rospy rate such that PID algorithm loops at the
	# desired sampling rate
	r = rospy.Rate(e_drone_position.sample_rate)
	while not rospy.is_shutdown():
		# Call pickup function if delivery flag is 0 else call delivery
		# function
		if e_drone_position.start_detection_flag == 0:
			e_drone_position.pickup()
		else:
			e_drone_position.detection()
		r.sleep()