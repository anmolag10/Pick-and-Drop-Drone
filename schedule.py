#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import numpy as np
import pandas as pd
import os
from vitarana_drone.srv import Gripper
import math


class Scheduling():

	def __init__(self):
		rospy.init_node('node_schedule')

		self.file = pd.read_csv(os.path.expanduser(
			'~/catkin_ws/src/vitarana_drone/scripts/manifest.csv'), delimiter=",|;", engine="python", header = None)
		self.manifest = [list(row) for row in self.file.values]
		self.hash = dict(enumerate(self.manifest))

		self.WH = np.array([18.999873523, 72.000142461, 16.757981])
		self.deliveries = []
		self.returns = []

	# For convering latitude to X coordinate
	def lat_to_x(self, input_latitude):
		return 110692.0702932625 * (input_latitude - 19)

	# For converting longitude to Y coordinate
	def long_to_y(self, input_longitude):
		return -105292.0089353767 * (input_longitude - 72)

	def schedule_plan(self):
		x1 = self.lat_to_x(self.WH[0])
		y1 = self.long_to_y(self.WH[1])

		for i, m in enumerate(self.manifest):
			if m[0] == "DELIVERY":
				x2 = self.lat_to_x(float(m[2]))
				y2 = self.long_to_y(float(m[3]))
				d = math.hypot((x1-x2), (y1-y2))
				self.deliveries.append([i,d])
			else:
				x2 = self.lat_to_x(float(m[1]))
				y2 = self.long_to_y(float(m[2]))
				d = math.hypot((x1-x2), (y1-y2))
				self.returns.append([i,d])

		self.deliveries.sort(key = lambda x : x[1])
		self.returns.sort(key = lambda x : x[1])

if __name__ == '__main__':

    s = Scheduling()
    s.schedule_plan()
