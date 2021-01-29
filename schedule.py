#!/usr/bin/env python

# Importing the required libraries
from std_msgs.msg import String
import rospy
import numpy as np
import pandas as pd
import os
import math

class Scheduling():

	def __init__(self):
		rospy.init_node('node_schedule')

		self.file = pd.read_csv(os.path.expanduser(
			'~/catkin_ws/src/vitarana_drone/scripts/manifest.csv'), delimiter=",", engine="python", header = None)
		self.manifest = [list(row) for row in self.file.values]
		self.hash = dict(enumerate(self.manifest))

		self.file = pd.read_csv(os.path.expanduser(
			'~/catkin_ws/src/vitarana_drone/scripts/manifest.csv'), delimiter=",|;", engine="python", header = None)
		self.manifest = [list(row) for row in self.file.values]

		self.WH = np.array([18.999873523, 72.000142461, 16.757981])
		self.deliveries = []
		self.returns = []
		self.paired = []

		self.length_d = 0
		self.length_r = 0

	# For convering latitude to X coordinate
	def lat_to_x(self, input_latitude):
		return 110692.0702932625 * (input_latitude - 19)

	# For converting longitude to Y coordinate
	def long_to_y(self, input_longitude):
		return -105292.0089353767 * (input_longitude - 72)

	def schedule_plan(self):
		# Warehouse location
		x1 = self.lat_to_x(self.WH[0])
		y1 = self.long_to_y(self.WH[1])

		# Making lists of delivery and return with index from original manifest, distance from warehouse and x, y coordinates
		for i, m in enumerate(self.manifest):
			if m[0] == "DELIVERY":
				x2 = self.lat_to_x(float(m[2]))
				y2 = self.long_to_y(float(m[3]))
				dist = math.hypot((x1-x2), (y1-y2))
				self.deliveries.append([i,dist,x2,y2])
			else:
				x2 = self.lat_to_x(float(m[1]))
				y2 = self.long_to_y(float(m[2]))
				dist = math.hypot((x1-x2), (y1-y2))
				self.returns.append([i,dist,x2,y2])

		# Sorting lists according to distances 
		self.deliveries.sort(key = lambda x : x[1])
		self.returns.sort(key = lambda x : x[1])

		self.length_d = len(self.deliveries)
		self.length_r = len(self.returns)

		# Pairing deliveries with closest returns
		for d in self.deliveries:
			closest_dist = 1000
			index = 0
			closest_ret = 0
			for i, r in enumerate(self.returns):
				dist = math.hypot((r[2]-d[2]), (r[3]-d[3]))
				if dist < closest_dist:
					closest_dist = dist
					index = i
					closest_ret = r[0]
			if closest_dist < 1000:
				self.returns.remove(self.returns[index])
				self.paired.append([d[0], closest_ret])
			else:
				self.paired.append([d[0], -1])

		'''with open('sequenced_manifest.csv', 'w', newLine=" ") as f:
			w = csv.writer(f)
			if self.length_d >= self.length_r:
				for i in self.paired:
					if(i[1] > -1):
						w.writerow([])'''
			

if __name__ == '__main__':

    s = Scheduling()
    s.schedule_plan()
