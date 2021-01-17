#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import numpy as np
from vitarana_drone.srv import Gripper
import math


class Position():

    def __init__(self):
        # initializing ros node with name position_controller
        rospy.init_node('node_position_controller')

	self.spawnloc = np.array([0, 0, 0])
        # GPS coordinates of buildings with height 1m more than specified
	self.buildingloc = np.array([[18.9993675932,
                                      72.0000569892,
                                      10.7 + 1],
                                     [18.9990965925,
                                      71.9999050292,
                                      22.2 + 1],
                                     [18.9990965928,
                                      72.0000664814,
                                      10.75 + 1]])
        # Location of package to pickup
        self.pickuploc = np.array([19.0007046575, 71.9998955286, 22.16])

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

        # Parameters required for the search pattern
        self.side = 0
        self.iterator = 0
        self.building_flag = 0
        self.flag = 0

        # Parameters required for PID
        self.Kp = np.array([325, 325, 225]) * 0.6
        self.Ki = np.array([0, 0, 4]) * 0.008
        self.Kd = np.array([1625, 1625, 465]) * 0.3
        self.error = np.array([0, 0, 0])
        self.prev_values = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        self.max_value = 2000
        self.min_value = 1000
        self.sample_rate = 10  # in Hz
        self.sample_time = 0.1  # in seconds

        # Flag to confirm detection
        self.detectconf = False
        # Info for detection
        self.detectedcoord = [0.0, "0.0", "0.0"]
        # Flag to switch state from building seek to detection
        self.start_detection_flag = 0
        # Flag to consider only one detection of marker
        self.detection_flag = 0
        # Waypoint for trajectory
        self.waypoint = np.array([0, 0, 0])
        # Other variables needed to generate waypoints
        self.dt = 0
        self.t = 0
        # Flag to check whether drone is in avoid state or goal seeking state
        self.avoid_flag = 0

        # Ranges from range finder top
        self.ranges = np.array([26, 26, 26, 26, 26])

        # To check gripper state from callback
        self.gripperState = False

        # Publishing /edrone/drone_command
        self.setpoint_pub = rospy.Publisher(
            '/edrone/drone_command', edrone_cmd, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/detect_confirm', String, self.confirm_cordinates)
        rospy.Subscriber(
            '/edrone/range_finder_top',
            LaserScan,
            self.laser_callback)
        self.gripper_check = rospy.Subscriber(
            "/edrone/gripper_check", String, self.checkGripper)
        self.gripper_activate = rospy.ServiceProxy(
            '/edrone/activate_gripper', Gripper)
        # ------------------------------------------------------------------------------------------------------------

    # Callback for Laser sensor ranges
    def laser_callback(self, msg):
        self.ranges = np.array(msg.ranges)

    # Callback for GPS location
    def gps_callback(self, msg):
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
            [((1 - self.t) * x0 + self.t * x1), ((1 - self.t) * y0 + self.t * y1), 0])
        return waypoint

    # Function for checking limits of PID output
    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    # Callback for detected marker information
    def confirm_cordinates(self, data):
        strdata = data.data.split(',')
        if strdata[0] == 'True':
            self.detectconf = True
            self.detectedcoord[1] = strdata[1]
            self.detectedcoord[2] = strdata[2]

    # Square spiral search pattern that starts with a distance of 5 m at a
    # height of 10 m
    def Search_pattern(self):
        if(self.iterator % 2 == 0):
            self.side += 5

        if self.iterator == 0:
            self.waypoint[2] = self.buildingloc[self.building_flag][2] + 10

        elif self.iterator % 4 == 0:
            self.waypoint[0] = self.currentlocxy[0] - self.sider

            print("move left {}m".format(self.side))
        elif self.iterator % 4 == 1:
            self.waypoint[1] = self.currentlocxy[1] + self.side
            print("move up {}m".format(self.side))

        elif self.iterator % 4 == 2:
            self.waypoint[0] = self.currentlocxy[0] + self.side
            print("move right {}m".format(self.side))

        elif self.iterator % 4 == 3:
            self.waypoint[1] = self.currentlocxy[1] - self.side
            print("move down {}m".format(self.side))

        self.iterator += 1

    # Callback for gripper state
    def checkGripper(self, data):
        if data.data == "True":
            self.gripperState = True

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

    # Function for delivery state with left wall following bug
    def delivery(self):
	if not np.any(self.currentloc):
            return

	else:
	    self.spawnloc[0] = np.array([self.lat_to_x(
            	self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])

        # If obstacle detcted by left laser, set avoid flag
        if self.ranges[3] < 5 and self.ranges[3] > 0.3:
            if self.avoid_flag == 0:
                self.avoid_flag = 1
                self.prev_values = np.array([0, 0, 0])
            # Move backward by 1 m and maintain 3.5 m from the wall
            self.waypoint[1] = self.currentlocxy[1] - 1
            self.waypoint[0] = self.currentlocxy[0] + (3.5 - self.ranges[3])

        # If obstacle detcted by front laser, set avoid flag
        elif self.ranges[0] < 5 and self.ranges[0] > 0.3:
            if self.avoid_flag == 0:
                self.avoid_flag = 1
                self.prev_values = np.array([0, 0, 0])
            # Move left by 1 m and maintain 3.5 m from the wall
            self.waypoint[0] = self.currentlocxy[0] - 1
            self.waypoint[1] = self.currentlocxy[1] - (3.5 - self.ranges[0])

        # If obstacle detcted by right laser, set avoid flag
        elif self.ranges[1] < 5 and self.ranges[1] > 0.3:
            if self.avoid_flag == 0:
                self.avoid_flag = 1
                self.prev_values = np.array([0, 0, 0])
            # Move front by 1 m and maintain 3.5 m from the wall
            self.waypoint[1] = self.currentlocxy[1] + 1
            self.waypoint[0] = self.currentlocxy[0] - (3.5 - self.ranges[3])

        # If obstacle detcted by back laser, set avoid flag
        elif self.ranges[2] < 5 and self.ranges[2] > 0.3:
            if self.avoid_flag == 0:
                self.avoid_flag = 1
                self.prev_values = np.array([0, 0, 0])
            # Move right by 1 m and maintain 3.5 m from the wall
            self.waypoint[0] = self.currentlocxy[0] + 1
            self.waypoint[1] = self.currentlocxy[1] + (3.5 - self.ranges[2])

        # Main waypoint generating condition if no obstacle
        else:
            if ((abs(self.error[0]) < 1 and abs(self.error[1]) < 1 and abs(
                    self.error[2]) < 0.1) or self.avoid_flag == 1) and self.t != 1:
                # If previous state was avoidance then reset the location from which waypoints should be generated
                # Also reset the avoid flag
                if self.avoid_flag == 1:
                    self.pickuploc = self.currentloc
                    self.dt = 0
                    self.t = 0
                    self.avoid_flag = 0
                # Generating waypoints to final goal with step size 18
		elif self.delivery_flag == 1:
		        self.waypoint = self.waypoint_generator(
		            self.pickuploc[self.building_flag][0],
		            self.pickuploc[self.building_flag][1],
		            self.building_loc[self.building_flag][0],
		            self.building_loc[self.building_flag][1],
		            18)
			if self.pickuploc[self.building_flag][2] > self.building_loc[self.building_flag][2]:
				self.waypoint[2] = self.pickuploc[self.building_flag][2] + 3
			else:
				self.waypoint[2] = self.building_loc[self.building_flag][1] + 3

		elif self.delivery_flag == 0:
			self.waypoint = self.waypoint_generator(
			    self.spawnloc[0]
			    self.spawnloc[1]
		            self.pickuploc[self.building_flag][0],
		            self.pickuploc[self.building_flag][1],
		            18)
			if self.pickuploc[self.building_flag][2] > self.spawnloc[2]:
				self.waypoint[2] = self.pickuploc[self.building_flag][2] + 3
			else:
				self.waypoint[2] = self.spawnloc[2] + 3

        # Call PID function for publishing control commands
        self.pid()

        # self.t == 1 implies that the current setpoint is the final goal
        if self.t == 1 and self.delivery_flag == 1:
                # Reach goal with minimum error
            if abs(self.error[0]) > 0.1 or abs(self.error[1]) > 0.1:
                return
            self.start_detection_flag = 1

	if self.t == 1 and self.delivery_flag == 0:
       	    	# Reach goal with minimum error
	   	if abs(self.error[0]) > 0.1:
	    		return
	    	# Hover 0.7 m above until coordinates on box are detected
	    	elif self.detectconf is not True:
	    		self.waypoint[2] = self.pickuploc[2] + 0.7
	    		# After detecting, drop on box and grip it
	    	else:
	    		self.setpoint_rpy.rcThrottle = 1000
	    		self.setpoint_pub.publish(self.setpoint_rpy)
	    		gripper_response = self.gripper_activate(self.gripperState)
	    		# Set the delivery_flag and reinitialise some parameters to
	    		# generate new waypoints
	    		if gripper_response.result is True:
				self.delivery_flag = 1
				self.error = np.array([0, 0, 0])
				self.dt = 0
				self.prev_values = np.array([0, 0, 0])
				self.integral = np.array([0, 0, 0])
				self.t = 0

    # Function for detection state
    def detection(self):
        # As soon as drone switches to detection state, start search
        if ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1 and abs(
                self.error[2]) < 0.1)) and self.detectconf is False:
            self.Search_pattern()

        # If detected, seek marker and drop to 5m above building for better
        # detection
        elif self.detectconf is True and self.detectedcoord[1] != "inf" and self.detectedcoord[1] != "-inf" and self.detectedcoord[1] != '0.0':
            self.waypoint[0] = self.currentlocxy[0] + \
                float(self.detectedcoord[1])
            self.waypoint[1] = self.currentlocxy[1] + \
                float(self.detectedcoord[2])
            self.waypoint[2] = self.buildingloc[self.building_flag][2] + 5

        # After reaching marker
        elif ((abs(self.error[0]) < 0.1 and abs(self.error[1]) < 0.1)):
            if self.waypoint[2] != self.building_loc[self.building_flag][2]:
                self.waypoint[2] = self.building_loc[self.building_flag][2]
            # When waypoint is within error threshold, deactivate gripper
            # Also switch off propellers
            elif abs(self.error[2]) < 0.2:
                gripper_response = self.gripper_activate(False)
                self.setpoint_rpy.rcThrottle = 1000
                self.setpoint_pub.publish(self.setpoint_rpy)
		self.start_detection_flag = 0
		self.spawnloc = self.currentlocxy
		# Reinitialising parameters for next building
		self.delivery_flag = 0
		self.start_detection_flag = 0
		self.iterator = 0
		self.side = 0
		self.building_flag += 1

        # Calling PID function
        self.pid()

        # Publish info to marker_related topic so it can be republished at 1Hz
        # through detection script
        if self.detection_flag == 1:
            # 1st and 3rd building have been flipped in self.building_loc
            curr_id = self.marker_id[self.building_flag]
            self.marker_related_pub.publish(
                "True," + str(curr_id) + "," + str(self.error[0]) + "," + str(self.error[1]))
        else:
            curr_id = self.marker_id[self.building_flag]
            self.marker_related_pub.publish("False," + str(curr_id))

# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Call pickup function if delivery flag is 0 else call delivery
        # function
        if e_drone_position.self.start_detection_flag == 0:
            e_drone_position.pickup()
        else:
            e_drone_position.detection()
        r.sleep()
