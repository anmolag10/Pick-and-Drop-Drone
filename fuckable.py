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

        self.spawnloc = np.array([19.0009248718, 71.9998318945, 22.16])

        # Numpy array for current GPS location
        self.currentloc = np.array([0.0, 0.0, 0.0])
        # Drone Command message of type edrone_cmd and initialisation
        self.setpoint_rpy = edrone_cmd()
        self.setpoint_rpy.rcRoll = 0
        self.setpoint_rpy.rcPitch = 0
        self.setpoint_rpy.rcYaw = 0
        self.setpoint_rpy.rcThrottle = 0

        # Numpy array for PID gains : [x, y, z] * coefficient ratio
        self.Kp = np.array([225, 225, 225]) * 0.6
        self.Ki = np.array([0, 0, 4]) * 0.008
        self.Kd = np.array([1265, 1265, 465]) * 0.3       

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

        #confirmation about the detection
        self.detectconf=False
        self.delivery_flag = 0
        self.del_error = np.array([0,0,0])
        self.currentlocxy=np.array([0,0,0])
        self.waypoint=np.array([0,0,0])
        self.pickuploc = np.array([19.0007046575, 71.9998955286, 22.16])
        self.dt = 0
        self.t = 0

        #Detected coordinates
        self.detectedcord=np.array([0.0,0.0,0.0])
        self.ranges=np.array([26,26,26,26,26])

        self.gripperState = False
        # Publishing /edrone/drone_command, /altitude_error, /latitude_error,
        # /longitude_error
        self.setpoint_pub = rospy.Publisher(
            '/edrone/drone_command', edrone_cmd, queue_size=1)
        self.altitude_error_pub = rospy.Publisher(
            '/altitude_error', Float32, queue_size=1)
        self.latitude_error_pub = rospy.Publisher(
            '/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher(
            '/longitude_error', Float32, queue_size=1)

        # Subscribing to /edrone/gps for current GPS location
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/detect_confirm',String, self.confirm_cordinates)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.lidar_callback)
        self.gripper_check = rospy.Subscriber(
            "/edrone/gripper_check", String, self.checkGripper)
        self.gripper_activate = rospy.ServiceProxy(
                        '/edrone/activate_gripper', Gripper)
        # ------------------------------------------------------------------------------------------------------------

    # Callback for GPS location
    def lidar_callback(self, msg):
        self.ranges = np.array(msg.ranges)
    
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def waypoint_generator(self, lat1, lon1, lat2, lon2, dist):
        x0, y0 = self.lat_to_x(lat1), self.long_to_y(lon1)
        x1, y1 = self.lat_to_x(lat2), self.long_to_y(lon2)
        d = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        self.dt = self.dt + dist if (self.dt + dist) < d else d
        t = self.dt / d
        waypoint = np.array([((1-t)*x0 + t*x1), ((1-t)*y0 + t*y1), 25.16])
        return waypoint, t

    # Function for checking limits of PID output
    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    def confirm_cordinates(self,data):
        strdata=data.data.split(',')
        if strdata[0]=='True':
            self.detectconf=True
        self.detectedcord[0]=strdata[1]
        self.detectedcord[1]=strdata[2]
        self.detectedcord[2]=strdata[3]
        
    def checkGripper(self, data):
        if data.data == "True":
            self.gripperState = True

    def pid(self):
	self.currentlocxy = np.array([self.lat_to_x(self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])

	self.del_error = np.round((self.waypoint - self.currentlocxy), 7)
	derivative = np.round(
            ((self.del_error - self.prev_values) / self.sample_time), 7)
	self.integral = np.round(
            ((self.integral + self.del_error) * self.sample_time), 7)
	output = np.round(
            ((self.Kp * self.del_error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)

	throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 - output[1])
        roll = self.checkLimits(1500.0 + output[0])

	self.prev_values = self.del_error

	self.setpoint_rpy.rcRoll = roll
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

    # PID algorithm
    def pickup(self):
        # Initially, GPS location is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.currentloc):
            return

        if abs(self.del_error[0]) < 1 and abs(self.del_error[1]) < 1 and abs(self.del_error[2]) < 0.1 and self.t != 1:
            self.waypoint, self.t = self.waypoint_generator(self.spawnloc[0], self.spawnloc[1], self.pickuploc[0], self.pickuploc[1], 12)

	self.pid()

	if self.t == 1:
            if abs(self.del_error[0]) > 0.1:
                return
            elif self.detectconf is not True:
                self.waypoint[2] = 22.86
            else:
                self.waypoint[2] = 22.16
                self.setpoint_rpy.rcThrottle = 1000
                self.setpoint_pub.publish(self.setpoint_rpy)
                gripper_response = self.gripper_activate(self.gripperState)
                if gripper_response.result is True:
                        self.delivery_flag = 1
                        self.prev_values = np.array([0,0,0])
                        self.integral = np.array([0,0,0])
                        self.t = 0

    def delivery(self):
        #if (self.ranges > 3).all():
        if abs(self.del_error[0]) < 1 and abs(self.del_error[1]) < 1 and abs(self.del_error[2]) < 0.1 and self.t != 1:
            self.waypoint, self.t = self.waypoint_generator(self.pickuploc[0], self.pickuploc[1], self.detectedcord[0], self.detectedcord[1], 25)

        #elif self.ranges[3] < 4:
        #    self.del_error = np.round(np.array([(2-self.ranges[3]), 1.5, (25.16-self.currentlocxy[2])]), 7)

        #elif self.ranges[4] < 4 :
        #    self.del_error = np.round(np.array([-1.5, (2-self.ranges[4]), (25.16-self.currentlocxy[2])]), 7)

	self.pid()

	if self.t == 1:
            if abs(self.del_error[0]) > 0.1:
                return
            elif self.waypoint[2]!=self.detectedcord[2]:
                self.waypoint[2] = self.detectedcord[2]
	    elif abs(self.del_error[2]) < 0.1:
		gripper_response = self.gripper_activate(False)
                
# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Calling PID function
        if e_drone_position.delivery_flag == 0:
            e_drone_position.pickup()
        else:
            e_drone_position.delivery()
        r.sleep()
