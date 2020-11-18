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

        # Numpy array for GPS coordinate setpoints
        self.setpoints = np.array(
        [[19.0009248718, 71.9998318945, 25.16], [19.0007046575, 71.9998955286, 25.16], [19.0007046575, 71.9998955286, 22.86], [19.0007046575, 71.9998955286, 22.16],[19.0009248718, 71.9998318945, 25.16]])

        # Numpy array for current GPS location
        self.currentloc = np.array([0.0, 0.0, 0.0])
        # Index of GPS setpoint that the drone is headed towards
        self.loc = 0
        # Count for Stability
        self.stabilize = 0
        # Drone Command message of type edrone_cmd and initialisation
        self.setpoint_rpy = edrone_cmd()
        self.setpoint_rpy.rcRoll = 0
        self.setpoint_rpy.rcPitch = 0
        self.setpoint_rpy.rcYaw = 0
        self.setpoint_rpy.rcThrottle = 0

        # Numpy array for PID gains : [Latitude, Longitude, Altitude]
        # Coefficient ratios for Pid[Latitude] [Kp, Ki, Kd] : [6000, 0.08, 12000]
        # Coefficient ratios for Pid[Longitude] [Kp, Ki, Kd] : [6000, 0.08, 12000]
        # Coefficient ratios for Pid[Altitude] [Kp, Ki, Kd] : [0.6, 0.008, 0.3]

        # Value of [Kp, Ki, Kd][Latitude] : [720, 55, 1750]
        # Value of [Kp, Ki, Kd][Longitude] : [720, 55, 1650]
        # Value of [Kp, Ki, Kd][Altitude] : [225, 4, 465]
        self.Kp1 = np.array([720 * 6000, 720 * 6000, 225 * 0.6])
        self.Ki1 = np.array([55 * 0.008, 55 * 0.008, 4 * 0.008])
        self.Kd1 = np.array([1750 * 12000, 1750 * 12000, 465 * 0.3])

	self.Kp2 = np.array([225 * 0.6, 225 * 0.6, 225 * 0.6])
        self.Ki2 = np.array([0 * 0.008, 0 * 0.008, 4 * 0.008])
        self.Kd2 = np.array([2265 * 0.3, 1665 * 0.3, 465 * 0.3])

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
	self.yaw_error = 0
	self.deliver = 0
	self.theta = 0
	self.theta1 = 0
	self.ranges = np.array([0,0,0,0,0])

        #Detected coordinates
        self.detectedcoord=np.array([0.0,0.0,0.0])

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
	rospy.Subscriber('/yaw_error', Float32, self.yaw_error_callback)
	rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.lidar_callback)
        self.gripper_check = rospy.Subscriber(
            "/edrone/gripper_check", String, self.checkGripper)
        self.gripper_activate = rospy.ServiceProxy(
                        '/edrone/activate_gripper', Gripper)
        # ------------------------------------------------------------------------------------------------------------

    # Callback for GPS location
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

    def yaw_error_callback(self, msg):
	self.yaw_error = msg.data

    def lidar_callback(self, msg):
	self.ranges = np.array(msg.ranges)

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
        self.detectedcoord[0]=strdata[1]
        self.detectedcoord[1]=strdata[2]
        self.detectedcoord[2]=strdata[3]
        
    def checkGripper(self, data):
        if data.data == "True":
            self.gripperState = True
       
    def lat_to_x(self, input_latitude):
	return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
	return -105292.0089353767 * (input_longitude - 72)
    # if you use this for control, you may have to change the relevant pitch   direction because of the sign
    
    def align_to_goal(self):
	X = (self.lat_to_x(self.setpoints[5,0])-self.lat_to_x(self.currentloc[0]))
	Y = (self.long_to_y(self.setpoints[5,1])-self.long_to_y(self.currentloc[1]))
        theta = math.degrees(math.atan2(Y, X))
        theta = theta if theta >= 0 else theta + 360
	self.theta = theta
        theta = (theta + 270) % 360
        theta = theta if theta < 180 else (theta - 360)
        theta = (theta + 540) / 0.36
	self.theta1 = theta
        return theta

    def distance(self):
	X = (self.lat_to_x(self.setpoints[5,0])-self.lat_to_x(self.currentloc[0]))
	Y = (self.long_to_y(self.setpoints[5,1])-self.long_to_y(self.currentloc[1]))
	goal_distance = X * math.sin(self.theta) + Y * math.cos(self.theta)
	roll_error = X * math.cos(self.theta) - Y * math.sin(self.theta)
	return goal_distance, roll_error

    # PID algorithm
    def pickup(self):
        # Initially, GPS location is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.currentloc):
            return

        # Calculating error term and rounding off to 7 decimal points
        # Rounding off because double precision is overkill for PID
        error = np.round((self.setpoints[self.loc] - self.currentloc), 7)

        # Calculating derivative term and rounding off
        # / symbol allows divison for sample_time scalar with every element in the array
        derivative = np.round(
            ((error - self.prev_values) / self.sample_time), 7)

        # Calculating integral term and rounding off
        # * symbol allows multiplication for sample_time scalar with every element in the array
        self.integral = np.round(
            (self.integral + error * self.sample_time), 7)

        # Calculating PID output and rounding off
        # * symbol for numpy arrays allows multiplication of the corresponding elements
        # output = [out_latitude, out_longitude, out_altitude]
        output = np.round(
            ((self.Kp1 * error) + (self.Ki1 * self.integral) + (self.Kd1 * derivative)), 7)

        # Calculating throttle, roll and pitch and checking limits
        # Since 1500 is the mid value,
        # Direction will change according to sign of output
        throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 + output[1])
        roll = self.checkLimits(1500.0 + output[0])

        # Assigning prev_values with error for the next iteration
        self.prev_values = error

        # Publishing errors for Plotjuggler
        self.latitude_error_pub.publish(error[0])
        self.longitude_error_pub.publish(error[1])
        self.altitude_error_pub.publish(error[2])

        # Publishing final PID output on /edrone/drone_command for the attitude
        # controller
	if self.loc < 4:
            self.setpoint_rpy.rcRoll = roll
            self.setpoint_rpy.rcPitch = pitch
            self.setpoint_rpy.rcYaw = 1500
            self.setpoint_rpy.rcThrottle = throttle
            self.setpoint_pub.publish(self.setpoint_rpy)
	
	if self.loc == 4 and (abs(self.yaw_error) > 0.1 or self.stabilize == 0):
	    self.setpoint_rpy.rcRoll = 1500
            self.setpoint_rpy.rcPitch = 1500
            self.setpoint_rpy.rcYaw = self.align_to_goal()
            self.setpoint_rpy.rcThrottle = throttle
            self.setpoint_pub.publish(self.setpoint_rpy)
	    self.stabilize = 1
	    return 

	elif self.loc == 4 and abs(self.yaw_error) < 0.1:
	    self.deliver = 1
	    self.prev_values = np.array([0.0, 0.0, 0.0])
            self.integral = np.array([0.0, 0.0, 0.0])  
	    print("deliver")
	    return

        # Checking condition for threshold box of setpoints
        if abs(
            error[1]) < 0.0000047487 and abs(
            error[0]) < 0.000004517 and abs(
                error[2]) < 0.2:
            # Checking if drone has reached threshold box of final setpoint
            # Publishing stop signal (throttle = 1000) for drone to land
            if self.loc == 2:
		if self.stabilize < 45:
                    self.stabilize = self.stabilize + 1
                    return
                elif self.detectconf is not True:
                    print('Hovering for detection') 
                    return
                else:
                    print('detection complete')
		    self.setpoints = np.vstack((self.setpoints, self.detectedcoord))
                    print(self.detectedcoord)
                    
            if self.loc == 3:
                self.setpoint_rpy.rcThrottle = 1000
                self.setpoint_rpy.rcRoll = 1500
                self.setpoint_rpy.rcPitch = 1500
                self.setpoint_rpy.rcYaw = 1500
                self.setpoint_pub.publish(self.setpoint_rpy)
                gripper_response = self.gripper_activate(self.gripperState)
                if self.stabilize < 30:
                    self.stabilize = self.stabilize + 1
                    return
                print('GripperState:'+str(self.gripperState)+str(gripper_response))

            if self.stabilize < 10:
                self.stabilize = self.stabilize + 1
                return
            # Increasing value of location index so that next GPS setpoint is
            # set
            self.loc = self.loc + 1
            # Reinitialising stabilize to 0 so it can be used for next setpoint
            self.stabilize = 0

    def delivery(self):
	print(self.ranges[0])
	if self.ranges[0] > 4:
		goal_distance, roll_error = self.distance()
		error = np.round(np.array([goal_distance, roll_error, (self.setpoints[4,2] - self.currentloc[2])]), 7)
		derivative = np.round(
		    ((error - self.prev_values) / self.sample_time), 7)
		self.integral = np.round(
		    (self.integral + error * self.sample_time), 7)
		output = np.round(
		    ((self.Kp2 * error) + (self.Ki2 * self.integral) + (self.Kd2 * derivative)), 7)
		throttle = self.checkLimits(1500.0 + output[2])
		pitch = self.checkLimits(1500.0 - output[0])
		roll = self.checkLimits(1500.0 + output[1])
		self.prev_values = error
		self.setpoint_rpy.rcThrottle = throttle
		self.setpoint_rpy.rcRoll = 1500
		self.setpoint_rpy.rcPitch = pitch
		self.setpoint_rpy.rcYaw = self.theta1
		self.setpoint_pub.publish(self.setpoint_rpy)

	else:
		error = np.round(np.array([(1-self.ranges[0]), 0, (self.setpoints[4,2] - self.currentloc[2])]), 7)
		derivative = np.round(
		    ((error - self.prev_values) / self.sample_time), 7)
		self.integral = np.round(
		    (self.integral + error * self.sample_time), 7)
		output = np.round(
		    ((self.Kp2 * error) + (self.Ki2 * self.integral) + (self.Kd2 * derivative)), 7)
		throttle = self.checkLimits(1500.0 + output[2])
		pitch = self.checkLimits(1500.0 - output[0])
		self.prev_values = error
		self.setpoint_rpy.rcThrottle = throttle
		self.setpoint_rpy.rcRoll = 1495
		self.setpoint_rpy.rcPitch = pitch
		self.setpoint_rpy.rcYaw = self.theta1
		self.setpoint_pub.publish(self.setpoint_rpy)
# ------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Calling PID function
	if e_drone_position.deliver == 0:
        	e_drone_position.pickup()
	else:
		e_drone_position.delivery()
        r.sleep()
