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
        [[19.0009248718, 71.9998318945, 25.16], [19.0007046575, 71.9998955286, 25.16], [19.0007046575, 71.9998955286, 22.86], [19.0007046575, 71.9998955286, 22.16],[19.0007046575, 71.9998955286, 25.16]])

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
        self.Kp = np.array([720 * 6000, 720 * 6000, 225 * 0.6])
        self.Ki = np.array([55 * 0.08, 55 * 0.08, 4 * 0.008])
        self.Kd = np.array([1650 * 12000, 1650 * 12000, 465 * 0.3])

        self.Kp2 = np.array([225 * 0.6, 225 * 0.6, 225 * 0.6])
        self.Ki2 = np.array([0 * 0.008, 0 * 0.008, 4 * 0.008])
        self.Kd2 = np.array([1625 * 0.3, 1625 * 0.3, 465 * 0.3])       

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

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def waypoint_generator(self, lat1, lon1, lat2, lon2, dist):
        x0, y0 = self.lat_to_x(lat1), self.long_to_y(lon1)
        x1, y1 = self.lat_to_x(lat2), self.long_to_y(lon2)
        d = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        print(d)
        self.dt = self.dt + dist if (self.dt + dist) < d else d
        t = self.dt / d
        waypoint = np.array([((1-t)*x0 + t*x1), ((1-t)*y0 + t*y1), 25.16])
        return waypoint, t
    
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])

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
       

    # PID algorithm
    def pickup(self):
        # Initially, GPS location is not updated,i.e., [0,0,0]
        # To avoid undesired motion due to this the following is used.
        # It returns to the main function if any values are 0
        if not np.any(self.currentloc):
            return

        # Calculating error term and rounding off to 7 decimal points
        # Rounding off because double precision is overkill for PID
        self.currentlocxy = np.array([self.lat_to_x(self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])
        if abs(self.del_error[0]) < 1 and abs(self.del_error[1]) < 1 and abs(self.del_error[2]) < 0.1 and self.t != 1:
            self.waypoint, self.t = self.waypoint_generator(self.setpoints[0,0], self.setpoints[0,1], self.pickuploc[0], self.pickuploc[1], 12)

	self.del_error = np.round((self.waypoint - self.currentlocxy), 7)

        # Calculating derivative term and rounding off
        # / symbol allows divison for sample_time scalar with every element in the array
        derivative = np.round(
            ((self.del_error - self.prev_values) / self.sample_time), 7)

        # Calculating integral term and rounding off
        # * symbol allows multiplication for sample_time scalar with every element in the array
        self.integral = np.round(
            ((self.integral + self.del_error) * self.sample_time), 7)

        # Calculating PID output and rounding off
        # * symbol for numpy arrays allows multiplication of the corresponding elements
        # output = [out_latitude, out_longitude, out_altitude]
        output = np.round(
            ((self.Kp2 * self.del_error) + (self.Ki2 * self.integral) + (self.Kd2 * derivative)), 7)

        # Calculating throttle, roll and pitch and checking limits
        # Since 1500 is the mid value,
        # Direction will change according to sign of output
        throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 - output[1])
        roll = self.checkLimits(1500.0 + output[0])

        # Assigning prev_values with error for the next iteration
        self.prev_values = self.del_error

        # Publishing final PID output on /edrone/drone_command for the attitude
        # controller
        self.setpoint_rpy.rcRoll = roll
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

	if self.t == 1:
            if abs(self.del_error[0]) > 0.1:
                return
            elif self.detectconf is not True:
                self.waypoint[2] = 22.86
                print('Hovering for detection') 
            else:
                self.waypoint[2] = 22.16
                self.setpoint_rpy.rcThrottle = 1000
                self.setpoint_pub.publish(self.setpoint_rpy)
                gripper_response = self.gripper_activate(self.gripperState)
                print('GripperState:'+str(self.gripperState)+str(gripper_response))
                if gripper_response.result is True:
                        self.delivery_flag = 1
                        self.prev_values = np.array([0,0,0])
                        self.integral = np.array([0,0,0])
                        self.t = 0

    def delivery(self):
        if not np.any(self.currentloc):
                return
        self.currentlocxy = np.array([self.lat_to_x(self.currentloc[0]), self.long_to_y(self.currentloc[1]), self.currentloc[2]])
        #if (self.ranges > 3).all():
        if abs(self.del_error[0]) < 1 and abs(self.del_error[1]) < 1 and abs(self.del_error[2]) < 0.1 and self.t != 1:
                
            self.waypoint, self.t = self.waypoint_generator(self.setpoints[3,0], self.setpoints[3,1], self.detectedcord[0], self.detectedcord[1], 25)
        self.del_error = np.round((self.waypoint - self.currentlocxy), 7)

        #elif self.ranges[3] < 4:
        #    self.del_error = np.round(np.array([(2-self.ranges[3]), 1.5, (25.16-self.currentlocxy[2])]), 7)

        #elif self.ranges[4] < 4 :
        #    self.del_error = np.round(np.array([-1.5, (2-self.ranges[4]), (25.16-self.currentlocxy[2])]), 7)
        

        derivative = np.round(
            ((self.del_error - self.prev_values) / self.sample_time), 7)
        self.integral = np.round(
            ((self.integral + self.del_error) * self.sample_time), 7)

        output = np.round(
            ((self.Kp2 * self.del_error) + (self.Ki2 * self.integral) + (self.Kd2 * derivative)), 7)


        throttle = self.checkLimits(1500.0 + output[2])
        pitch = self.checkLimits(1500.0 - output[1])
        roll = self.checkLimits(1500.0 + output[0])

        self.prev_values = self.del_error

        self.setpoint_rpy.rcRoll = roll
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

	if self.t == 1:
            if abs(self.del_error[0]) > 0.1:
                return
            elif self.waypoint[2]!=self.detectedcord[2]:
                self.waypoint[2] = self.detectedcord[2]
	    elif abs(self.del_error[2]) < 0.1:
		gripper_response = self.gripper_activate(False)
		if gripper_response is not False:
			return
                

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
