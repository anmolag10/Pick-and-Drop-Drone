#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32
import rospy
import numpy as np
import math
import time


class Position():

    def __init__(self):
        # initializing ros node with name position_controller
        rospy.init_node('node_position_controller')

        # Numpy array for GPS coordinate setpoints
        self.setpoints = np.array(
            [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 0.31]])

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
        # Coefficient ratios for Pid[Longitude] [Kp, Ki, Kd] : [6000, 0.8, 12000] 
        # Coefficient ratios for Pid[Altitude] [Kp, Ki, Kd] : [0.6, 0.008, 0.3]
        
        # Value of [Kp, Ki, Kd][Latitude] : [720, 55, 1750]
        # Value of [Kp, Ki, Kd][Longitude] : [720, 55, 1650]
        # Value of [Kp, Ki, Kd][Altitude] : [225, 4, 465]
        self.Kp = np.array([225 * 0.6, 225 * 0.6, 225 * 0.6,0.0])
        self.Ki = np.array([0 * 0.008, 0 * 0.008, 4 * 0.008,0.0])
        self.Kd = np.array([2665 * 0.3, 2665 * 0.3, 465 * 0.3,0.0])


    

        # For storing previous error for derivative term
        self.prev_values = np.array([0.0, 0.0, 0.0,0.0])
        # For storing sum of error for integral term
        self.integral = np.array([0.0, 0.0, 0.0,0.0])
        # Maximum and Minimum values for roll, pitch and throttle commands
        self.max_value = 2000
        self.min_value = 1000

        # PID sampling rate and time
        self.sample_rate = 10  # in Hz
        self.sample_time = 0.1  # in seconds

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
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.lidar_callback)
        self.confirmationval=False
        # ------------------------------------------------------------------------------------------------------------

    # Callback for GPS location
    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])
    
    def lidar_callback(self, msg):
        self.ranges = msg.ranges

    # Function for checking limits of PID output
    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)
    # if you use this for control, you may have to change the relevant pitch   direction because of the sign
    
    def align_to_goal(self):
        theta = math.degrees(math.atan2((self.long_to_y(self.setpoints[0,1])-self.long_to_y(self.currentloc[1])), (self.lat_to_x(self.setpoints[0,0])-self.lat_to_x(self.currentloc[0]))))
        X=self.lat_to_x(self.setpoints[0,0]) - self.lat_to_x(self.currentloc[0])
        Y=self.long_to_y(self.setpoints[0,1]) - self.long_to_y(self.currentloc[1])
        goal_distance=X * math.sin(theta) + Y * math.cos(theta)
        theta = theta if theta >= 0 else theta + 360
        theta = (theta + 270) % 360
        theta = theta if theta < 180 else (theta - 360)
        theta = (theta + 540) / 0.36
        print(theta)
        return theta, goal_distance

    # PID algorithm
    def pid(self):

        if not np.any(self.currentloc):
            return
        #aligning to the goal
        error = np.round((self.setpoints[self.loc] - self.currentloc), 7)
        derivative = np.round(
            ((error - self.prev_values) / self.sample_time), 7)
        self.integral = np.round(
            (self.integral + error * self.sample_time), 7)
        output = np.round(
            ((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)
        throttle = self.checkLimits(1500.0 + output[2])
        self.prev_values = error
        #values being published to align to the goal
        angle, _ =self.align_to_goal()
        self.setpoint_rpy.rcRoll = 1500
        self.setpoint_rpy.rcPitch = 1500
        self.setpoint_rpy.rcYaw = angle
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)

    def pitchingpid(self):
        if not np.any(self.currentloc):
            return
        #calculating throttle
        error = np.round((self.setpoints[self.loc] - self.currentloc), 7)
        derivative = np.round(
            ((error - self.prev_values) / self.sample_time), 7)
        self.integral = np.round(
            (self.integral + error * self.sample_time), 7)
        output = np.round(
            ((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)), 7)
        throttle = self.checkLimits(1500.0 + output[2])
        self.prev_values = error

        #calculating  pitch val
        _, pitcherror=np.roun((self.align_to_goal()),)
  
        derivative[3] = np.round(
            ((pitcherror - self.prev_values[3]) / self.sample_time), 7)
        self.integral[3] = np.round(
            (self.integral[3] + pitcherror * self.sample_time), 7)
        output = np.round(
            ((self.Kp[3] * pitcherror) + (self.Ki[3] * self.integral[3]) + (self.Kd[3] * derivative[3])), 7)
        pitch = self.checkLimits(1500.0 + output)
        self.prev_values = error

        self.setpoint_rpy.rcRoll = 1500
        self.setpoint_rpy.rcPitch = pitch
        self.setpoint_rpy.rcYaw = 1500
        self.setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(self.setpoint_rpy)
  
    def obsavd(self):
        if self.confimationval==True:
          if self.ranges[0]<1.5:  
             self.pid()
             self.pitchingpid()
          else:
              if self.ranges[1]>1.5:
                    self.setpoint_rpy.rcRoll = 1505
                    self.setpoint_rpy.rcPitch = 1500
                    self.setpoint_rpy.rcYaw = 15000
                    self.setpoint_rpy.rcThrottle = 1500
                    self.setpoint_pub.publish(self.setpoint_rpy)
              else:
                    self.setpoint_rpy.rcRoll = 1495
                    self.setpoint_rpy.rcPitch = 1500
                    self.setpoint_rpy.rcYaw = 15000
                    self.setpoint_rpy.rcThrottle = 1500
                    self.setpoint_pub.publish(self.setpoint_rpy)





   

# ------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':

    e_drone_position = Position()
    # Defining rospy rate such that PID algorithm loops at the
    # desired sampling rate
    r = rospy.Rate(e_drone_position.sample_rate)
    while not rospy.is_shutdown():
        # Calling PID function
        e_drone_position.obsavd()
        r.sleep()
