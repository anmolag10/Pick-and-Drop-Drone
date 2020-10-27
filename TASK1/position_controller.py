#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf
import math
import numpy as np


class Edrone():
    """docstring for Edrone"""

    def __init__(self):
        # initializing ros node with name drone_control
        rospy.init_node('position_controller')

        self.setpoints = np.array(
            [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0], [19.0000451704, 72.0, 0.31]])

        self.currentloc = np.array([0.0, 0.0, 0.0])
        self.loc = 0
        self.stabilize = 0

        self.Kp = np.array([60 * 6000, 60 * 6000, 1397 * 0.06])
        self.Ki = np.array([8 * 0.8, 8 * 0.8, 39 * 0.008])
        self.Kd = np.array([2009 * 300, 2009 * 300, 3493 * 0.03])
        # -----------------------Add other required variables for pid here ----
        #
        self.prev_values = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        self.max_value = 2000
        self.min_value = 1000

        self.sample_time = 10  # in seconds

        self.setpoint_pub = rospy.Publisher(
            '/drone_command', edrone_cmd, queue_size=1)
        # ------------------------Add other ROS Publishers here----------------
        self.altitude_error_pub = rospy.Publisher(
            '/altitude_error', Float32, queue_size=1)
        self.latitude_error_pub = rospy.Publisher(
            '/latitude_error', Float32, queue_size=1)
        self.longitude_error_pub = rospy.Publisher(
            '/longitude_error', Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here--------------
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # ------------------------------------------------------------------------------------------------------------

    def gps_callback(self, msg):
        self.currentloc = np.array([msg.latitude, msg.longitude, msg.altitude])


    def roll_set_pid(self, roll):
        # This is just for an example. You can change the ratio/fraction value
        # accordingly
        self.Kp[0] = roll.Kp * 6000
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 300
    # ----------------------------------------------------------------------------------------------------------------------

    def checkLimits(self, drone_command):
        if drone_command > self.max_value:
            return self.max_value
        elif drone_command < self.min_value:
            return self.min_value
        else:
            return drone_command

    def pid(self):

        if not np.any(self.currentloc):
            return

        error = np.array([0, 0, 0])
        error = np.round((self.setpoints[self.loc] - self.currentloc), 7)

        derivative = np.array([0, 0, 0])
        derivative = np.round(
            ((error - self.prev_values) * self.sample_time), 7)

        self.latitude_error_pub.publish(error[0])
        self.longitude_error_pub.publish(error[1])
        self.altitude_error_pub.publish(error[2])

        self.integral = np.round(
            (((self.integral + error) * self.Ki) / self.sample_time), 7)

        output = np.round(
            ((self.Kp * error) + self.integral + (self.Kd * derivative)), 7)

        throttle = self.checkLimits((511.5 + output[2]) * 500.0 / 511.5 + 1000)
        pitch = self.checkLimits((0.0 + output[1]) * 50.0 + 1500.0)
        roll = self.checkLimits((0.0 + output[0]) * 50.0 + 1500.0)

        self.prev_values = error
        
        if abs(
            error[1]) < 0.0000047487 and abs(
            error[0]) < 0.000004517 and abs(
                error[2]) < 0.2:
	    if self.loc == 1:
		return
            if self.stabilize < 5:
                self.stabilize = self.stabilize + 1
                return
            self.loc = self.loc + 1
            self.stabilize = 0
        
        #
        #
        #
        #
        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------

        setpoint_rpy = edrone_cmd()
        setpoint_rpy.rcRoll = roll
        setpoint_rpy.rcPitch = pitch
        setpoint_rpy.rcYaw = 1500
        setpoint_rpy.rcThrottle = throttle
        self.setpoint_pub.publish(setpoint_rpy)


if __name__ == '__main__':

    e_drone = Edrone()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if
    # desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
