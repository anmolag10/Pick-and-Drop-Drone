#!/usr/bin/env python

import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from std_msgs.msg import String
import time
import tf
import math
import numpy as np


class obs_avoid():
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.obs_str = ""
        self.edrone_top_range = np.array([0, 0, 0, 0])
        self.edrone_bottom_range = 0
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, callback=self.range_top)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, callback=self.range_bottom)
        self.val_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.obs_pub = rospy.Publisher('/isObstacle', String, queue_size=1)

    def range_top(self, msg):
        self.edrone_top_range =  np.array(msg.ranges)  

    def range_bottom(self, msg):
        self.edrone_bottom_range =  msg.ranges  

    def avoid(self):
        setpoint_val = edrone_cmd()
        self.obs_str = "False"

        if self.edrone_top_range[0] < 3 or self.edrone_top_range[1] < 3 or self.edrone_top_range[2] < 3 or self.edrone_top_range[3] < 3:
            self.obs_str = "True"

        self.obs_pub.publish(self.obs_str)

        if self.obs_str == "True":
            setpoint_val.rcPitch = 1500
            setpoint_val.rcRoll = 1500
            setpoint_val.rcYaw = 1500
            setpoint_val.rcThrottle = 1600
            self.val_pub.publish(setpoint_val)


if __name__ == '__main__':
    obs_avoid = obs_avoid()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        obs_avoid.avoid()
        r.sleep()
