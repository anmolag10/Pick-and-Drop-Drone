#!/usr/bin/env python

'''
# Team ID:          1212
# Theme:            Vitarana Drone
# Author List:      Aditi Rao, Anmol Agarwal, Keshav Kapur
# Filename:         position_controller.py
# Functions:        init, image_callback, detectlogo, marker_distance
# Global variables: None
'''

from time import sleep
from vitarana_drone.msg import *
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Float32, Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import math
import os


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('node_qr_detect')  # Initialise rosnode
        # Subscribing to the camera topic
        rospy.Subscriber(
            "/edrone/camera/image_raw",
            Image,
            self.image_callback)

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.gray = np.empty([])
        # x,y coordinate of centeral pixel
        self.x_center = 0
        self.y_center = 0
        # x,y value of the calculated error using
        # Values from centeral pixel
        self.x_error = 0
        self.y_error = 0
        # Marker id for the detected marker
        self.curr_marker_id = 0
        # Calculating focal length of the camera
        self.focal_length = (200) / math.tan(1.3962634 / 2)
        # Height from bottom range sensor
        self.z_m = 0
        self.logo = ()
        self.marker_data = MarkerData()
        # Confirmation message for detection of the logo
        self.confirmation_msg = "False,0.0,0.0"
        # Publisers for confirmation of detection,
        # x_err, y_err and current marker id
        self.detect_confirm_pub = rospy.Publisher(
            '/detect_confirm', String, queue_size=1)

    # Callback for image data from the camera

    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return

    def detectlogo(self, event):
        '''
        Purpose:
        ---
        For Detection and getting the central pixel of the detection

        Input Arguments:
        ---
        event :  [ ]
            Required for rospy.Timer

        Returns:
        ---
        Does not return value but changes a member variable

        Example call:
        ---
        self.detectlogo()
        '''
        self.logo_cascade = cv2.CascadeClassifier(os.path.expanduser(
            '~/catkin_ws/src/intro_cascade_classifiers_training_and_usage/data/cascade.xml'))
        sleep(0.05)
        # Making sure 400x400 image is received
        if self.gray.size == 160000:
            self.logo = self.logo_cascade.detectMultiScale(
                self.gray, scaleFactor=1.1)
            if len(self.logo) is not 0:
                for (x, y, w, h) in self.logo:
                  # Printing rectangle over the detect logo for
                  # Debugging and visualisation
                    self.img = cv2.rectangle(
                        self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    # Calculating number of pixels from center to the centre of the image
                    # (with sign)
                    self.x_center = x + w / 2 - 200
                    self.y_center = 200 - (y + h / 2)

        cv2.imshow('img', self.img)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            cv2.destroyAllWindows()

    def marker_distance(self, event):
        '''
        Purpose:
        ---
        Function to calculate and publish the distance of drone from marker

        Input Arguments:
        ---
        event :  [ ]
            Required for rospy.Timer

        Returns:
        ---
        None

        Example call:
        ---
        self.marker_distance()
        '''
        if len(self.logo) is not 0:
            x = self.x_center / self.focal_length
            y = self.y_center / self.focal_length
        # Publishing the confirmation message for the detection
            self.confirmation_msg = "True," + str(x) + "," + str(y)
            self.detect_confirm_pub.publish(self.confirmation_msg)

        else:
            self.confirmation_msg = "False," + str(0.0) + "," + str(0.0)
            self.detect_confirm_pub.publish(self.confirmation_msg)

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the marker_distance() and detectlogo() functions 
if __name__ == '__main__':
    img = image_proc()
    # Using ros timer to publish at different frequencies in same node
    rospy.Timer(rospy.Duration(1.0 / 30.0),
                img.marker_distance)  # Frequency = 30 Hz
    rospy.Timer(rospy.Duration(1.0 / 30.0),
                img.detectlogo)  # Frequency = 30 Hz
    rospy.spin()
