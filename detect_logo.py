#!/usr/bin/env python

from time import sleep
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
import time
from vitarana_drone.srv import Gripper
import rospy


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('node_qr_detect')  # Initialise rosnode
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
	self.gray = np.empty([])cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	self.logo_cascade = cv2.CascadeClassifier('data/cascade.xml')

        self.confirmation_msg = "False,0.0,0.0,0.0,0.0"
        self.detect_confirm_pub = rospy.Publisher(
            '/detect_confirm', String, queue_size=1)

    # Function to decode the QR code
    def imdecode(self):

        time.sleep(0.05)
        try:
            logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
	    if len(logo) is not 0:
		self.confirmation_msg = "True,"+str(logo[0,0])+","+str(logo[0,1])+","+str(logo[0,2])+","+str(logo[0,3])
		
        except BaseException:
            pass
        self.detect_confirm_pub.publish(self.confirmation_msg)

    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        image_proc_obj.imdecode()
        r.sleep()
