#!/usr/bin/env python

from time import sleep
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Float32, Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import math

flag = 0
class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('node_qr_detect')  # Initialise rosnode
        # Subscribing to the camera topic
        rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.globalheight)
	rospy.Subscriber('/marker_related', String, self.marker_info)

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.gray = np.empty([])
        self.x_center = 0
        self.y_center = 0
	self.x_error = 0
	self.y_error = 0
	self.curr_marker_id = 0
	self.focal_length = (200)/math.tan(1.3962634/2)
        self.z_m=0
        self.logo = ()
        self.confirmation_msg = "False,0.0,0.0"
        self.detect_confirm_pub = rospy.Publisher(
            '/detect_confirm', String, queue_size=1)
	self.x_err_pub = rospy.Publisher(
			'/edrone/err_x_m', Float32, queue_size=1)
	self.y_err_pub = rospy.Publisher(
			'/edrone/err_y_m', Float32, queue_size=1)
	self.curr_marker_pub = rospy.Publisher(
			'/edrone/curr_marker_id', Int32, queue_size=1)
    # Function to decode the QR code

    def marker_info(self, data):
	strdata = data.data.split(',')
	self.curr_marker_id = int(strdata[1])
	if strdata[0] == 'True' :
		self.x_error = float(strdata[2])
		self.y_error = float(strdata[3])
	else:
		self.x_error = float("NaN")
		self.y_error = float("NaN")

    def globalheight(self, msg):
        self.z_m=msg.ranges[0]
    
    def imdecode(self, event):
        global flag
        self.logo_cascade = cv2.CascadeClassifier('/home/rohan/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
        sleep(0.05)
        
        if self.gray.size == 160000:
            self.logo = self.logo_cascade.detectMultiScale(self.gray, scaleFactor = 1.1) 
            if len(self.logo) is not 0:
                for (x, y, w, h) in self.logo:
                      self.img=cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                      self.x_center = x + w/2 - 200
                      self.y_center = 200 - (y + h/2)
            
        cv2.imshow('img',self.img)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            cv2.destroyAllWindows()
    

    def error_finder(self, event):
        if len(self.logo) is not 0:
            x = self.x_center*self.z_m/self.focal_length
            y = self.y_center*self.z_m/self.focal_length
            self.confirmation_msg = "True,"+str(x)+","+str(y)
            self.detect_confirm_pub.publish(self.confirmation_msg)

        else:
            self.confirmation_msg = "False,"+str(0.0)+","+str(0.0)
            self.detect_confirm_pub.publish(self.confirmation_msg)

    def publish_slow(self, event):
	self.x_err_pub.publish(self.x_error)
	self.y_err_pub.publish(self.y_error)
	self.curr_marker_pub.publish(self.curr_marker_id)
            
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return

if __name__ == '__main__':
    img = image_proc()
    rospy.Timer(rospy.Duration(1.0 / 30.0), img.error_finder)
    rospy.Timer(rospy.Duration(1.0 / 30.0), img.imdecode)
    rospy.Timer(rospy.Duration(1.0 / 1.0), img.publish_slow)
    rospy.spin()
