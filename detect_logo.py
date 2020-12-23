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
	# Subscribing to bottom range finder
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.globalheight)
	# Subscring to marker_related
	rospy.Subscriber('/marker_related', String, self.marker_info)

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
	self.focal_length = (200)/math.tan(1.3962634/2)
	# Height from bottom range sensor
        self.z_m=0
        self.logo = ()
	# Confirmation message for detection of the logo
        self.confirmation_msg = "False,0.0,0.0"
	# Publisers for confirmation of detection,
	# x_err, y_err and current marker id
        self.detect_confirm_pub = rospy.Publisher(
            '/detect_confirm', String, queue_size=1)
	self.x_err_pub = rospy.Publisher(
			'/edrone/err_x_m', Float32, queue_size=1)
	self.y_err_pub = rospy.Publisher(
			'/edrone/err_y_m', Float32, queue_size=1)
	self.curr_marker_pub = rospy.Publisher(
			'/edrone/curr_marker_id', Int32, queue_size=1)
   

    # Callback for marker info
    def marker_info(self, data):
        strdata = data.data.split(',')
        self.curr_marker_id = int(strdata[1])
        if strdata[0] == 'True' :
            self.x_error = float(strdata[2])
            self.y_error = float(strdata[3])
        else:
            self.x_error = float("NaN")
            self.y_error = float("NaN")
			
    # Callback for borrom range sensor
    def globalheight(self, msg):
        self.z_m=msg.ranges[0]
	     
   # Callback for image data from the camera
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return
	
    # Function for detection and getting centeral pixel from the logo 
    def detectlogo(self, event):
        global flag
        self.logo_cascade = cv2.CascadeClassifier('/home/rohan/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
        sleep(0.05)
        # Making sure 400x400 image is received
        if self.gray.size == 160000:
            self.logo = self.logo_cascade.detectMultiScale(self.gray, scaleFactor = 1.1) 
            if len(self.logo) is not 0:
                for (x, y, w, h) in self.logo:
		      # Printing rectangle over the detect logo for
		      # Debugging and visualisation
                      self.img=cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
		      # Calculating number of pixels from center to the centre of the image (with sign)
                      self.x_center = x + w/2 - 200
                      self.y_center = 200 - (y + h/2)
         
        cv2.imshow('img',self.img)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            cv2.destroyAllWindows()
    
   # Function to calculate and publish the distance of drone from marker
    def marker_distance(self, event):
        if len(self.logo) is not 0:
            x = self.x_center*self.z_m/self.focal_length
            y = self.y_center*self.z_m/self.focal_length
	    # Publishing the confirmation message for the detection
            self.confirmation_msg = "True,"+str(x)+","+str(y)
            self.detect_confirm_pub.publish(self.confirmation_msg)

        else:
            self.confirmation_msg = "False,"+str(0.0)+","+str(0.0)
            self.detect_confirm_pub.publish(self.confirmation_msg)
		
    # Function to publish x_err, y_err and curr_marker_id
    def publish_slow(self, event):
        self.x_err_pub.publish(self.x_error)
        self.y_err_pub.publish(self.y_error)
        self.curr_marker_pub.publish(self.curr_marker_id)

if __name__ == '__main__':
    img = image_proc()

    # Using ros timer to publish at different frequencies in same node
    rospy.Timer(rospy.Duration(1.0 / 30.0), img.marker_distance)   #Frequency = 30 Hz
    rospy.Timer(rospy.Duration(1.0 / 30.0), img.detectlogo)     #Frequency = 30 Hz      
    rospy.Timer(rospy.Duration(1.0 / 1.0), img.publish_slow)    #Frequency = 1 Hz
    rospy.spin()
