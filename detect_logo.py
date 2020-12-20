#!/usr/bin/env python

from time import sleep
from sensor_msgs.msg import Image, NavSatFix, LaserScan
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
import time
from vitarana_drone.srv import Gripper
import rospy
import matplotlib.pyplot as plt
import math

flag = 0
class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('node_qr_detect')  # Initialise rosnode
        # Subscribing to the camera topic
        rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber('/quadxy',String, self.quadxy)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.globalheight)

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.gray = np.empty([])
        self.x_center = 0
        self.y_center = 0
        self.x_err = 0
        self.y_err = 0
        self.x_quad=0
        self.y_quad=0
        self.z_m=0
        self.lat= 0
        self.logo = ()
        self.confirmation_msg = "False,0.0,0.0"
        self.detect_confirm_pub = rospy.Publisher(
            '/detect_confirm', String, queue_size=1)
        self.x_pub=rospy.Publisher('/edrone/err_x_m',Float32,queue_size=1)
        self.y_pub=rospy.Publisher('/edrone/err_y_m',Float32,queue_size=1)
    # Function to decode the QR code

    def quadxy(self,msg):
       msgstr=msg.data.split(',')
       self.x_quad=float(msgstr[0])
       self.y_quad=float(msgstr[1])

    def globalheight(self, msg):
        self.z_m=msg.ranges[0]
    
    def imdecode(self):
        global flag
        self.logo_cascade = cv2.CascadeClassifier(r'/home/blebot/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
        time.sleep(0.05)
        
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
    

    def error_finder(self):
        if len(self.logo) is not 0:
            focal_length = (200)/math.tan(1.3962634/2)
            self.x_err = self.x_center*self.z_m/focal_length
            self.y_err = self.y_center*self.z_m/focal_length
            self.confirmation_msg = "True,"+str(self.x_err)+","+str(self.y_err)
            self.detect_confirm_pub.publish(self.confirmation_msg)
            self.x_pub.publish(self.x_err)
            self.y_pub.publish(self.y_err)

        else:
            self.confirmation_msg = "False,"+str(self.x_err)+","+str(self.y_err)
            self.detect_confirm_pub.publish(self.confirmation_msg)
            


    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
            return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        image_proc_obj.error_finder()
        image_proc_obj.imdecode()
        r.sleep()
