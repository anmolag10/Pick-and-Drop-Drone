#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


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

flag = 0
flaggrip = 0


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('barcode_test')  # Initialise rosnode
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)
        # This will contain your image frame from camera
        self.detect_confirm_pub=rospy.Publisher('/detect_confirm',String,queue_size=1)
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.decodedimg = ("")
        self.setpointCordinates = [0, 0, 0]
        self.dectection_Coordinates=String()
        self.confirmation_msg="False,0.0,0.0,0.0"
    def imdecode(self):
            global flaggrip
            time.sleep(0.05)
            try:
                self.decodedimg = decode(self.img)
                gps = []
                for i in self.decodedimg:
                    gps = i.data.decode("utf-8").split(',')
                self.setpointCordinates = list(map(float, gps))
                if gps:
                
                    print("Coordinates detected")
                    self.confirmation_msg="True,"+str(self.setpointCordinates[0])+","+str(self.setpointCordinates[1])+","+str(self.setpointCordinates[2])
                    print(self.setpointCordinates)
                    self.detect_confirm_pub.publish(self.confirmation_msg)
                    return
            except:
                pass
            self.detect_confirm_pub.publish(self.confirmation_msg)
       
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    image_proc_obj = image_proc()
    r=rospy.Rate(30)
    while not rospy.is_shutdown():
        image_proc_obj.imdecode()
        r.sleep()
