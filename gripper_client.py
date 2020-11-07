#!/usr/bin/env python

import sys
import rospy
from vitarana_drone.srv import *

def gripper_client(x):
    rospy.wait_for_service('/edrone/activate_gripper')
    try:
        gripper_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        gripper_response = gripper_activate(x)
        return gripper_response.result
    
    except rospy.ServiceException as e:
        print(e)

if __name__=="__main__":
    if len(sys.argv) == 2:
        if sys.argv[1] == "False" or sys.argv[1] == "false":
            x = False
        else:
            x = True
    else:
        print(len(sys.argv))
        sys.exit(1)
    print("Service Resultant: ", gripper_client(x))