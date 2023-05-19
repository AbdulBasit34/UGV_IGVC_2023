#!/usr/bin/env python

from __future__ import print_function

import sys

import cv2
import numpy as np

from ugv_bot.srv import SendImage, SendImageResponse,SendImageRequest
import rospy

def image_client():
    rospy.wait_for_service('get_image_service')
    try:
        print("Input was :",a)
        get_image_service = rospy.ServiceProxy('get_image_service', SendImage,persistent=True)
        responce = get_image_service(a)
        return responce
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    
    VERBOSE = False

    a = input("Enter Bool :")

    if a==True:
        if VERBOSE:
            print("Entered Bool : ",a)
            print("---------------------------")
            print("Recieved Something ",image_client())

        ros_data = image_client()

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        cv2.imshow('Image',image_np)
        cv2.waitKey()
        cv2.destroyAllWindows()

    else:
        print("Shit Happened in client")
        sys.exit(1)

