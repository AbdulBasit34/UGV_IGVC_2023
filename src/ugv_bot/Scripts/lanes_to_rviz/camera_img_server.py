#!/usr/bin/env python3

import os

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

import ugv_bot.srv
from ugv_bot.srv import SendImage, SendImageResponse
import rospy
def handle_image(request):
    global img

    if img is None:
        rospy.logerr("Failed to load image")
        return None

    header = Header()
    format = "jpeg"
    image_data = img

    return SendImageResponse(header=header, format=format, data=image_data)



def Img_redirector(msg):
    global img_data
    img_data = msg.data


if __name__ == "__main__":
    VERBOSE = False

    img = None

    count = 0

    rospy.init_node('camera_image_server')
    rospy.Subscriber("/zed2i/zed_node/left/image_rect_color",CompressedImage, Img_redirector,  queue_size = 1)# change topic here to zed topic /zed2i/zed_node/left/image_rect_color
    s = rospy.Service('get_camera_image_service', SendImage, handle_image)

    print("Ready to send Image.")
    
    rospy.spin()
