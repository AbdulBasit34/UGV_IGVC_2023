#!/usr/bin/env python
from __future__ import print_function

import os

import cv2
import numpy as np

import ugv_bot.srv
from ugv_bot.srv import SendImage, SendImageResponse
import rospy


def rgba2rgb( rgba, background=(255,255,255) ):
    row, col, ch = rgba.shape

    if ch == 3:
        print("Yess")
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros( (row, col, 3), dtype='float32' )
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]

    a = np.asarray( a, dtype='float32' ) / 255.0

    R, G, B = background

    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray( rgb, dtype='uint8' )

def handle_image(request):

    print(os.path.dirname(os.getcwd())+"/catkin_ws/src/ugv_bot/Scripts/test_img.png")

    if VERBOSE:
        print("Request Variable",request)
        print("Request Variable type",type(request))
        print("-----------------------------------------")

    if request:

        img = cv2.imread(os.getcwd()+"/catkin_ws/src/ugv_bot/Scripts/test_img.png",cv2.IMREAD_UNCHANGED) # Read the test img
        img = rgba2rgb(img)


        srv_msg = SendImageResponse()
        srv_msg.header.stamp = rospy.Time.now()
        srv_msg.format = "jpeg"
        srv_msg.data = np.array(cv2.imencode('.jpg',img)[1]).tostring()

        print("Hey Homie........")
        print("Done Sending !!!!")
        print("===============================================")

        return srv_msg

def image_server():
    rospy.init_node('image_server')
    s = rospy.Service('get_image_service', SendImage, handle_image)
    print("Ready to send Image.")
    rospy.spin()

if __name__ == "__main__":
    VERBOSE = False
    image_server()    