#!/usr/bin/env python
from __future__ import print_function

import os

import cv2
import numpy as np

import ugv_bot.srv
from ugv_bot.srv import SendLaser, SendLaserResponse
import rospy

from sensor_msgs.msg import LaserScan

def handle_laser(request):
    global laser_data
    #print(os.path.dirname(os.getcwd())+"/catkin_ws/src/ugv_bot/Scripts/test_img.png")

    if VERBOSE:
        print("Request Variable",request)
        print("Request Variable type",type(request))
        print("-----------------------------------------")

    if request:

        srv_msg=SendLaserResponse()
       
        #srv_msg.header.stamp = rospy.Time.now()
        srv_msg.header = laser_data.header

        srv_msg.angle_min = laser_data.angle_min
        srv_msg.angle_max = laser_data.angle_max

        srv_msg.angle_increment = laser_data.angle_increment
        srv_msg.time_increment = laser_data.time_increment

        srv_msg.scan_time = laser_data.scan_time

        srv_msg.range_min = laser_data.range_min

        srv_msg.range_max = laser_data.range_max

        srv_msg.ranges = laser_data.ranges

        srv_msg.intensities = laser_data.intensities
      

        print("Hey Homie........")
        print("Done Sending !!!!")
        print("===============================================")

        return srv_msg

def laser_redirector(ros_data):

    global laser_data
    laser_data = ros_data

if __name__ == "__main__":
    VERBOSE = False
   
    rospy.init_node('laser_server')

    rospy.Subscriber("/scan",LaserScan,laser_redirector,queue_size = 1)

    s = rospy.Service('get_laser_service', SendLaser, handle_laser)
    
    print("Ready to send Laser.")
    rospy.spin()