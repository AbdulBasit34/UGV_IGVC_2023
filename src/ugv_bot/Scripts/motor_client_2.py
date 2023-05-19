#!/usr/bin/env python3

from __future__ import print_function

import sys

import cv2
import numpy as np

from ugv_bot.srv import SendTwist, SendTwistResponse,SendTwistRequest
import rospy

from geometry_msgs.msg import Twist
import serial
import time

ser=serial.Serial("/dev/ttyUSB0",115200)


def twist_client():
    rospy.wait_for_service('get_twist_service')
    try:
        get_twist_service = rospy.ServiceProxy('get_twist_service', SendTwist ,persistent=True)
        responce = get_twist_service(1)
        return responce

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def twist_serial_publisher(srv_responce):
    
    l=0.51      #wheel separation
    r=0.15      #wheel radius
    

    print(type(srv_responce))

    vl=srv_responce.linear_vel
    w=srv_responce.angular_vel



    #w=1.5*w

    print("w = ", w)
    print("vl = ",vl)

    v1= (2*vl+l*w)/2
    v2= (2*vl-l*w)/2

    # v1 = 0.2
    # v2 = 0.2

    print("--------------")
    print("v1 = ", v1)
    print("v2 = ",v2 )


    if w <= 0.2 and w > 0.05:
        w = 0.2

    if w >= -0.2 and w < -0.05:
        w = -0.2


    w1 = v1/r
    w2 = v2/r

    print("-------------")
    print("w1 = ", w1)
    print("w2 = ", w2)

    # if w1 < -1:
    #     w1 = -1
    # if w2 < -1:
    #     w2 = -1

    #w1=1.2*w1


    ser.write(bytes('@0sv'+str(w1)+'\r', 'utf-8'))
    ser.write(bytes('@1sv'+str(w2)+'\r', 'utf-8'))


    if VERBOSE :
        rospy.loginfo("From Hokuyo Laser Redirector")
        print("length of Range list = ",len(srv_responce.ranges))
    else :
        print("           Laser Redirected......Bitches!")


if __name__ == "__main__":

    VERBOSE = False

    rospy.init_node('twist_Redirecting_from_Client', anonymous=False)


    while True:
        try:
            srv_responce = twist_client()
            if srv_responce is not None:
                twist_serial_publisher(srv_responce)

        except KeyboardInterrupt:
            print("Shit Happened in twist client")
            sys.exit(1)

