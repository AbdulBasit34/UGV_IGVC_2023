#!/usr/bin/env python
from __future__ import print_function

import os

import cv2
import numpy as np

import ugv_bot.srv
from ugv_bot.srv import SendImu, SendImuResponse
import rospy

import serial

from sensor_msgs.msg import Imu

uno=serial.Serial(port='/dev/ttyACM0', baudrate='115200', timeout=0.01)


def read_serial():
    
    global ax
    global ay
    global az

    global gx
    global gy
    global gz

    global mx
    global my
    global mz

    data=uno.readline()
    data=data.decode()
    data=data.replace("\n", "")
    sdata=data.split(',')


    if len(sdata) == 9:
        try :
            for i in range(9):
                float(sdata[i])

            ax = float(sdata[0])
            ay = float(sdata[1])
            az = float(sdata[2])

            gx = float(sdata[3])
            gy = float(sdata[4])  
            gz = float(sdata[5])   


            mx = float(sdata[3])
            my = float(sdata[4])  
            mz = float(sdata[5])   
            #gz = float(sdata[5].replace('\r\n', ''))

            #print(ax,ay,az,gx,gy,gz,"||")

            print("----------Varibles Updated----------")
        except:
            print("Not A Float !!!!")

def handle_imu(request):
    # global imu_data
    #print(os.path.dirname(os.getcwd())+"/catkin_ws/src/ugv_bot/Scripts/test_img.png")

    global ax
    global ay
    global az

    global gx
    global gy
    global gz

    global mx
    global my
    global mz

    if VERBOSE:
        print("Request Variable",request)
        print("Request Variable type",type(request))
        print("-----------------------------------------")

    if request:

        read_serial()
        srv_msg=SendImuResponse()
        srv_msg.angular_velocity.x = gx
        srv_msg.angular_velocity.y = gy
        srv_msg.angular_velocity.z = gz

        srv_msg.linear_acceleration.x = ax
        srv_msg.linear_acceleration.y = ay
        srv_msg.linear_acceleration.z = az

        srv_msg.magnetometer.x = mx
        srv_msg.magnetometer.y = my
        srv_msg.magnetometer.z = mz

        srv_msg.header.seq=0
        srv_msg.header.frame_id='odom'
        srv_msg.header.stamp=rospy.Time.now()
      

        print("Hey Homie........")
        print("Done Sending !!!!")
        print("===============================================")

        return srv_msg

# def imu_redirector(ros_data):

#     global imu_data
#     imu_data = ros_data
#     print('===========')

if __name__ == "__main__":
    VERBOSE = False
   
    rospy.init_node('imu_server')

    #rospy.Subscriber("/Imu_accel",Imu ,imu_redirector,queue_size = 1)

    s = rospy.Service('get_imu_service',SendImu , handle_imu)
    
    print("Ready to send imu.")
    rospy.spin()
