#!/usr/bin/env python

from __future__ import print_function

import sys

import cv2
import numpy as np

from ugv_bot.srv import SendImu, SendImuResponse,SendImuRequest
import rospy

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


def imu_client():
    rospy.wait_for_service('get_imu_service')
    try:
        get_imu_service = rospy.ServiceProxy('get_imu_service', SendImu ,persistent=True)
        responce = get_imu_service(1)
        return responce

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def imu_publisher(srv_responce):
    imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
    mag_pub = rospy.Publisher('/imu/data_raw_msg', MagneticField, queue_size=1)

    print(type(srv_responce))

    final_imu_data=Imu()
    final_mag_data=MagneticField()

    final_imu_data.angular_velocity.x = srv_responce.angular_velocity.x
    final_imu_data.angular_velocity.y = srv_responce.angular_velocity.y
    final_imu_data.angular_velocity.z = srv_responce.angular_velocity.z

    final_imu_data.linear_acceleration.x = srv_responce.linear_acceleration.x
    final_imu_data.linear_acceleration.y = srv_responce.linear_acceleration.y
    final_imu_data.linear_acceleration.z = srv_responce.linear_acceleration.z


    final_imu_data.header.seq=srv_responce.header.seq
    final_imu_data.header.frame_id=srv_responce.header.frame_id
    final_imu_data.header.stamp=srv_responce.header.stamp

    #print(type(final_imu_data))

    imu_pub.publish(final_imu_data)


    final_mag_data.magnetic_field.x = srv_responce.magnetometer.x
    final_mag_data.magnetic_field.y = srv_responce.magnetometer.y
    final_mag_data.magnetic_field.z = srv_responce.magnetometer.z


    mag_pub.publish(final_mag_data)




    if VERBOSE :
        rospy.loginfo("From Hokuyo imu Redirector")
        print("length of Range list = ",len(srv_responce.ranges))
    else :
        print("           imu Redirected......Bitches!")


if __name__ == "__main__":

    VERBOSE = False

    rospy.init_node('imu_Redirecting_from_Client', anonymous=False)


    while True:
        try:
            srv_responce = imu_client()
            if srv_responce is not None:
                imu_publisher(srv_responce)

        except KeyboardInterrupt:
            print("Shit Happened in imu client")
            sys.exit(1)

