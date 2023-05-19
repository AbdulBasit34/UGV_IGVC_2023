#!/usr/bin/env python3
from __future__ import print_function

import os

import cv2
import numpy as np
import genpy

import ugv_bot.srv
from ugv_bot.srv import SendTwist, SendTwistResponse
import rospy
import time
import threading

from geometry_msgs.msg import Twist


def handle_twist(request):
	global twist_data
	global twist_time 
	#print(os.path.dirname(os.getcwd())+"/catkin_ws/src/ugv_bot/Scripts/test_img.png")

	if VERBOSE:
		print("Request Variable",request)
		print("Request Variable type",type(request))
		print("-----------------------------------------")

	if request:

		srv_msg=SendTwistResponse()
	   
		#srv_msg.header.stamp = rospy.Time.now()
		srv_msg.linear_vel = twist_data.linear.x
		srv_msg.angular_vel = twist_data.angular.z    

		#msg=Twist()
		#msg.linear.x=0.0
		#msg.angular.x=0.0
		#srv_msg.linear_vel =0.5
		#srv_msg.angular_vel=0
		got_time = twist_time
		# prev_time = rospy.Time.now()

		print(twist_time - (rospy.Time.now()))

		if rospy.Time.now() - twist_time < rospy.Duration.from_sec(3):
			srv_msg.linear_vel = twist_data.linear.x
			srv_msg.angular_vel = twist_data.angular.z
			print("Done Sending !!!!")
			print("===============================================")

		else:
			print("ERROR : noting is being published on cmd_vel")
			srv_msg.linear_vel = 0
			srv_msg.angular_vel = 0

		# print(twist_data)
		print(f"service:{rospy.Time.now()}")

		time.sleep(0.01)
		#print("Hey Homie........")

		prev_time = twist_time

		return srv_msg

def twist_redirector(ros_data):
	#print(ros_data)

	global twist_data
	global twist_time

	twist_time = rospy.Time.now()
	print(f"subscriber:{rospy.Time.now()}")

	if ros_data is None:
		rosdata.linear.x=0.0
		rosdata.angular.x=0.0
	twist_data = ros_data




if __name__ == "__main__":
	VERBOSE = False

	global twist_data

	twist_data = Twist()
	twist_data.linear.x=0
	twist_data.angular.z =0

   
	rospy.init_node('twist_server')
	twist_time = rospy.Time.now()

	rospy.Subscriber('/ugvbot/velocity_controller/cmd_vel',Twist ,twist_redirector,queue_size = 1)

	rospy.Service('get_twist_service',SendTwist,handle_twist)
	
	print("Ready to send twist  .")
	rospy.spin()
