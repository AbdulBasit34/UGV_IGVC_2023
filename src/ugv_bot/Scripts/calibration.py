#!/usr/bin/env python3

import serial
import time
import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


#ser = serial.Serial("/dev/tty/USB0", 115200)


def dist(u, v):
    global x
    global y
    dist = math.sqrt((u - x)**2 + (v - y)**2)
    print(f"distance calculated at {x} {y}: {dist}")
    return dist


def calibration():

	global x
	global y

	min_w = 0.01
	max_w = 1.0

	threshold = 0.1

	w = min_w
	
	start_x = x
	start_y = y
	

	while w <= max_w:
		#ser.write(bytes('@0sv'+str(w)+'\r', 'utf-8'))
		#ser.write(bytes('@1sv'+str(w)+'\r', 'utf-8'))
		print(f"testing w :{w}")
		#start_x = x
		#start_y = y

		if dist(start_x, start_y) > threshold:
			updated_w = w
			print(f"w found: {w}")
			return w

		w = w + 0.01

		time.sleep(1)



def loop(data):

	global x
	global y

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	



if __name__ == '__main__':

	rospy.init_node('calibration_node', anonymous=False)
	
	x=0
	y=0

	try:
		rospy.Subscriber("/zed2i/zed_node/odom", Odometry, loop)
		time.sleep(1)
		calibration()

	except KeyboardInterrupt:
	    print("Shit Happened in twist client")
	    sys.exit(1)


	

	



