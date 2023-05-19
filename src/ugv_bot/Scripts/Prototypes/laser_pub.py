#!/usr/bin/env python
import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import LaserScan

def generate(scan_rate, angle_min, angle_max, ranges, intensities):

    scan=LaserScan()
    start_time = rospy.Time.now()
    angle_increment=(angle_max-angle_min)/400
    time_increment=1/scan_rate

    scan.header.stamp = start_time
    scan.header.frame_id = "laser"
    scan.angle_min=angle_min
    scan.angle_max=angle_max
    scan.angle_increment=angle_increment
    scan.time_increment=time_increment
    scan.range_min=0
    scan.range_max=10
    scan.ranges=ranges
    scan.intensities=intensities

    return scan


def publisher():

    rospy.init_node('LaserScan', anonymous='True')
    pub = rospy.Publisher('ugvbot/fake_scan', LaserScan, queue_size=10)

    img=np.zeros((400,400), np.uint8)

    for i in range(1,400):
        img[i][100]=255


    while not rospy.is_shutdown():
        cv2.imshow("IMG", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        angle_min=-(math.pi)/2
        angle_max=math.pi/2
        ranges=[]
        intensities=[]
	for i in range(1,400):
	    ranges.append(1.0)

        scan_rate = 33
        rate=rospy.Rate(scan_rate)

        scan=generate(scan_rate, angle_min, angle_max, ranges, intensities)

        pub.publish(scan)
	rospy.loginfo("Publishing")
	rate.sleep()



if __name__ == '__main__' :

	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
