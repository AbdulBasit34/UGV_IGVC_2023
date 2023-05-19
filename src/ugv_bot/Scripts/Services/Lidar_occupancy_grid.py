#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge 
import cv2
import numpy as np
import matplotlib.pyplot as plt

resolution=100

def laser_client():
    rospy.wait_for_service('get_laser_service')
    try:
        get_laser_service = rospy.ServiceProxy('get_laser_service', SendLaser ,persistent=True)
        responce = get_laser_service(1)
        return responce

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pol2cart(theta, rho):
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y


def laser_publisher(srv_responce):
    laser_pub = rospy.Publisher('ugvbot/scan', LaserScan, queue_size=1)

    print(type(srv_responce))

    final_laser_data=LaserScan()

    final_laser_data.header.stamp = rospy.Time.now()
    final_laser_data.header.frame_id = srv_responce.header.frame_id

    final_laser_data.angle_min = srv_responce.angle_min
    final_laser_data.angle_max = srv_responce.angle_max

    final_laser_data.angle_increment = srv_responce.angle_increment
    final_laser_data.time_increment = srv_responce.time_increment

    final_laser_data.scan_time = srv_responce.scan_time

    final_laser_data.range_min = srv_responce.range_min
    final_laser_data.range_max = srv_responce.range_max

    final_laser_data.ranges = srv_responce.ranges

    final_laser_data.intensities = srv_responce.intensities


    increment=np.array(srv_responce.angle_increment)
    increment = increment.cumsum(increment)
    theta=increment+srv_responce.angle_min
    r=srv_responce.ranges

    cart=[]
    for value  in enumerate(zip(r,theta)):
        cart.append(pol2cart(value))
    
    cart=np.array(cart)
    cart=cart.astype(int)

    occupancy_grid=np.zeros(resolution,resolution)

    for element  in cart:
        try : 
            occupancy_grid[element]=1
        except :
            pass

    plt.imshow(occupancy_grid)
    plt.show()










    #print(type(final_laser_data))

    laser_pub.publish(final_laser_data)





if __name__ == "__main__":

    VERBOSE = False

    rospy.init_node('Laser_Redirecting_from_Client', anonymous=False)


    while True:
        try:
            srv_responce = laser_client()
            if srv_responce is not None:
                laser_publisher(srv_responce)

        except KeyboardInterrupt:
            print("Shit Happened in laser client")
            sys.exit(1)

