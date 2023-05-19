#!/usr/bin/env python3

import rospy
import math
import numpy as np	

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

pub = rospy.Publisher('/odom', Odometry,queue_size=20)

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def quaternion_to_euler(q):
    (w,x,y,z) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def transform_callback(data):       
    data.pose.pose.position.z = 0  
    w = data.pose.pose.orientation.w
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    q = [w, x, y, z]
    (yaw, pitch, roll) = quaternion_to_euler(q)
    pitch = 0.0       
    r = [yaw, pitch, roll]
    q_updated = euler_to_quaternion(r)
    data.pose.pose.orientation.w = q_updated[0]
    data.pose.pose.orientation.x = q_updated[1]
    data.pose.pose.orientation.y = q_updated[2]
    data.pose.pose.orientation.z = q_updated[3]
    pub.publish(data)


def transform_callback_a(data):       
    data.transforms.transform.translation.z = 0  
    # w = data.pose.pose.orientation.w
    # x = data.pose.pose.orientation.x
    # y = data.pose.pose.orientation.y
    # z = data.pose.pose.orientation.z
    # q = [w, x, y, z]
    # (yaw, pitch, roll) = quaternion_to_euler(q)
    # pitch = 0.0       
    # r = [yaw, pitch, roll]
    # q_updated = euler_to_quaternion(r)
    # data.pose.pose.orientation.w = q_updated[0]
    # data.pose.pose.orientation.x = q_updated[1]
    # data.pose.pose.orientation.y = q_updated[2]
    # data.pose.pose.orientation.z = q_updated[3]
    pub.publish(data)


def dxl_control():
    rospy.init_node('zed_zControl', anonymous=True)
    rospy.Subscriber('/zed2i/zed_node/odom', Odometry, transform_callback)
    rospy.Subscriber('/tf', Odometry, transform_callback_a)
    # Initial movement.
    rospy.spin()


if __name__ == '__main__':
    try:
        dxl_control()
        
    except rospy.ROSInterruptException:
        pass
