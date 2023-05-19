#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu



vl=0
w_zed = 0
w =0 


def cmd_vel_callback(data):
    global vl
    global w
    #To print data and ***** for delimiting.
    # print(data)
    # print("********")

    vl=data.linear.x
    w = data.angular.z
    print("real : ", w)

def odom_callback(data):
    #To print data and ***** for delimiting.

    global vl
    global w_zed
    global w

    # print(data)
    # print("********")

    w_zed = data.angular_velocity.z
    print("Zed : " , w_zed)

    
def contol():
    
    global vl
    global w
    global w_zed

    # Inititate node named "test_subscriber"
    rate = rospy.Rate(10) # 10hz

    pub = rospy.Publisher('/setpoint', Float64, queue_size=10)
    pub_zed = rospy.Publisher('/state', Float64, queue_size=10)

    while not rospy.is_shutdown():
        pub.publish(w)
        pub_zed.publish(w_zed)

        rate.sleep()
        print("Publishing !!!!")


if __name__ == '__main__':
    rospy.init_node('test_control', anonymous=False)

    #set test_subscriber to subscribe form topic testing_temp the data of type temp
    rospy.Subscriber("/ugvbot/velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped

    try:
        contol()
    except rospy.ROSInterruptException:
        pass