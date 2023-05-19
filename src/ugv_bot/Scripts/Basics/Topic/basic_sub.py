#!/usr/bin/env python
import rospy
from my_mouse.msg import basic_message as basic_msg_type

def callback(data):
    #To print data and ***** for delimiting.
    print(data)
    print("********")
    
def listener():
    # Inititate node named "test_subscriber"
    rospy.init_node('test_subscriber', anonymous=False)

    #set test_subscriber to subscribe form topic testing_temp the data of type temp
    rospy.Subscriber("testing_temp", basic_msg_type, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()