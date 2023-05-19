#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist



def talker():
    pub = rospy.Publisher('/ugvbot/velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('test_publisher', anonymous=False)
    rate = rospy.Rate(400) # 10hz
    msg = Twist()

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        print("Publishing !!!!")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
