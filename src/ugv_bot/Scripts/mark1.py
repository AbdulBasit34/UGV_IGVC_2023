#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

pi=math.pi
x_dist = 0
y_dist= 0
lis = []
front=1
left=1
right=1
front_nav=1
left_nav=1
right_nav=1
pub = rospy.Publisher('/ugvbot/velocity_controller/cmd_vel', Twist, queue_size=10)


def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw_
    global lis
    
    

    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    x_dist = position_.x
    y_dist = position_.y




    
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    #print(x_dist, y_dist)
    lis=[x_dist,y_dist]

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)



def clbk_laser(msg):
    global front
    global left
    global right

    global front_nav
    global left_nav
    global right_nav


    right = min(min(msg.ranges[1:90]), 10)
    #'fright': min(min(msg.ranges[72:143]), 10),
    front = min(min(msg.ranges[177:183]), 10)
        #'fleft':  min(min(msg.ranges[216:287]), 10),
    left = min(min(msg.ranges[270:359]), 10)
    #print (front,left,right)

    print("left = ",left)
    print("front = ",front)
    print("######")    

    right_nav = min(min(msg.ranges[1:10]), 10)
    #'fright': min(min(msg.ranges[72:143]), 10),
    front_nav = min(min(msg.ranges[177:183]), 10)
        #'fleft':  min(min(msg.ranges[216:287]), 10),
    left_nav = min(min(msg.ranges[349:359]), 10)
    #print (front,left,right)
   

 
def GoToNextCell(init) :

    global pub
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()
    initial = init

    while not rospy.is_shutdown():
	
        current= lis
        
        
        # if front < 0.3 :
        #     msg1.linear.x = 0
        #     msg1.angular.z = 0
        #     pub.publish(msg1)
        #     print('case1')           
        #     rate.sleep()
        if (dist(initial,current)<0.18):
            if front < 0.08 :
                msg1.linear.x = 0
                msg1.angular.z = 0
                pub.publish(msg1)
                print('case4')
                break
            
            if right < 0.08 :
                msg1.linear.x = 0.1
                msg1.angular.z = 0.1
                print('case1')
            elif left < 0.08 :
                msg1.linear.x = 0.1
                msg1.angular.z = -0.1
                print('case2')
            else:
            
                msg1.linear.x = 0.1
                msg1.angular.z = 0
                print('case3')
            
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.linear.x =0
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
	    break
    
def TurnLeft(inp) :
    
    global pub
    initial=inp
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw_      
        
        
        if (abs(current-initial)<pi/2):
            msg1.linear.x = 0
    
            msg1.angular.z = 0.5
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            break

        

def TurnRight(inp) :
    
    global pub
    initial=inp
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw_      
        
        
        if (abs(current-initial)<pi/2):
            msg1.linear.x = 0
    
            msg1.angular.z = -0.5
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            break

        
def is_front_clear() :
    global front_nav
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    print (front_nav)
    if (front_nav>0.12) :
	return True
    else :
        return False    

def is_left_clear() :
    global left_nav
    print (left_nav)
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if (left_nav>0.12) :
	return True
    else :
        return False    


def is_right_clear() :
    global right_nav
    print (right_nav)
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if (right_nav>0.12) :
	return True
    else :
        return False    

def BackTrack() :
    while (True) :
        if is_right_clear() :
            TurnRight(yaw_)
            GoToNextCell(lis)
        elif is_left_clear() :
            TurnLeft(yaw_)
            GoToNextCell(lis)
        elif is_front_clear() :
            GoToNextCell(lis)
        else :
            TurnLeft(yaw_)
            TurnLeft(yaw_)
            GoToNextCell(lis)
            BackTrack()

if __name__ == '__main__':
   
    sub_odom = rospy.Subscriber('/ugvbot/velocity_controller/odom', Odometry, clbk_odom) 
    sub = rospy.Subscriber('/ugvbot/scan', LaserScan, clbk_laser)  
    rospy.init_node('cmd_robot', anonymous=True)

    rospy.spin()

    # while (True) :
	# if is_front_clear() :
    #         GoToNextCell(lis)
    #     elif is_right_clear() :
    #         TurnRight(yaw_)
    #         GoToNextCell(lis)
    #     elif is_left_clear() :
    #         TurnLeft(yaw_)
    #         GoToNextCell(lis)
    #     else:
    #         TurnLeft(yaw_)
    #         TurnLeft(yaw_)
    #         GoToNextCell(lis)
    #         BackTrack()
    
    
    
