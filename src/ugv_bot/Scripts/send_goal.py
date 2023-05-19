#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations
import utm
import math

# Define initial UTM coordinates
initial_utm_x = None
initial_utm_y = None

current_utm_x = None
current_utm_y = None

def odom_callback(data):
    global initial_utm_x, initial_utm_y
    global current_utm_x, current_utm_y

    # Get UTM coordinates from the Odometry message
    goal_utm_x = data.pose.pose.position.x
    goal_utm_y = data.pose.pose.position.y
    
    current_utm_x = goal_utm_x
    current_utm_y = goal_utm_y

    # If initial coordinates haven't been set, set them now
    if initial_utm_x is None or initial_utm_y is None:
        initial_utm_x = goal_utm_x
        initial_utm_y = goal_utm_y
        rospy.loginfo('Set initial UTM coordinates to (%f, %f)', initial_utm_x, initial_utm_y)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('gps_waypoint')

    # Create a subscriber to the /ublox_gps/odom topic
    rospy.Subscriber('/ublox_gps/odom', Odometry, odom_callback)
    while initial_utm_x is None or initial_utm_y is None:
          rospy.sleep(1.0)  # sleep for 1 second
    
    rospy.loginfo('Sending goal.')


    # Create a publisher to the move_base goal topic
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    

    # Define your waypoints as (latitude, longitude) pairs
    waypoints = [(28.74991253,77.1169595)]  # Fill in your actual latitude and longitude values
    
    rate = rospy.Rate(1)

    # For each waypoint
    for waypoint in waypoints:

        # Convert the waypoint's latitude and longitude to UTM coordinates
        waypoint_utm = utm.from_latlon(*waypoint)

        # Create a goal pose
        goal = PoseStamped()

        # Set the goal pose's frame and time
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        #goal.header.seq = 2

        # Set the goal pose's position and orientation
        goal.pose.position.x = waypoint_utm[0]
        goal.pose.position.y = waypoint_utm[1]
        goal.pose.position.z = 0  # Assume the robot is on the ground
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # No rotation
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        
        #print(goal)
        
        while goal_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers... ")
            rospy.sleep(1.0)  # sleep for 1 second
        
        rospy.loginfo("Subscriber found, publishing message.")

        # Publish the goal pose
        goal_pub.publish(goal)
        
        while True:
            # Calculate the Euclidean distance between the current location and the goal
            distance = math.sqrt((goal.pose.position.x - current_utm_x)**2 +
                                 (goal.pose.position.y - current_utm_y)**2)
            
            print(f"\rDistance to goal: {distance} ({goal.pose.position.x}, {goal.pose.position.y}) -> ({current_utm_x}, {current_utm_y})", end="")

            # If the distance is less than 1 meter, break the loop and proceed to the next waypoint
            if distance < 1:
                break

            # Sleep for a while before checking the distance again
            rospy.sleep(0.5)

    # Keep the node alive until it's shut down
    rospy.spin()


