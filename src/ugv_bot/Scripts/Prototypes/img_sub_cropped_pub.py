#!/usr/bin/env python
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage


def callback(ros_data):

  # topic where we publish
  image_pub = rospy.Publisher("/ugvbot/image_raw/compressed",CompressedImage, queue_size = 1)

  #### direct conversion to CV2 ####
  np_arr = np.fromstring(ros_data.data, np.uint8)
  image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

  IMAGE_H = image_np.shape[0]
  IMAGE_W = image_np.shape[1]

  stencil = np.zeros_like(image_np[:,:,0])

  # specify coordinates of the polygon 
  # (_, 330) px is roughly 3.75m away infront
  polygon = np.array([[0, 330], [IMAGE_W, 330], [IMAGE_W, IMAGE_H], [0, IMAGE_H]])        # specify coordinates of the ROI polygon

  # fill polygon with ones
  cv2.fillConvexPoly(stencil, polygon, 1)

  masked_img = cv2.bitwise_and(image_np[:,:,:], image_np[:,:,:], mask=stencil)

  cv2.imshow('masked_img', masked_img)
  cv2.waitKey(2)

  ''''''
  #### Create CompressedIamge ####
  msg = CompressedImage()
  msg.header.stamp = rospy.Time.now()
  msg.format = "jpeg"
  msg.data = np.array(cv2.imencode('.jpg', masked_img)[1]).tostring()
  
  # Publish new image
  image_pub.publish(msg)
  
  rate = rospy.Rate(20)
  rate.sleep()


if __name__ == '__main__':
    '''Initializes and cleanup ros node'''

    rospy.init_node('image_cropper', anonymous=True)
    try:
        sub_data = rospy.Subscriber("/ugvbot/image_raw/compressed",CompressedImage, callback,  queue_size = 1)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()