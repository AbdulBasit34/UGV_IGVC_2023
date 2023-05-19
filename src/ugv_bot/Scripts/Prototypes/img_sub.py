#!/usr/bin/env python
import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import CompressedImage

import sys, os

def subscriber(ros_data):
  global count
  count+=1
  print("###########-----"+str(count)+"----###########")

  if VERBOSE :
    print("-------------------------------------------------")
    rospy.loginfo("From Image Processor")
    print("Format of Received Image = "+ str(ros_data.format))
    print("Time of acquisition of image Recieved Image = ",str(ros_data.header.stamp))
  

  #### direct conversion to CV2 ####
  np_arr = np.fromstring(ros_data.data, np.uint8)
  #print("type of np_arr",type(np_arr))
  #print("shape of np_arr",np_arr.shape)
  #print("-----------------------------------")
  img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
  #print("type of img",type(img))
  #print("shape of img",img.shape)



  #print('sys.argv[0] =', sys.argv[0])             
  pathname = os.path.dirname(sys.argv[0])        

  cv2.imwrite(str(pathname) + "/current_img.jpeg", img) 
  print("image written successfully")

  rate=rospy.Rate(0.5)
  rate.sleep()


if __name__ == '__main__' :
  VERBOSE = False

  count =0
  rospy.init_node('img_subscriber', anonymous=False)
  rospy.Subscriber("/ugvbot/image_raw/compressed",CompressedImage, subscriber, queue_size = 1)

  try:
    rospy.spin()
  except KeyboardInterrupt:
		print("shit !!!!!")
