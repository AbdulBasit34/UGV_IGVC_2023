
import os
import matplotlib.pyplot as plt
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage

import cv_bridge as CvBridge

import cv2

# theta = np.array([0.36,-0.47])
# radius = np.array([338.9528,356.8865])

# plt.polar(theta,radius,'g.')
# plt.show()

# print("rcos of 1",radius[1]*np.cos(-0.47))


#print(os.path.dirname(os.path.abspath(__file__)))
#print(os.path.abspath(os.getcwd())+"/shie")

# directory = os.path.dirname(os.getcwd())
# st = "catkin_ws/src/ugv_bot/Scripts/test_img.png"
# print(directory+st)

def callback():

  rospy.init_node('save_img')
  j=0
  while not rospy.is_shutdown():

      ros_data = rospy.wait_for_message('/ugvbot/image_raw/compressed', CompressedImage)  

      np_arr = np.fromstring(ros_data.data, np.uint8)
      img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

      #print("Saved to: ", st)
      #cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)
      cv2.imshow('masked_img', img)              #masked
      cv2.waitKey(0)
      cv2.destroyAllWindows()
      #cv2.imwrite(st, img)
      j=j+1
      print(j)
      
      rospy.spin() 

if __name__ == "__main__":
  directory = os.path.dirname(os.getcwd())
  st = directory + "catkin_ws/src/ugv_bot/Scripts/new_img.png"
  callback()

  #sub = rospy.Subscriber("/ugvbot/image_raw/compressed",CompressedImage, callback,  queue_size = 1)
  #print(sub)
  #rospy.spin()
