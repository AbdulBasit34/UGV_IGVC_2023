#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
import math
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
'''
TO understand
Minimize all the function except the main,
then start reading from there 
then start opening function as you encounter one.

imagesShow() is complimentary if you want to see all the images

'''
#=========================================================================================
#=========================================================================================
# openCV imshow Plots of all the images
def imagesShow():

  cv2.imshow('masked_img', masked_img)              #masked
  cv2.imshow('warped_img', warped_img)              #wrapped
  cv2.imshow('thresh', thresh)                      #Binary Thresholded
  cv2.imshow('reconst_img', reconst_img)            #reconstructed
  cv2.imshow('Final Image', final_img)

  print("In ImgShow")
  print("Type of final Image = ",type(final_img))

  cv2.waitKey(0)          
  cv2.destroyAllWindows()
  
  print("Image Show Successful !!")

#Searches white pixels in an input image
def whitePixelSearch(img):
  #global IMAGE_H
  #global IMAGE_W
  
  indices=[]
  for i in range(0,IMAGE_H):
    j = 0
    while j < IMAGE_W:
      if img[i,j]==255.0:         # If the edge of lane is hit.
        indices.append([j+20,i])  # Assumung lane is 40px wide.
        #print(j)
        j += 100                  # To skip useless search in blank space between lanes  
        #print(j)
        #print("#################")
      j+= 1
  #print(type(indices))
  #print(len(indices))        

  return indices

#Creates and publishes Final Image message to rviz
def Img_msg_Publisher():

  global count      #just fancy stuff (for loop counter)
  count += 1

  #------------------Publish Final Image to rviz-----------------------

  image_pub = rospy.Publisher("/ugvbot/image_processed/compressed",CompressedImage, queue_size = 1)

  msg = CompressedImage()
  msg.header.stamp = rospy.Time.now()
  msg.format = "jpeg"
  msg.data = np.array(cv2.imencode('.jpg', final_img)[1]).tostring()
  
  # Publish Final Image
  image_pub.publish(msg)

  print("#######################-"+ str(count) +"-############################")
  if VERBOSE :
    rospy.loginfo("From Image Msg Publisher")
    print("Msg Format = ",msg.format)
    print("Shape of Image = ",final_img.shape)
    print("-------------------------------------------------")
  else :
    print("Publishing Final Image.....Bitches!")

  #rate = rospy.Rate(20)
  #rate.sleep()

#Processes Image and Calls Img_msg_Publisher at end.
def Img_Processor(ros_data):

  global warped_img  
  global masked_img
  global warped_img
  global thresh
  global reconst_img
  global final_img
  
  global IMAGE_H
  global IMAGE_W


  if VERBOSE :
    print("-------------------------------------------------")
    rospy.loginfo("From Image Processor")
    print("Type of Received Image = "+ str(ros_data.format))
    print("Shape of Recieved Image = ",ros_data.shape)
  

  #### direct conversion to CV2 ####
  np_arr = np.fromstring(ros_data.data, np.uint8)
  img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

  #Image Dimensions
  IMAGE_H = img.shape[0]  # assuming 720
  IMAGE_W = img.shape[1]  # assuming 1280 
  #print(IMAGE_W)

  #-----------------------Masking the Image----------------------------
  #[0,0]------->--------[W,0]
  #  |                    |
  #  ^                    v
  #  |                    |
  #[0,H]-------<-------[W,H]
  
  edges = np.array([[0, 0], [IMAGE_W, 0], [IMAGE_W, IMAGE_H], [0, IMAGE_H]])      #full image polygon
  
  #ROI Start 

  src_H = 275
  #src_H = 325

  stencil = np.zeros_like(img[:,:,0])
  polygon = np.array([[0, src_H], [IMAGE_W, src_H], [IMAGE_W, 400], [0, 400]])    # specify coordinates of the ROI polygon
  cv2.fillConvexPoly(stencil, polygon, 1)                                         # fill polygon with ones
  masked_img = cv2.bitwise_and(img[:,:,:], img[:,:,:], mask=stencil)              # masking the image 

  #------------------------Transforming ROI ----------------------------
  a=500

  src = np.float32([[0, src_H], [IMAGE_W, src_H], [IMAGE_W, 400], [0, 400]])      #Source Polygon coordinates
  dst = np.float32([[0, src_H], [IMAGE_W, src_H], [IMAGE_W-a, 400], [a, 400]])    #Destination Polygon coordinates

  M = cv2.getPerspectiveTransform(src, dst)                                       # The prespective transformation matrix
  Minv = cv2.getPerspectiveTransform(dst, src)                                    # Inverse transformation matrix 

  warped_img = cv2.warpPerspective(masked_img, M, (IMAGE_W, IMAGE_H))
  reconst_img = cv2.warpPerspective(warped_img,Minv,(IMAGE_W, IMAGE_H))

  #------------------------Lanes Extraction------------------------------
  #Making Hough Lines on ROI

  _, thresh = cv2.threshold(cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY), 80, 255, cv2.THRESH_BINARY)
  lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30 ,minLineLength=100, maxLineGap=10)

  # create a copy of size of the thresholded frame
  final_img = np.zeros(thresh.shape)

  # draw Hough lines
  if( lines is None):
      pass
  else:
      for line in lines:
          x1, y1, x2, y2 = line[0]
          cv2.line(final_img, (x1, y1), (x2, y2), (255, 255, 255), 5)

  #Multiple figures
  #imagesShow()

  Img_msg_Publisher()

  laser_processor()

#=========================================================================================
#=========================================================================================
# Calculates Polar Cordinates for 3 given cartesian coordinates
def calculatePolarCord(x3, y3): 
  '''
  the x coordinates increases from left to right
  while the y coordinates increses from top to bottom which 
  leaves us with no standard coordinate system so 
  to do polar angle and radius calculation we make the image appear
  in 4th quadrant where x increase left to right and y increase top to bottom but in the negative range
  so the index of y coordinate is negated everywhere   
  '''
  #   C(i , -j) 0   0 B (IMAGE_W/2 , 0)---BOT's HEADING
  #              \  |
  #               \ |
  #                \|
  #                 0 A (IMAGE_W/2 , -IMAGE_H)---BOT's POSITION

  #bot position in image (IMAGE_W/2 , IMAGE_H)
  x1 ,y1 = IMAGE_W/2 , -IMAGE_H
  #Reference Point for angle
  x2 ,y2 = IMAGE_W/2 , 0
  # White lanes cordinate in image
  x3 ,y3 = x3 , -y3

  # Find direction ratio of line AB 
  ABx = x2 - x1 
  ABy = y2 - y1 

  # Find direction ratio of line AC 
  ACx = x3 - x1 
  ACy = y3 - y1 

  # Find the dotProduct of lines AB & AC 
  dotProduct = (ABx * ACx + ABy * ACy)

  # square of magnitude of line AB and AC 
  magABsq = (ABx * ABx + ABy * ABy )        
  magACsq = (ACx * ACx + ACy * ACy )

  # cosine of the angle formed by line AB and AC 
  angle = dotProduct
  angle /= math.sqrt(magABsq * magACsq)

  angle = math.acos(angle)

  if x3 < IMAGE_W/2:
    angle = angle 
  if x3 >= IMAGE_W/2:
    angle = -1*angle

  return [angle, math.sqrt(magACsq)]

#Indirect Function for calculatePolarCord of a list of points
def cartToPolar(cart):
  indices =[]

  for i in cart:
    index = calculatePolarCord(i[0],i[1])
    indices.append(index)

  return indices

#Converts distaces using functions (from image distance to actual distances)
def converterForRviz(image_polar_list):
  # y = a.exp(-b.t) <-- input image 
  # a = 1012
  # b = 0.0156

  real_polar=[]

  for i in image_polar_list:

    theta = i[0]
    radius = i[1]

    rcos = radius*math.cos(theta)       

    # Not accurate but gets work done under --no-obstacle--only-lanes-- conditions
    # Got the Y direction function by curve fitting 
    # and the X distance function is just rough estimate as its not that important

    ar_cos = 1012* math.exp(-1*0.0156*(720-rcos))                               #Got by measuring how actual distances in the vertical dir. of image

    ar_sin = radius*math.sin(theta)                                             #Got by measuring how actual distances in the horizontal dir. of image
    ar_sin = (ar_sin*3)/125                                                     #change of 3m from centre to 125px left.
                                                  
        
    actual_theta = math.atan(ar_sin/ar_cos)
    actual_radius = math.sqrt(ar_sin*ar_sin + ar_cos*ar_cos)
    
    real_polar.append([actual_theta, actual_radius])

  return real_polar

#Creates and publishes Laser message for lanes to rviz 
def laser_msg_publisher(ranges_list):
  #------------------Publish Fake Laser to rviz-----------------------
  laser_pub = rospy.Publisher('ugvbot/fake_scan', LaserScan, queue_size=1)

  angle_min=-(math.pi)/2
  angle_max=math.pi/2
  intensities=[]

  scan_rate = 10
  rate=rospy.Rate(scan_rate)

  start_time = rospy.Time.now()
  angle_increment=(angle_max-angle_min)/360
  time_increment=1/scan_rate

  msg=LaserScan()
  msg.header.stamp = start_time
  msg.header.frame_id = "laser"
  msg.angle_min=angle_min
  msg.angle_max=angle_max
  msg.angle_increment=angle_increment   # Angle Increment
  msg.time_increment=time_increment     # Time Increment
  msg.range_min=0                       # If range < MinRange range = 0 
  msg.range_max=100                     # If range > MaxRange range = inf 
  msg.ranges=ranges_list                # Range of Lanes pixel
  msg.intensities=intensities           # Intensities empty

  # Publish fake LaserScan 
  laser_pub.publish(msg)

  if VERBOSE :
    rospy.loginfo("From Laser message Publisher")
    print("Angle Increment", msg.angle_increment)
    print("length of Range list = ",len(msg.ranges))
  else :
    print("           Fake Laser......Bitches!")

  rate.sleep()

#Processes Final Image and finds polar cordinates wrt to bot
#Also calls laser_msg_publisher at end
def laser_processor():

  if type(final_img) is np.ndarray:

    # Indices of white pixels form image
    cart = whitePixelSearch(final_img)
    #cart_plot()  

    # Cartesian coordinates from image to polar cordinates of lanes wrt robot.
    polar = cartToPolar(cart)
    #polar_plot()

    actual_polar = converterForRviz(polar)
    #polar_plot(actual_polar)

    ranges=[1000]*360       # Cross Verify length from laser msg Defination in laser_msg_publisher
    
    #degree 0.5
    for i in actual_polar:
      degree = ((i[0])*180/np.pi + 90)*2
      ranges[int(round(degree,0))] = i[1]

    laser_msg_publisher(ranges)

  else :
    print("##################################")
    print("Shit Happened in laser_processor!!")
    print("Type of final Image = ",type(final_img))
    exit(1)
  
#=========================================================================================
#=========================================================================================
if __name__ == '__main__':
  #-----------------Global Variables Used-------------------
  #For Image processing
        # masked_img     Masked with ROI
        # warped_img     prespective transformed 
        # thresh         thresholded to 0.0 or 255.0
        # reconst_img    Reconstructed From Inverse Prespective
        # final_img      Image with Hough lines on thresh 
        # IMAGE_H        Image Height 720
        # IMAGE_W        Image Width 1280


  count = 0
  VERBOSE = False

  rospy.init_node('Lanes_Processor_N_Publisher', anonymous=False)
  rospy.Subscriber("/ugvbot/image_raw/compressed",CompressedImage, Img_Processor,  queue_size = 1)

  try:
    print("inside main's try")
    #os.system('clear')                                  #cleans the mess we made on last iteration
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down ROS Lanes_Processor_N_Publisher module")
