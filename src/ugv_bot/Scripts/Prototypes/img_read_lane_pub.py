#!/usr/bin/env python
from __future__ import division, print_function
import os

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import LaserScan

def rgba2rgb( rgba, background=(255,255,255) ):
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros( (row, col, 3), dtype='float32' )
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]

    a = np.asarray( a, dtype='float32' ) / 255.0

    R, G, B = background

    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray( rgb, dtype='uint8' )

def imagesShow():
  cv2.imshow('masked_img', masked_img)              #masked
  cv2.imshow('warped_img', warped_img)              #wrapped
  cv2.imshow('thresh', thresh)                      #Binary Thresholded
  cv2.imshow('reconstructed_img', reconstructed_img)#reconstructed
  cv2.imshow('Final Image', final_img)

  cv2.waitKey(0)          
  cv2.destroyAllWindows()

def whitePixelSearch(img):
  indices=[]
  for i in range(0,IMAGE_H):
    j = 0
    while j < IMAGE_W:
      if img[i,j]==255.0:
        indices.append([j+20,i])
        #print(j)
        j += 100
        #print(j)
        #print("#################")
      j+= 1
  #print(type(indices))
  #print(len(indices))        

  return indices

def calculatePolarCord(x3, y3): 
  '''
  the x coordinates increases from left to right
  while the y coordinates increses from top to bottom which 
  leaves us with no standard coordinate system so 
  to do polar angle and radius calculation we make the image appear
  in 4th quadrant where x increase left to right and y increase top to bottom but in the negative range
  so the index of y coordinate is negated everywhere   
  '''
  #   C(i , -j) 0   0 B (IMAGE_W/2 , 0)
  #              \  |
  #               \ |
  #                \|
  #                 0 A (IMAGE_W/2 , -IMAGE_H)

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

def cartToPolar(cart):
  indices =[]

  for i in cart:
    index = calculatePolarCord(i[0],i[1])
    indices.append(index)

  return indices

def polar_plot(polar_list):
  
  print(type(polar_list))
  print(len(polar_list))
  
  pol = np.array(polar_list)
  print(type(pol))  
  print(pol.shape)
 
  plt.polar(pol[:,0],pol[:,1],'g.')
  plt.show()

def cart_plot(cart_data):
  
  tmp = np.zeros((IMAGE_H, IMAGE_W))
  print(type(cart_data))
  print(len(cart_data))
  
  for i in cart_data:
    print(i)
    tmp[i[1]][i[0]] = 255.0

  print(tmp.shape)
  plt.imshow(tmp)
  plt.show()

def converterForRviz(image_polar_list):
  # y = a.exp(-b.t) <-- input image 
  # a = 1012
  # b = 0.0156

  real_polar=[]

  for i in image_polar_list:

    theta = i[0]
    radius = i[1]

    rcos = radius*math.cos(theta)       

    r_cos = 1012* math.exp(-1*0.0156*(720-rcos))
    r_sin = radius*math.sin(theta)
    r_sin = (r_sin*3)/140
        
    theta_ = math.atan(r_sin/r_cos)
    radius_ = math.sqrt(r_sin*r_sin + r_cos*r_cos)
    
    real_polar.append([theta_, radius_])

  return real_polar

def generate(scan_rate, angle_min, angle_max, ranges, intensities):
  scan=LaserScan()
  start_time = rospy.Time.now()
  angle_increment=(angle_max-angle_min)/360
  time_increment=1/scan_rate

  scan.header.stamp = start_time
  scan.header.frame_id = "laser"
  scan.angle_min=angle_min
  scan.angle_max=angle_max
  scan.angle_increment=angle_increment
  scan.time_increment=time_increment
  scan.range_min=0
  scan.range_max=100
  scan.ranges=ranges
  scan.intensities=intensities

  return scan

def rviz_publisher(ranges_list):

  while not rospy.is_shutdown():
    angle_min=-(math.pi)/2
    angle_max=math.pi/2
    intensities=[]

    scan_rate = 10
    rate=rospy.Rate(scan_rate)

    scan=generate(scan_rate, angle_min, angle_max, ranges_list, intensities)

    pub.publish(scan)

    #rospy.loginfo("Publishing")

    print("Last element in polar ",polar[-1])
    print("Last element in polar ",polar[-2])
    print("----------------------------------")
    print("Last element in actual ",actual_polar[-1])
    print("Second Last element in actual ",actual_polar[-2])
    print("===================================================")

  rate.sleep()

if __name__ == '__main__':
  
  img = cv2.imread(os.path.dirname(os.getcwd())+"catkin_ws/src/ugv_bot/Scripts/test_img.png",cv2.IMREAD_UNCHANGED) # Read the test img
  img = rgba2rgb(img)

  IMAGE_H = img.shape[0]  # assuming 720
  IMAGE_W = img.shape[1]  # assuming 1280 

  #-----------------------Masking the Image----------------------------
  #[0,0]------->--------[W,0]
  #  |                    |
  #  ^                    v
  #  |                    |
  #[0,H]-------<-------[W,H]
   
  edges = np.array([[0, 0], [IMAGE_W, 0], [IMAGE_W, IMAGE_H], [0, IMAGE_H]])      #full image polygon
  
  stencil = np.zeros_like(img[:,:,0])
  polygon = np.array([[0, 275], [IMAGE_W, 275], [IMAGE_W, 400], [0, 400]])        # specify coordinates of the ROI polygon
  cv2.fillConvexPoly(stencil, polygon, 1)                                         # fill polygon with ones
  masked_img = cv2.bitwise_and(img[:,:,:], img[:,:,:], mask=stencil)              # masking the image 

  #--------------------Transforming ROI --------------------
  a=500

  src = np.float32([[0, 275], [IMAGE_W, 275], [IMAGE_W, 400], [0, 400]])          #Source Polygon coordinates
  dst = np.float32([[0, 275], [IMAGE_W, 275], [IMAGE_W-a, 400], [a, 400]])        #Destination Polygon coordinates

  M = cv2.getPerspectiveTransform(src, dst)                                       # The prespective transformation matrix
  Minv = cv2.getPerspectiveTransform(dst, src)                                    # Inverse transformation matrix 

  warped_img = cv2.warpPerspective(masked_img, M, (IMAGE_W, IMAGE_H))
  reconstructed_img = cv2.warpPerspective(warped_img,Minv,(IMAGE_W, IMAGE_H))

  #--------------------Image Processing------------------------
  #Making Hough Lines on ROI

  ret, thresh = cv2.threshold(cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY), 80, 255, cv2.THRESH_BINARY)
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
  
  #------------------------------Preprations for rviz------------------------------
  # y = a.exp(-b.t) <-- input image 
  # a = 1012
  # b = 0.0156

  # Indices of white pixels form image
  cart = whitePixelSearch(final_img)
  #cart_plot()  

  # Cartesian coordinates from image to polar cordinates of lanes wrt robot.
  polar = cartToPolar(cart)
  #polar_plot(polar)

  actual_polar = converterForRviz(polar)

  ranges=[100]*360


  #degree 0.5
  for i in actual_polar:
    degree = ((i[0])*180/np.pi + 90)*2
    ranges[int(round(degree,0))] = i[1]

  rospy.init_node('LaserScan', anonymous='True')
  pub = rospy.Publisher('ugvbot/fake_scan', LaserScan, queue_size=1)


  try:
    rviz_publisher(ranges)
  except KeyboardInterrupt:
    print("Shutting down ROS Image feature detector module")
  cv2.destroyAllWindows()



  #polar_plot(actual_polar)

  # Reconstruct Cartesian coordinates from polar coordinates.
  '''
  x = []
  y = [] 
  for i in polar:
    x.append(i[1]*math.sin(i[0]))
    y.append(i[1]*math.cos(i[0]))

  plt.scatter(x,y)
  plt.show()
  '''


