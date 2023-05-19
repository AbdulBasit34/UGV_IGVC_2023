#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import math
import matplotlib.pyplot as plt

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
  ABx = x1 - x2 
  ABy = y1 - y2 

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

  # angle in radian 
  #angle = (angle * 3.14) / 180

  # Print angle 
  #print(round(abs(angle), 4)) 

  return [angle, math.sqrt(magACsq)]

def cartToPolar(cart):
  indices =[]

  for i in cart:
    index = calculatePolarCord(i[0],i[1])
    indices.append(index)

  return indices

if __name__ == '__main__':

  img_size = 100
  IMAGE_W = img_size
  IMAGE_H = img_size 

  cart = np.zeros((img_size,img_size))

  for i in range(0,img_size):
    for j in range(0,img_size):
      if j == 25 or j ==75:
        cart[i][j] = 255.0

  print(type(cart))
  print(len(cart))

  #plt.imshow(cart)
  #plt.show()

  polar = cartToPolar(cart)
  #print(type(polar))
  #print(len(polar))

  pol = np.array(polar) # List to Array
  print(type(pol))  
  print(pol.shape)

  #print(pol[:,0],pol[:,1])

  plt.polar(pol[:,0],pol[:,1],'g.')
  plt.show()

