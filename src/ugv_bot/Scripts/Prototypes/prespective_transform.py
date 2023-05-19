from __future__ import division, print_function
import os

import cv2
import numpy as np
import matplotlib.pyplot as plt

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


if __name__ == '__main__':
    img = cv2.imread(os.getcwd()+"/catkin_ws/src/ugv_bot/Scripts/test_img.png",cv2.IMREAD_UNCHANGED) # Read the test img
    img = rgba2rgb(img)

    IMAGE_H = img.shape[0]  # assuming 720
    IMAGE_W = img.shape[1]  # assuming 1280 

    ###########--Image--##########
    #[0,0]------->--------[W,0]
    #  |                    |
    #  ^                    v
    #  |                    |
    #[0,H]-------<-------[W,H]
    ##############################
    
    edges = np.array([[0, 0], [IMAGE_W, 0], [IMAGE_W, IMAGE_H], [0, IMAGE_H]])      #full image polygon
    
    stencil = np.zeros_like(img[:,:,0])
    polygon = np.array([[0, 275], [IMAGE_W, 275], [IMAGE_W, 400], [0, 400]])        # specify coordinates of the ROI polygon
    cv2.fillConvexPoly(stencil, polygon, 1)                                         # fill polygon with ones
    masked_img = cv2.bitwise_and(img[:,:,:], img[:,:,:], mask=stencil)              # masking the image 

    #----------------------------------
    # Transforming ROI 
    a=500

    src = np.float32([[0, 275], [IMAGE_W, 275], [IMAGE_W, 400], [0, 400]])          #Source Polygon coordinates
    dst = np.float32([[0, 275], [IMAGE_W, 275], [IMAGE_W-a, 400], [a, 400]])        #Destination Polygon coordinates

    M = cv2.getPerspectiveTransform(src, dst)                                       # The prespective transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, src)                                    # Inverse transformation matrix 

    warped_img = cv2.warpPerspective(masked_img, M, (IMAGE_W, IMAGE_H))
    reconstructed_img = cv2.warpPerspective(warped_img,Minv,(IMAGE_W, IMAGE_H))
    #-----------------------------------

    cropped_img = warped_img[275:IMAGE_H, 0:IMAGE_W]

    # y = a.exp(-b.x)
    # a = 1012
    # b = 0.0156

    #14 = 275
    # 2 = 400
    # 0 = 720


    
    #-----------------------------------
    #Multiple figures

    cv2.imshow('masked_img', masked_img)    #masked
    cv2.imshow('warped_img', warped_img)    #wrapped

    #cv2.imshow('cropped_img', cropped_img)   #Cropped

    #cv2.imshow('reconstructed_img', reconstructed_img) #reconstructed


    cv2.waitKey(0)          
    cv2.destroyAllWindows() 



    # dst = np.float32([[0,0], [IMAGE_W, IMAGE_H], [a , IMAGE_H], [IMAGE_W-a,IMAGE_H]])
    # M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

    # img = cv2.imread('/home/jacksparrow/Pictures/test_img.png') # Read the test img

    # #plt.imshow(img)
    # #img = cv2.imread('/home/jacksparrow/catkin_ws/src/ugv_bot/lanes_extraction/test_img.jpeg')
    # print(type(img))
    # #img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
    # warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
    # plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
    # plt.show()