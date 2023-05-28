#!/usr/bin/env python3

print("Loading libraries, please wait (~20s)... \r", end="", flush=True)

import sys

import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf
import threading
import time
import tkinter as tk

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

#If you run into import issues, try enabling these:
#from std_msgs.msg import Header
#from geometry_msgs.msg import PointStamped
#import geometry_msgs

print("Loaded all libraries.                             ")


# -------------------------------------------------------------------

# CONSTANTS ZONE

# Don't modify stuff unless you know what you're doing!!

# Verbose
CONFIDENCE_THRESHOLD    =    0.55		# redundant
VERBOSE                 =    True		# show text
IMVERBOSE               =    True		# show images
INTERACTIVE             =    False		# allow modification of (some) global variables live

# Image Transformation (do not change unless lane-lidar merging is off)
IMAGE_W                 =    640
IMAGE_H                 =    360
HORIZON                 =    -190
SKEW                    =    190
Y_OFFSET                =    0.25

# ROS Publishing and Processing
PUBTOPIC                =    '/lanes/scan'	# final publishing topic
SCALE                   =    0.0045
CAMERA_DISTANCE         =    1
WHITE_THRESHOLD         =    255
OFFSET                  =    0

# Optimisation Parameters
USE_GPU                 =    True       # use gpu for cv2 (huge performance bonus)
HALFRES                 =    True       # process in intervals of 2 degrees
QUARTRES                =    False      # process in intervals of 4 degrees (experimental)
FPSCAP                  =    None       # set cap on how many fps we can output (experimental)

# --------------------------------------------------------------------

# Global vars

HORIZONTAL_SCALING_FACTOR =  2 * CAMERA_DISTANCE * math.tan(math.radians(WHITE_THRESHOLD / 2)) / IMAGE_W

bridge = None
img_received = False
current_img = None
final_img = None
ranges_list = None

r_x = int(IMAGE_W/2)
r_y = int(IMAGE_H)

angles = np.arange(0, 359) + OFFSET
angles[angles < 0] += 360
angles[angles >= 360] -= 360

dispvals = [0] * 360
for i, val in enumerate(dispvals):
	dispvals[i] = np.cos(np.deg2rad(i)) * Y_OFFSET

if FPSCAP is not None:
    min_duration = 1/FPSCAP

if USE_GPU:
    stream = cv2.cuda_Stream()


confidence_threshold_var = None
image_w_var = None
image_h_var = None
horizon_var = None
skew_var = None
scale_var = None
camera_distance_var = None
wthres_var = None
y_offset_var = None

def setup_sliders():
    def on_slider_change(*args):
        global CONFIDENCE_THRESHOLD
        global IMAGE_W
        global IMAGE_H
        global HORIZON
        global SKEW
        global SCALE
        global CAMERA_DISTANCE
        global WHITE_THRESHOLD
        global Y_OFFSET
        
        global confidence_threshold_var
        global image_w_var
        global image_h_var
        global horizon_var
        global skew_var
        global scale_var
        global camera_distance_var
        global wthres_var
        global y_offset_var

        CONFIDENCE_THRESHOLD = confidence_threshold_var.get()
        IMAGE_W = image_w_var.get()
        IMAGE_H = image_h_var.get()
        HORIZON = horizon_var.get()
        SKEW = skew_var.get()
        SCALE = scale_var.get()
        CAMERA_DISTANCE = camera_distance_var.get()
        WHITE_THRESHOLD = wthres_var.get()
        Y_OFFSET = y_offset_var.get()

    def run_tk():
        global CONFIDENCE_THRESHOLD
        global IMAGE_W
        global IMAGE_H
        global HORIZON
        global SKEW
        global SCALE
        global CAMERA_DISTANCE
        global WHITE_THRESHOLD
        global Y_OFFSET
        
        global confidence_threshold_var
        global image_w_var
        global image_h_var
        global horizon_var
        global skew_var
        global scale_var
        global camera_distance_var
        global wthres_var
        global y_offset_var
        
        root = tk.Tk()
        root.title("Sliders")

        confidence_threshold_var = tk.DoubleVar()
        image_w_var = tk.IntVar()
        image_h_var = tk.IntVar()
        horizon_var = tk.IntVar()
        skew_var = tk.IntVar()
        scale_var = tk.DoubleVar()
        camera_distance_var = tk.DoubleVar()
        wthres_var = tk.DoubleVar()
        y_offset_var = tk.DoubleVar()

        confidence_threshold_var.trace("w", on_slider_change)
        image_w_var.trace("w", on_slider_change)
        image_h_var.trace("w", on_slider_change)
        horizon_var.trace("w", on_slider_change)
        skew_var.trace("w", on_slider_change)
        scale_var.trace("w", on_slider_change)
        camera_distance_var.trace("w", on_slider_change)
        wthres_var.trace("w", on_slider_change)
        y_offset_var.trace("w", on_slider_change)

        slider_configs = [
            ("Confidence Threshold", confidence_threshold_var, 0.1, 1, 0.05, CONFIDENCE_THRESHOLD),
            ("Image Width", image_w_var, 360, 1920, 10, IMAGE_W),
            ("Image Height", image_h_var, 160, 1080, 10, IMAGE_H),
            ("Horizon", horizon_var, -500, 0, 10, HORIZON),
            ("Skew", skew_var, 0, 500, 10, SKEW),
            ("Scale", scale_var, 0.001, 0.01, 0.001, SCALE),
            ("Camera Distance", camera_distance_var, 0, 3, 0.1, CAMERA_DISTANCE),
            ("White Threshold", wthres_var, 150, 255, 1, WHITE_THRESHOLD),
            ("Y-Offset", y_offset_var, 0, 50, 0.1, Y_OFFSET),
        ]

        for i, (label_text, var, min_value, max_value, step, default_value) in enumerate(slider_configs):
            label = tk.Label(root, text=label_text)
            label.grid(row=i, column=0)
            slider = tk.Scale(root, from_=min_value, to=max_value, resolution=step, orient=tk.HORIZONTAL, variable=var)
            slider.set(default_value)
            slider.grid(row=i, column=1)

        root.mainloop()
    
    slider_thread = threading.Thread(target=run_tk, daemon=True)
    slider_thread.start()

def image_callback(data):
    global current_img
    global img_received
    
    try:
        current_img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        img_received = True
    except CvBridgeError as e:
        print(e)

def depth_callback(data):
    try:
        current_depth_img = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_img_received = True
    except CvBridgeError as e:
        logger.error("Depth image conversion error: %s", e)
        depth_img_received = False

#Creates and publishes Laser message for lanes to rviz 
def laser_msg_publisher():
    global VERBOSE
    global PUBTOPIC
    
    global ranges_list
    
    if ranges_list is None:
        return

    #------------------Publish Fake Laser to rviz-----------------------
    laser_pub = rospy.Publisher(PUBTOPIC, LaserScan, queue_size=1)

    angle_min=-(math.pi)
    angle_max=math.pi
    intensities=[]

    scan_rate = 10
    rate=rospy.Rate(scan_rate)

    start_time = rospy.Time.now()
    angle_increment=(angle_max-angle_min)/360
    time_increment=1/scan_rate

    msg=LaserScan()
    msg.header.stamp = start_time
    msg.header.frame_id = "new_laser"
    msg.angle_min=angle_min
    msg.angle_max=angle_max
    msg.angle_increment=angle_increment     # Angle Increment
    msg.time_increment=time_increment         # Time Increment
    msg.range_min=0                                             # If range < MinRange range = 0 
    msg.range_max=100                                         # If range > MaxRange range = inf 
    msg.ranges=ranges_list                                # Range of Lanes pixel
    msg.intensities=intensities                     # Intensities empty

    # Publish fake LaserScan 
    laser_pub.publish(msg)

    if VERBOSE :
        print(f"\rPublishing to: {PUBTOPIC}\tLast published: {start_time.secs:.2f}", end="")

    rate.sleep()

def publish_handler():
    rate = 30
    while True:
        start_time = time.time()
        laser_msg_publisher()
        time_spent = time.time() - start_time
        sleep_time = max(1.0 / rate - time_spent, 0)
        if sleep_time != 0:
            time.sleep(sleep_time)

#Processes Final Image and finds polar cordinates wrt to bot
#Also calls laser_msg_publisher at end
def laser_processor():

    global IMAGE_W
    global IMAGE_H
    global SCALE
    global CAMERA_DISTANCE
    global HORIZONTAL_SCALING_FACTOR
    global OFFSET
    global IMVERBOSE
    global INTERACTIVE
    global USE_GPU
    global Y_OFFSET

    global HALFRES
    global QUARTRES
    
    global final_img
    global angles
    global dispvals
    global r_x
    global r_y
    
    global ranges_list

    if type(final_img) is np.ndarray:
        # Initialize a list of ranges with a default value of 1000 for each angle
        ranges = [1000] * 360
        
        if INTERACTIVE:
            angles = np.arange(0, 359) + OFFSET
            angles[angles < 0] += 360
            angles[angles >= 360] -= 360
            for i, val in enumerate(dispvals):
                dispvals[i] = np.cos(np.deg2rad(i)) * Y_OFFSET
        
        x_plt_list = []
        y_plt_list = []

        if INTERACTIVE:
            r_x = int(IMAGE_W/2)
            r_y = int(IMAGE_H*CAMERA_DISTANCE)
        
        if USE_GPU:
            #print(final_img.shape)
            gpu_final_img = cv2.cuda_GpuMat()
            gpu_final_img.upload(final_img)
        
            for i, angle in enumerate(angles):
                if i > 90 and i < 270:
                    continue
                
                if HALFRES and i%2:
                    continue

                if QUARTRES and i%4:
                    continue
            
                M = cv2.getRotationMatrix2D((r_x, r_y), -i, 1.0)
                gpu_rotated = cv2.cuda.warpAffine(gpu_final_img, M, (IMAGE_W, int(IMAGE_H*CAMERA_DISTANCE)))
                rotated = gpu_rotated.download()
                colm = rotated[:, r_x]
                
                wpx_indices = np.where(colm[::-1] == 255)[0]
                if wpx_indices.size > 0:
                    distance = wpx_indices[0]
                    ranges[angle] = distance * SCALE
                    if IMVERBOSE:
                        x_plot = distance * math.cos(math.radians(angle))
                        y_plot = distance * math.sin(math.radians(angle))
                        x_plt_list.append(x_plot)
                        y_plt_list.append(y_plot)
        
        else:
            for i, angle in enumerate(angles):
                if i > 90 and i < 270:
                    continue
                
                if HALFRES and i%2:
                    continue
                
                M = cv2.getRotationMatrix2D((r_x, r_y), -i, 1.0)
                rotated = cv2.warpAffine(final_img, M, (IMAGE_W, IMAGE_H))
                colm = rotated[:, r_x]
                
                wpx_indices = np.where(colm[::-1] == 255)[0]
                if wpx_indices.size > 0:
                    distance = wpx_indices[0]
                    ranges[angle] = distance * SCALE + dispvals[i]
                    #print(f"{distance * SCALE} -> {ranges[angle]}")
                    if IMVERBOSE:
                        x_plot = distance * math.cos(math.radians(angle))
                        y_plot = distance * math.sin(math.radians(angle))
                        x_plt_list.append(x_plot)
                        y_plt_list.append(y_plot)
        
        #print(ranges)

        # Publish the range list as a laser scan message
        #laser_msg_publisher(ranges)
        #pubthread = threading.Thread(target=laser_msg_publisher, args=(ranges,))
        #pubthread.start()
        
        ranges_list = ranges

        if IMVERBOSE:
            plt.scatter(x_plt_list, y_plt_list)
            plt.xlabel("x_real")
            plt.ylabel("y_real")
            plt.title("Scatter plot of x_real and y_real points")
            plt.axis('equal')
            #plt.show()
        
        #cv2.imshow('out_img', out_img)
        #cv2.waitKey(1)
        
        
    else:
        print("##################################")
        print("Shit Happened in laser_processor!!")
        print("Type of final Image = ", type(final_img))

def live_predictions():
    global CONFIDENCE_THRESHOLD
    global IMAGE_W
    global IMAGE_H
    global HORIZON
    global SKEW
    global WHITE_THRESHOLD
    
    global VERBOSE
    global IMVERBOSE
    global INTERACTIVE
    
    global current_img
    global img_received
    global final_img
    global min_duration

    # Get video properties
    width = 1280
    height = 720

    # Subscribe to the ROS image topic
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/zed2i/zed_node/left_raw/image_raw_gray", Image, image_callback)
    #rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, depth_callback)
    
    camera_frame = "/zed2i_left_camera_frame"  # Replace with the actual camera frame
    base_frame = "base_link"  # Replace with the actual base frame
    
    # Add this before the while loop
    tf_listener = tf.TransformListener()
    
    print("Using python version", cv2.__version__)
    
    src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [IMAGE_W-SKEW, -HORIZON], [SKEW, -HORIZON]])      #Source Polygon coordinates
    dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [IMAGE_W, 0], [0, 0]])    #Destination Polygon coordinates

    M = cv2.getPerspectiveTransform(src, dst)                                       # The prespective transformation matrix

    stored_exception = None
    
    iterations = 0
    start_time = time.time()
    
    print("Starting lane detection.")

    try:
        while True:
            sttim = time.time() - start_time
            
            # Grab a frame from the ZED camera
            if not img_received:
                print("NO IMAGE RECEIVED")
                time.sleep(1)
                continue

            img = current_img
            img = cv2.resize(img, (IMAGE_W, IMAGE_H))
            _, img = cv2.threshold(img, WHITE_THRESHOLD, 255, cv2.THRESH_BINARY)

            #------------------------Transforming ROI ---------------------------#
            
            if INTERACTIVE:
                src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [IMAGE_W-SKEW, -HORIZON], [SKEW, -HORIZON]])      #Source Polygon coordinates
                dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [IMAGE_W, 0], [0, 0]])    #Destination Polygon coordinates

                M = cv2.getPerspectiveTransform(src, dst)                                       # The prespective transformation matrix
                #Minv = cv2.getPerspectiveTransform(dst, src)                                    # Inverse transformation matrix 
            
            warped_img = None

            if USE_GPU:
                img_gpu = cv2.cuda_GpuMat()
                img_gpu.upload(img)
                warped_img_gpu = cv2.cuda.warpPerspective(img_gpu, M, (IMAGE_W, int(IMAGE_H * CAMERA_DISTANCE)))
                final_img = warped_img_gpu.download()
                
                if INTERACTIVE:
                    cimg_gpu = cv2.cuda_GpuMat()
                    cimg_gpu.upload(current_img)
                    cimg_gpu = cv2.cuda.resize(cimg_gpu, (IMAGE_W, int(CAMERA_DISTANCE * IMAGE_H)))
                    cwarped_img_gpu = cv2.cuda.warpPerspective(cimg_gpu, M, (IMAGE_W, int(IMAGE_H * CAMERA_DISTANCE)))
                    cfinal_img = cwarped_img_gpu.download()
            else:
                img = cv2.resize(img, (IMAGE_W, int(IMAGE_H * CAMERA_DISTANCE)))
                final_img = cv2.warpPerspective(img, M, (IMAGE_W, int(IMAGE_H * CAMERA_DISTANCE)))
                if INTERACTIVE:
                    cfinal_img = cv2.warpPerspective(current_img, M, (IMAGE_W, int(IMAGE_H * CAMERA_DISTANCE)))
            
            if INTERACTIVE:
                cv2.imshow('Actual Image (Q to close)', current_img)
                cv2.waitKey(1)
                cv2.imshow('Actual Transformed Image (Q to close)', cfinal_img)
                cv2.waitKey(1)
                #print("displayed... ", end="", flush=True)
            
            # Replace the video writer with cv2.imshow() to display the output on the screen
            if IMVERBOSE:
                cv2.imshow('Lane Predictions (Q to close)', final_img)
                cv2.waitKey(1)
                #print("displayed... ", end="", flush=True)
            
            laser_processor()
            #print("published.", end="", flush=True)
            
            iterations += 1
            
            elpt = time.time() - start_time
            if elpt > 0:
                fps = iterations / elpt
                if FPSCAP is not None and fps > FPSCAP:
                    #passed = elpt-sttim
                    #print(f"{fps} > {FPSCAP} i.e. {min_duration} > {passed}")
                    time.sleep(abs(min_duration - (elpt-sttim)))
                    elpt = time.time() - start_time
                    fps = iterations / elpt
                print(f"\tFPS: {fps:.3f}", end="", flush=True)
            
            if stored_exception is not None:
                print("Encountered exception.")
                sys.exit()
    except:
        stored_exception=sys.exc_info()            

    cv2.destroyAllWindows()
    
    if stored_exception:
        print (stored_exception[0], stored_exception[1], stored_exception[2])
        sys.exit()

def init():
    
    print("Initialising parameters...\r", end="", flush=True)
    bridge = CvBridge()
    img_received = False
    current_img = None
    depth_img_received = False
    current_depth_img = None
    
#        transform = transforms.ToTensor()
    
    # Set up sliders in the __init__ function
    if INTERACTIVE:
        setup_sliders()
    
    publisher_thread = threading.Thread(target=publish_handler)
    publisher_thread.daemon = True  # Set the thread as a daemon so it exits when the main program exits
    publisher_thread.start()
    
    print("Initialised all parameters.     ")

def main():
    print("CAUTION: Do NOT directly run this script\nInstead run the following:\n\n\trosrun ugv_bot lane_wpx_wrapper.sh\n")
    
    count = cv2.cuda.getCudaEnabledDeviceCount()
    if count < 1:
        print(f"\nWARNING: OpenCV is not using the GPU! (GPUs detected: {count})\nThis will absurdly slow down the code if on Jetson! Please build OpenCV with support for CUDA/GPU.")
    else:
        print(f"\nINFO: Using {count} GPUs.")
    
#    if not torch.cuda.is_available():
#        print("WARNING: PyTorch is not using the GPU!\nThis will absurdly slow down the code if on Jetson!")
#    else:
#        devname = torch.cuda.get_device_name(0)
#        print(f"INFO: Using {devname} device as GPU.")
    
    init()
    live_predictions()

if __name__ == '__main__':
    main()
