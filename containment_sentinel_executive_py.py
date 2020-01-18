#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  t
#  
#  Copyright 2020  Martin Lee 
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  
#  
#picam has a 45 deg FOV
import sys
from pymavlink import mavutil
from helper_func import hot_spot
from helper_func import get_distance_metres
import csv
from storm_ctrl import setAngle
import serial
#import storm_ctrl

  # import the necessary packages
from picamera import PiCamera
import picamera
import picamera.array


import time
import cv2
import numpy as np;
import threading

from PIL import Image
import datetime
from Function_Lib_2 import read_thermal_camera
from Function_Lib_2 import find_thermal_src
from Function_Lib_2 import analyze_hotspots
from Function_Lib_2 import annotate_image
from Function_Lib_2 import save_image
from Function_Lib_2 import value_min_max
from Function_Lib_2 import update_camera
font = cv2.FONT_HERSHEY_SIMPLEX

lat = 0.0
lon = 0.0
hdg = 0.0
rel_alt = 0.0
targ_ang = 245.0 #90.0
encroach = 0
gps_time = 0
CH09 = 0.0 # left shoulder wheel
CH10 = 0.0 # right shoulder wheel
gimbal_pitch = 45.0
gimbal_roll = 0.0
gimbal_yaw = 0.0
yaw_skip = 0.0
pitch_skip = 0.0
rel_alt = 0

point_list = [] #def __init__(self, time, lat, lon, en_type, size, image_taken):

cur_time = datetime.datetime.now()
most_rec_encroach = hot_spot(cur_time, lat, lon, 1, 2, False)
point_list.append( hot_spot(cur_time, lat, lon, 1, 2, False))

print(len(point_list))
print(point_list.pop().time)
print(len(point_list))
    


def mavlink_listen ():
    global lat
    global lon
    global hdg
    global rel_alt
    global CH09
    global CH10
    global gimbal_pitch
    global gimbal_roll
    global gimbal_yaw
    scale = 0.005
    prev_pitch = 45.0
    prev_roll = 0.0
    prev_yaw = 0.0
    yaw_skip = time.time()
    pitch_skip = time.time()
    #time.sleet(2.0)
    master = mavutil.mavlink_connection(
        '/dev/ttyAMA0', 
        baud=57600)
        
        
        
    # serial to STorM32
    # use: 'dmesg | grep tty' to view avalible serial ports
    ser = serial.Serial(
       port='/dev/ttyACM0', # was ttyS0
       baudrate = 115200,
       parity=serial.PARITY_NONE,
       stopbits=serial.STOPBITS_ONE,
       bytesize=serial.EIGHTBITS,
       timeout=1
    )
    #lat = 0.0
    #lon = 0.0
    time.sleep(0.5) #0.5
    setAngle(45.0, 0.0, 0.0, ser) #pitch, roll, yaw, serial channel
    
    try:
        #master.mav.MAV_CMD_SET_MESSAGE_INTERVAL(2,0,0) # display time
        #master.mav.MAV_CMD_SET_MESSAGE_INTERVAL(340,0,0)# Time command
        print ('time command success')
    except:
        print('MAV_CMD fail')
    
    while True:
        
        msg = master.recv_match()
        #lat, lon, hdg, rel_alt, CH09, CH10 = receive_MavLink(msg, lat, lon, hdg, rel_alt, CH09, CH10)
        
         
        if not msg:
        #try:
            #master.mav.MAV_CMD_SET_MESSAGE_INTERVAL(340,0,0)
        #except:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT': #33
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            #print("\nAs dictionary: %s" % msg.to_dict())
            # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
            #print("\nSystem status: %s" % msg.system_status)
            #print(msg.lat)
            #print(msg)
            lat = msg.lat / 1e7 #10000000.0
            #print(lat)
            lon = msg.lon / 1e7 #10000000.0
            #print(lon)
            hdg = msg.hdg /100.0
            #print(hdg)
            rel_alt = msg.relative_alt # in millimeters above ground or home location
            #print(rel_alt)
            
        elif msg.get_type() == 'RC_CHANNELS': ##65
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            #print("\nAs dictionary: %s" % msg.to_dict())
            CH09 = float(msg.chan9_raw)
            #print('CH09', CH09) #1033 to 2000, center is 1514 Lower value is CCW
            CH10 = float(msg.chan10_raw)
            #print('CH10: ', CH10) #1033 to 2000, center is 1514 Lower value is CW
            
        elif msg.get_type() == 'SYSTEM_TIME': #2
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            got_time = True
        elif msg.get_type() == 'UTM_GLOBAL_POSITION': #340 #UTM_GLOBAL_POSITION
            #pass
            print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
        
    
        gimbal_yaw, yaw_skip, gimbal_pitch, pitch_skip = update_camera(CH09, CH10, gimbal_yaw, yaw_skip, gimbal_pitch, pitch_skip, scale)
        
        if gimbal_pitch != prev_pitch or gimbal_yaw != prev_yaw:
            prev_pitch = gimbal_pitch
            prev_yaw = gimbal_yaw
            setAngle(gimbal_pitch, 0.0, gimbal_yaw, ser) #pitch, roll, yaw, serial channel
        

        

    
    
        

def detect_blobs():
    global encroach
    wpos = 0
    captured = False
    global gimbal_pitch
    global gimbal_roll
    global gimbal_yaw
    global CH09
    global CH10
    global rel_alt
    save_folder = "/home/pi/fire_records/"
    scale = 0.1
    last_capture_time = datetime.datetime.now().second
    #color_image = (640, 480, 3)
    
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
 
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100
    
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 9000
    
    
    # Set up the detector with default parameters.
    #detector = cv2.SimpleBlobDetector(params)
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)
    
    
     
    # initialize the Pi camera and grab a reference to the raw camera capture
    camera = PiCamera() #PiCam is typically logical 0
    camera.exposure_mode = 'auto'
    #sleep(5)
    #camera.awb_mode = 'auto'
    camera.resolution = (640, 480)
    camera.framerate = 10 #32
    camera.start_preview()
    print(camera.exposure_speed)
    #camera.shutter_speed = 7000 #1000
    
    #color_image = 
    # allow the camera to warmup
    time.sleep(0.1)
    #img = cv2.imread('/home/pi/learn_camera/image_2.jpg',0)
    
    #find and configure the thermal array
    thermal_addr = find_thermal_src()
    print("find_thermal_scr returned: ", thermal_addr)
    cap = cv2.VideoCapture(thermal_addr) #1 is the logical number for first UVC source PureThermal2. The RPi camera is typically 0
    
    time.sleep(1.5)
   
    
    while True:
        
        camera.exposure_mode = 'auto'
        ret, frame = cap.read()
        image, image_2, thermal_pass_though = read_thermal_camera(frame)
        
        #thresh1 = image #= frame.array
        ret,thresh1 = cv2.threshold(image,220,255,cv2.THRESH_BINARY)

        #invert image
        thresh1 = cv2.bitwise_not(thresh1)
        
        # Detect blobs.
        keypoints = detector.detect(thresh1)
        #print keypoints
        #print len(keypoints)
        
        #log encroacments if they are in the correct spot
        encroach = analyze_hotspots(keypoints, point_list, wpos, datetime, lat, lon, encroach, save_folder+"img/", rel_alt)
        
        image_with_keypoints, wpos = annotate_image(image_2, keypoints, wpos, hdg, targ_ang, font)
        
        if encroach > 0:
            encroach = 0
            #store encroachment image
            save_image(image_with_keypoints,"infrared", datetime.datetime.now(),save_folder+"img/")
            save_image(thermal_pass_though,"pass_through",datetime.datetime.now(),save_folder+"img/")
        
            #print("datetime seconds delta",datetime.datetime.now().second - last_capture_time)
            if abs(datetime.datetime.now().second - last_capture_time) > 10:
                last_capture_time = datetime.datetime.now().second
                
                cv2.namedWindow('frame_color', cv2.WINDOW_NORMAL)
                with picamera.array.PiRGBArray(camera) as stream:
                    camera.capture(stream, format='bgr')
                    # At this point the image is available as stream.array
                    image_color = stream.array
                    cv2.resizeWindow('frame_color', 640,480)
                    cv2.imshow('frame_color', image_color)
                save_image(image_color,"Area_image", datetime.datetime.now(),save_folder+"img/")
                
        # show the frame
        
        cv2.imshow("Frame2", image_with_keypoints)
        cv2.imshow("Frame", thresh1)
        key = cv2.waitKey(1) & 0xFF
     
     
        #if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
    
                

t1 = threading.Thread(target=mavlink_listen, args=())
t2 = threading.Thread(target=detect_blobs, args=())


t1.start()
t2.start()
