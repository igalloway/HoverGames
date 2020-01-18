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

### Functions for hotspot detector.
import cv2
import numpy as np;
import os #added for find thermal
import v4l2capture #added for find thermal
from helper_func import hot_spot # added for analyze hot spots
from pymavlink import mavutil
import csv
import time

def read_thermal_camera(image_frame):
    
    #***next lines are copies from thermal open CV example***
    #ret, frame = cap.read()
    gray = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
    #*************
    gray = cv2.resize(gray, (800,600),interpolation = cv2.INTER_CUBIC)
    thermal_pass_through = gray
    img_avg = np.average(gray) / 2
    #gray = cv2.subtract(gray, np.array([img_avg]))

    image = gray
    image_2 = gray
    return image, image_2, thermal_pass_through


#!/usr/bin/python
#
# python-v4l2capture
#
# 2009, 2010 Fredrik Portstrom
#
# I, the copyright holder of this file, hereby release it into the
# public domain. This applies worldwide. In case this is not legally
# possible: I grant anyone the right to use this work for any
# purpose, without any conditions, unless such conditions are
# required by law.

def find_thermal_src():
    file_names = [x for x in os.listdir("/dev") if x.startswith("video")]
    file_names.sort()
    for file_name in file_names:
        path = "/dev/" + file_name
        print path
        try:
            video = v4l2capture.Video_device(path)
            driver, card, bus_info, capabilities = video.get_info()
            print "    driver:       %s\n    card:         %s" \
                "\n    bus info:     %s\n    capabilities: %s" % (
                    driver, card, bus_info, ", ".join(capabilities))

            if "PureThermal" == card[0:11]:
                print(card[0:11])
            
            if "PureThermal" in card and "readwrite" in capabilities and "streaming" in capabilities and "video_capture" in capabilities:
                print("Thermal camera is at: ", file_name[5:])
                thermal_addr = int(file_name[5:])
                print(thermal_addr)
                video.close()
                return thermal_addr
                break
            video.close()
        except IOError, e:
            print "    " + str(e)
        
    return thermal_addr
        

def analyze_hotspots(keypoints, point_list, wpos, datetime, lat, lon, encroach, save_folder, rel_alt):#,folder_loc):
    if len(keypoints) > 0:
        cur_time = datetime.datetime.now()

        print(encroach)
        print('encroach at: ', lat,', ', lon)


        for i in range(0, len(keypoints)):
            #print("x= ",keypoints[i].pt[0]," y= ",keypoints[i].pt[1], "Diameter= ", keypoints[i].size, "\n")
            print(i)
            #patrol pat shall be routed to keep the cold side on the left and the hot side on the right.
            #hot spots on the right are allowed.
            #hot spots on the left are flagged as encroachments.
            if (keypoints[i].pt[0] < wpos): # if hotspot is to the left
                encroach = encroach + 1
            ##add lockout for encroachments in previous field of view. or do we let multiple hits indicate spark longevity.
                point_list.append( hot_spot(cur_time, lat, lon, 1, keypoints[i].size, False))##
                most_rec_encroach = point_list[len(point_list)-1]
            #def __init__(self, time, lat, lon, en_type, size, image_taken):
            
        if len(point_list) > 10:
            print('file updated')
            filename = save_folder + "points_" + str(cur_time.year) + "_" + str(cur_time.month) + "_" + str(cur_time.day) + "_" + str(cur_time.hour) + "00.csv"
            fieldnames = ['lat', 'lon', 'size', 'date_time','en_type','image_taken', 'rel_alt']
            try:
                with open(filename) as csvfile: #check to see if file for this hour exists
                    csvfile.close()
                with open(filename, 'a') as csvfile:# If it does exist append encroachments
                    
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                    
                    for i in range(len(point_list)):
                        att = point_list.pop(0)
                        writer.writerow({'lat': att.lat, 'lon': att.lon, 'size': att.size,
                        'date_time': att.time, 'en_type': att.en_type, 'image_taken': att.image_taken, 'rel_alt':rel_alt})
                        #print(att.time.year, att.size)
                        #hot spot is: def __init__(self, time, lat, lon, en_type, size, image_taken):
                        if len(point_list) < 1:
                            most_rec_encroach = att
                            print(att.time)
            except:
                with open(filename, 'w') as csvfile: # file didn't exist, so start a new one with a header
                    
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                    writer.writeheader()
                    #writer.writerow({'lat': 46.2, 'lon': 117, 'seq':3})
                    for i in range(len(point_list)):
                        att = point_list.pop(0)
                        writer.writerow({'lat': att.lat, 'lon': att.lon, 'size': att.size,
                        'date_time': att.time, 'en_type': att.en_type, 'image_taken': att.image_taken, 'rel_alt': rel_alt})
                        #print(att.time.year, att.size)
                        #hot spot is: def __init__(self, time, lat, lon, en_type, size, image_taken):
                        if len(point_list) < 1:
                            most_rec_encroach = att
                            print(att.time)
        
    return encroach

def annotate_image(image_in, keypoints, wpos, hdg, targ_ang, font):
        
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    image_with_keypoints = cv2.drawKeypoints(image_in, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.drawKeypoints
    h, w, cha = image_with_keypoints.shape
    
    disp_ang = str(hdg)
    ang_dif = targ_ang - hdg #target - heading = ang_dif
    #fov = 45 deg
    pix_per_deg = w/45 #width divided by FOV
    wpos = w/2 #
    wpos = int(wpos)
    
    #cv2.line(image, start, end, color bgr, width)
    cv2.line(image_with_keypoints,(wpos,0),(wpos,h),(255,255,255),3)
    
    cv2.putText(image_with_keypoints, disp_ang,(wpos,30),font,1,(0,255,0),2,cv2.LINE_AA)
    return image_with_keypoints, wpos


def save_image(src_image, postfix, cur_time, save_folder):

    #print(cur_time)
    #filename = prefix + str(cur_time.minute) + "_"+ str(cur_time.second) + ".jpg"
    filename = str(cur_time.year) + "-" + str(cur_time.month) + "-" + str(cur_time.day) + "-" + str(cur_time.hour) + "-" + str(cur_time.minute) + "-"+ str(cur_time.second) + "_" + postfix + ".jpg"
    filename = save_folder + filename
    #print(filename)
    # Using cv2.imwrite() method 
    # Saving the image 
    cv2.imwrite(filename, src_image) 
    captured = True
        
    



def value_min_max (input_val, lower, upper):
    #print ('input_raw', input_val)
    input_val = max(input_val, lower) # ensure input is atleast as large as lower
    #print('in after max', input_val)
    input_val = min(input_val, upper) # ensure input is no larger than upper
    #print ('in after min', input_val)
    #seconds = time.time()
    #print("Seconds since epoch =", seconds)
    return input_val
    

def update_camera(yaw_input, pitch_input, yaw_current, yaw_skip, pitch_current, pitch_skip, scale):
    center = 1514 # 1.515mS
    deadband = 50
    #print('CH09: ', yaw_input)
    #print('CH10: ', pitch_input)
    #coherce inputs.
    if yaw_input < 100:
        yaw_input = center
    if pitch_input < 100:
        pitch_input = center
    yaw_input = value_min_max(yaw_input, 1033, 2000) # make sure input is atleast 1033 #1033 to 2000, center is 1514
    pitch_input = value_min_max(pitch_input, 1033, 2000)
    #apply if outside deadband.
    #print ("yaw_input: ", yaw_input)
    
    if abs(yaw_input - center) > deadband and (yaw_skip - time.time()) < -0.10 :
        yaw_skip = time.time()
        #print('yaw delta' , yaw_input-center)
        yaw_current = yaw_current - (yaw_input - center) * scale
        yaw_current = value_min_max(yaw_current, -180.0, 180.0)
        #print('yaw current: ', yaw_current)
    
    if abs(pitch_input - center) > deadband and (pitch_skip - time.time()) < -0.10:
        pitch_skip = time.time()
        pitch_current = pitch_current + (pitch_input - center) * scale
        pitch_current = value_min_max(pitch_current, -180.0, 180.0)
        #print('pitch_current: ', pitch_current)
        
        
    return yaw_current, yaw_skip, pitch_current, pitch_skip
