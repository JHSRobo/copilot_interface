#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

import numpy as np
from cv2 import cv2

toggleLasers = False
fishLengthProgram = False
shipwreckLengthProgram = False
photomosaicProgram = False

def fishLengthCallback(msg, cb_args=0):
    def draw_circle(img,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            img = cv2.circle(img,(x,y),20,(0,255,0),-1)
    
    clicks = []
    vid_capture = cv2.VideoCapture("https://www.pexels.com/video/7222009/download/")
    
    cv2.setMouseCallback('image',draw_circle)
    count = 0
    
    while (vid_capture.isOpened()):
        ret, frame = c.vid_capture.read()
        cv2.imshow('video', frame)
        k = cv2.waitKey(1)
        
        if count < 1:
            if k == ord('p'):
                cv2.imwrite('home/jhsrobo/Pictures/fishLength.png', frame)
                count = count + 1
                break
        
    if count >= 1:
        img = cv2.imread("home/jhsrobo/Pictures/fishLength.png", cv2.IMREAD_COLOR)
        status = True
        while status:
            cv2.imshow("image", img)
            cv2.waitKey(1)
            if len(clicks) == 4:
                reference = abs(coords[0][0] - coords[1][0])
                ratio = 5 / reference
                total = ((((abs(coords[2][0] - coords[3][0]) ** 2) + (abs(coords[2][1] - coords[3][1])) ** 2)) ** 0.5) * ratio
                status = False
        displayImg = np.zeros((512,512,3), dtype=np.uint8)
        cv2.putText(displayImg, "{:.2f} cm".format(total), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 5, (50, 255, 50), 3)
        
        cv2.imshow('displayImage', displayImg)
        cv2.waitKey(0)

def shipwreckLengthCallback(msg, cb_args=0):
    def draw_circle(img,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            img = cv2.circle(img,(x,y),20,(0,255,0),-1)
    
    clicks = []
    vid_capture = cv2.VideoCapture("https://www.pexels.com/video/7222009/download/")
    
    cv2.setMouseCallback('image',draw_circle)
    count = 0
    
    while (vid_capture.isOpened()):
        ret, frame = c.vid_capture.read()
        cv2.imshow('video', frame)
        k = cv2.waitKey(1)
        
        if count < 1:
            if k == ord('p'):
                cv2.imwrite('home/jhsrobo/Pictures/shipwreckLength.png', frame)
                count = count + 1
                break
        
    if count >= 1:
        img = cv2.imread("home/jhsrobo/Pictures/shipwreckLength.png", cv2.IMREAD_COLOR)
        status = True
        while status:
            cv2.imshow("image", img)
            cv2.waitKey(1)
            if len(clicks) == 4:
                reference = abs(coords[0][0] - coords[1][0])
                ratio = 15 / reference
                total = ((((abs(coords[2][0] - coords[3][0]) ** 2) + (abs(coords[2][1] - coords[3][1])) ** 2)) ** 0.5) * ratio
                status = False
        displayImg = np.zeros((512,512,3), dtype=np.uint8)
        cv2.putText(displayImg, "{:.2f} cm".format(total), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 5, (50, 255, 50), 3)
        
        cv2.imshow('displayImage', displayImg)
        cv2.waitKey(0)

def main():
    global pubFishLength, pubShipwreckLength, pubLasers, pubPhotomosaic, pubShipwreck, subFishLength, subShipwreckLength, subLasers, subPhotomosaic, subShipwreck
    rospy.init_node('ops_interface')
    
    # Subscribers
    subFishLength = rospy.Subscriber('ops/fish_toggle', Bool, fishLengthCallback)
    subShipwreckLength = rospy.Subscriber('ops/shipwreck_toggle', Bool, shipwreckLengthCallback)
    
    # Publishers
    pubLasers = rospy.Publisher('ops/toggle_lasers', Bool, queue_size=1)
    pubFishLength = rospy.Publisher('ops/fish_toggle', Bool, queue_size=1)
    pubShipwreckLength = rospy.Publisher('ops/shipwreck_toggle', Bool, shipwreckLengthCallback)
    pubPhotomosaic = rospy.Publisher('ops/photomosaic_toggle', Bool, queue_size=1)
    pubShipwreck = rospy.Publisher('ops/shipwreck_mapper', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global fishLengthProgram, shipwreckLengthProgram, toggleLasers, photomosaicProgram, mapShipwreck
      
      toggleLasers = config.toggle_lasers
      fishLengthProgram = config.fish_length_finder
      shipwreckLengthProgram = config.shipwreck_length_finder
      photomosaicProgram = config.photomosaic
      mapShipwreck = config.shipwreck_mapper
      
      
      pubLasers.publish(toggleLasers)
      pubFishLength.publish(fishLengthProgram)
      pubShipwreckLength.publish(shipwreckLengthProgram)
      pubPhotomosaic.publish(photomosaicProgram)
      pubShipwreck.publish(mapShipwreck)
      
      config.fish_length_finder = False
      config.shipwreck_length_finder = False
      config.photomosaic = False
      config.shipwreck_mapper = False
    
      return config
      

    # setup dynamic reconfigure
    server = Server(opsControlParamsConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()
