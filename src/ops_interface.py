#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, Int32

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

import numpy as np
import cv2

toggleLasers = False
fishLengthProgram = False
shipwreckLengthProgram = False
photomosaicProgram = False
old_msg = False
old_msg_2 = False

def fishLengthCallback(msg, cb_args=0):
    window_name = "image"
    def draw_circle(event,x,y,flags,param,msg,cb_args=0):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            img = cv2.circle(img,(x,y),20,(0,255,0),-1)
    
    clicks = []
    vid_capture = cv2.VideoCapture('https://www.pexels.com/video/7222009/download/')
    
    count = 0
    
    while (vid_capture.isOpened()):
        ret, frame = vid_capture.read()
        cv2.imshow('video', frame)
        k = cv2.waitKey(1)
        
        if count < 1:
            if k == ord('p'):
                cv2.imwrite('home/jhsrobo/Pictures/fishLength.png', frame)
                count = count + 1
                break
        
    if count >= 1:
        img = cv2.imread('home/jhsrobo/Pictures/fishLength.png', cv2.IMREAD_COLOR)
        window_open = False
        while not window_open:
            try:
                cv2.namedWindow(window_name)
                cv2.setMouseCallback(window_name, draw_circle)
                window_open = True
            except:
                cv2.destroyAllWindows()
        status = True
        while status:
            cv2.imshow(window_name, img)
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
    window_name = "image"
    def draw_circle(event,x,y,flags,param,msg,cb_args=0):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            img = cv2.circle(img,(x,y),20,(0,255,0),-1)
    
    clicks = []
    vid_capture = cv2.VideoCapture('https://www.pexels.com/video/7222009/download/')
    
    count = 0
    
    while (vid_capture.isOpened()):
        ret, frame = vid_capture.read()
        cv2.imshow('video', frame)
        k = cv2.waitKey(1)
        
        if count < 1:
            if k == ord('p'):
                cv2.imwrite('home/jhsrobo/Pictures/shipwreckLength.png', frame)
                count = count + 1
                break
        
    if count >= 1:
        img = cv2.imread('home/jhsrobo/Pictures/shipwreckLength.png', cv2.IMREAD_COLOR)
        window_open = False
        while not window_open:
            try:
                cv2.namedWindow(window_name)
                cv2.setMouseCallback(window_name, draw_circle)
                window_open = True
            except:
                cv2.destroyAllWindows()
        status = True
        while status:
            cv2.imshow(window_name, img)
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

def enable_front_lasers(msg, cb_args=0):
    global gpio_pub, old_msg
    if old_msg != msg.data:
        gpio_pub.publish(21)
        gpio_pub.publsih(20)
        old_msg = msg.data

def enable_bottom_lasers(msg, cb_args=0):
    global gpio_pub, old_msg_2
    if old_msg_2 != msg.data:
        gpio_pub.publish(26)
        gpio_pub.publish(6)
        old_msg_2 = msg.data
        
def main():
    global pubFishLength, pubShipwreckLength, pubFrontLasers, pubBottomLasers, pubPhotomosaic, pubShipwreck, subFishLength, subShipwreckLength, subFrontLasers, subBottomLasers, subPhotomosaic, subShipwreck, gpio_pub
    rospy.init_node('ops_interface')
    
    # Subscribers
    subFishLength = rospy.Subscriber('ops/fish_toggle', Bool, fishLengthCallback)
    subShipwreckLength = rospy.Subscriber('ops/shipwreck_toggle', Bool, shipwreckLengthCallback)
    subFrontLasers = rospy.Subscriber('ops/toggle_front_lasers', Bool, enable_front_lasers)
    subBottomLasers = rospy.Subscriber('ops/toggle_bottom_lasers', Bool, enable_bottom_lasers)
    
    # Publishers
    pubFrontLasers = rospy.Publisher('ops/toggle_front_lasers', Bool, queue_size=1)
    pubBottomLasers = rospy.Publisher('ops/toggle_bottom_lasers', Bool, queue_size=1)
    pubFishLength = rospy.Publisher('ops/fish_toggle', Bool, queue_size=1)
    pubShipwreckLength = rospy.Publisher('ops/shipwreck_toggle', Bool, queue_size=1)
    pubPhotomosaic = rospy.Publisher('ops/photomosaic_toggle', Bool, queue_size=1)
    pubShipwreck = rospy.Publisher('ops/shipwreck_mapper', Bool, queue_size=1)
    
    gpio_pub = rospy.Publisher('rov/gpio_control', Int32, queue_size=1)
    
    def opsCallback(config, level):
      global fishLengthProgram, shipwreckLengthProgram, toggle_front_lasers, toggle_bottom_lasers, photomosaicProgram, mapShipwreck
      
      toggle_front_lasers = config.toggle_front_lasers
      toggle_bottom_lasers = config.toggle_bottom_lasers
      fishLengthProgram = config.fish_length_finder
      shipwreckLengthProgram = config.shipwreck_length_finder
      photomosaicProgram = config.photomosaic
      mapShipwreck = config.shipwreck_mapper
      
      
      pubFrontLasers.publish(toggle_front_lasers)
      pubBottomLasers.publish(toggle_bottom_lasers)
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
