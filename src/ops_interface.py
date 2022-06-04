#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

import numpy as np
from cv2 import cv2

toggleLasers = False
lengthProgram = False
photomosaicProgram = False

def lengthCallback(msg, cb_args):
    class Mouse:
        def __init__(self):
            self.clicks = []
            self.img = cv2.imread('https://www.thesprucepets.com/thmb/wNqkG8YzvRd22RBSk6lu4h5QZVo=/941x0/filters:no_upscale():max_bytes(150000):strip_icc():format(webp)/low-maintenance-freshwater-fish-4770223-hero-ffb66c229c194e2db4916e88bbd17a15.jpg', cv2.IMREAD_COLOR)
            cv2.namedWindow('image')

        def draw_circle(self, event,x,y,flags,param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.clicks.append((x, y))
                print('x = %d, y = %d'%(x, y))
                self.img = cv2.circle(self.img,(x,y),20,(0,255,0),-1)
            

    def fish(coords):
        print(coords)
        reference = abs(coords[0][0] - coords[1][0])
        ratio = 5 / reference
        total = ((((abs(coords[2][0] - coords[3][0]) ** 2) + (abs(coords[2][1] - coords[3][1])) ** 2)) ** 0.5) * ratio
        print("Length of Fish: {:.2f} centimeters".format(total))


    def shipwreck(coords):
        reference = abs(coords[0][0] - coords[1][0])
        ratio = 15 / reference
        total = ((((abs(coords[2][0] - coords[3][0]) ** 2) + (abs(coords[2][1] - coords[3][1])) ** 2)) ** 0.5) * ratio
        print("Length of Shipwreck: {:.2f} centimeters".format(total))

    def lengthMain():
        c = Mouse()
        cv2.setMouseCallback('image',c.draw_circle)

        selection = input("s = shipwreck | f = fish:\n")

        status = True
        while status:
            cv2.imshow("image", c.img)
            cv2.waitKey(1)
            if len(c.clicks) == 4:
                if selection == 's':
                    shipwreck(c.clicks)
                    status = False
                elif selection == 'f':
                    fish(c.clicks)
                    status = False
                else:
                    print("Wrong input, try again.")
    lengthMain()

def main():
    global pubLength, pubLasers, pubPhotomosaic, pubShipwreck, subLength, subLasers, subPhotomosaic, subShipwreck
    rospy.init_node('ops_interface')
    
    # Subscribers
    subLength = rospy.Subscriber('ops/measure_toggle', Bool, lengthCallback)
    
    # Publishers
    pubLasers = rospy.Publisher('ops/toggle_lasers', Bool, queue_size=1)
    pubLength = rospy.Publisher('ops/measure_toggle', Bool, queue_size=1)
    pubPhotomosaic = rospy.Publisher('ops/photomosaic_toggle', Bool, queue_size=1)
    pubShipwreck = rospy.Publisher('ops/shipwreck_mapper', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global lengthProgram, toggleLasers, photomosaicProgram, mapShipwreck
      
      toggleLasers = config.toggle_lasers
      lengthProgram = config.length_finder
      photomosaicProgram = config.photomosaic
      mapShipwreck = config.shipwreck_mapper
      
      
      pubLasers.publish(toggleLasers)
      pubLength.publish(lengthProgram)
      pubPhotomosaic.publish(photomosaicProgram)
      pubShipwreck.publish(mapShipwreck)
      
      config.length_finder = False
      config.photomosaic = False
      config.shipwreck_mapper = False
    
      return config
      

    # setup dynamic reconfigure
    server = Server(opsControlParamsConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()
