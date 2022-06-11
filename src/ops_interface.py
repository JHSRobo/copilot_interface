#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, Int32

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

import numpy as np
import cv2
import PIL
from PIL import Image

toggle_front_lasers = False
toggle_bottom_lasers = False
old_msg = False
old_msg_2 = False


def fishLengthCallback(msg,cb_args=0):
    if msg.data:
	print("Running fish")
	    window_name = 'image'
	    def draw_circle(event,x,y,flags,param,cb_args=0,img=cv2.imread('/home/jhsrobo/Pictures/fishLength.png', cv2.IMREAD_COLOR)):
		window_name = 'image'
		if event == cv2.EVENT_LBUTTONDOWN:
		    clicks.append((x, y))
		    img = cv2.circle(img,(x,y),20,(0,255,0),-1)
		    img = cv2.imread('/home/jhsrobo/Pictures/fishLength.png', cv2.IMREAD_COLOR)
		    cv2.imshow(window_name, img)
	    clicks = []
	    vid_capture = cv2.VideoCapture('192.168.1.98') # CHANGE DEPENDING ON IP OF CAMERAS

	    count = 0

	    while (vid_capture.isOpened()):
		ret, frame = vid_capture.read()
		cv2.imshow('video', frame)
		k = cv2.waitKey(1)

		if count < 1:
		    if k == ord('p'):
			cv2.imwrite('/home/jhsrobo/Pictures/fishLength.png', frame)
			count = count + 1
			break

	    if count >= 1:
		cv2.destroyAllWindows()
		img = cv2.imread('/home/jhsrobo/Pictures/fishLength.png', cv2.IMREAD_COLOR)
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
			reference = abs(clicks[0][0] - clicks[1][0])
			ratio = 5. / reference
			total = ((((abs(clicks[2][0] - clicks[3][0]) ** 2) + (abs(clicks[2][1] - clicks[3][1])) ** 2)) ** 0.5) * ratio
			status = False
		displayImg = np.zeros((512,1024,3), dtype=np.uint8)
		cv2.putText(displayImg, "{:.2f} cm".format(total), (50,325), cv2.FONT_HERSHEY_SIMPLEX, 6, (50, 255, 50), 3)
		cv2.destroyWindow(window_name)
		finalDisplay = True
		while finalDisplay:
		    cv2.imshow('displayImage', displayImg)
		    q = cv2.waitKey(1)
		    if q == ord('q'):
			cv2.destroyAllWindows()
			break
        
def shipwreckLengthCallback(msg, cb_args=0):
    if msg.data:
        window_name = 'image'
	def draw_circle(event,x,y,flags,param,cb_args=0,img=cv2.imread('/home/jhsrobo/Pictures/shipwreckLength.png', cv2.IMREAD_COLOR)):
	    window_name = 'image'
	    if event == cv2.EVENT_LBUTTONDOWN:
		clicks.append((x, y))
		img = cv2.circle(img,(x,y),20,(0,255,0),-1)
		img = cv2.imread('/home/jhsrobo/Pictures/shipwreckLength.png', cv2.IMREAD_COLOR)
		cv2.imshow(window_name, img)
	    clicks = []
	    vid_capture = cv2.VideoCapture('192.168.1.111') # CHANGE DEPENDING ON IP OF CAMERAS

	    count = 0

	    while (vid_capture.isOpened()):
		ret, frame = vid_capture.read()
		cv2.imshow('video', frame)
		k = cv2.waitKey(1)

		if count < 1:
		    if k == ord('p'):
			cv2.imwrite('/home/jhsrobo/Pictures/shipwreckLength.png', frame)
			count = count + 1
			break

	    if count >= 1:
		cv2.destroyAllWindows()
		img = cv2.imread('/home/jhsrobo/Pictures/shipwreckLength.png', cv2.IMREAD_COLOR)
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
			reference = abs(clicks[0][0] - clicks[1][0])
			ratio = 15. / reference
			total = ((((abs(clicks[2][0] - clicks[3][0]) ** 2) + (abs(clicks[2][1] - clicks[3][1])) ** 2)) ** 0.5) * ratio
			status = False
		displayImg = np.zeros((512,1024,3), dtype=np.uint8)
		cv2.putText(displayImg, "{:.2f} cm".format(total), (50,325), cv2.FONT_HERSHEY_SIMPLEX, 6, (50, 255, 50), 3)
		cv2.destroyWindow(window_name)
		finalDisplay = True
		while finalDisplay:
		    cv2.imshow('displayImage', displayImg)
		    q = cv2.waitKey(1)
		    if q == ord('q'):
			cv2.destroyAllWindows()
		        break

def photomosaicCallback(msg, cb_args=0):
    if msg.data:
	filePath = '/home/jhsrobo/Pictures/photomosaicImage'
	tile = 1
    	vid_capture = cv2.VideoCapture("192.168.1.111")
    
    	while (vid_capture.isOpened()):
            ret, frame = vid_capture.read()
	    cv2.imshow('image', frame)

            k = cv2.waitKey(1)
            if tile <= 8:
            	if k == ord('p'):
                    cv2.imwrite('/home/jhsrobo/Pictures/photomosaicImage{}.png'.format(tile), frame)
                    tile = tile + 1
        
            if tile > 8:
            	# Top layer
            	top_list_im = ['{}1.png'.format(filePath), '{}2.png'.format(filePath), '{}3.png'.format(filePath), '{}4.png'.format(filePath)]
            	top_imgs = [PIL.Image.open(i) for i in top_list_im]
            	min_shape = sorted([(np.sum(i.size), i.size) for i in top_imgs])[0][1]
            	top_img_layer = np.hstack((np.asarray(i.resize(min_shape)) for i in top_imgs))
            	top_imgs_layer = PIL.Image.fromarray(top_img_layer)
            	top_imgs_layer.save('/home/jhsrobo/Pictures/photomosaicTopLayer.png')

            	# Bottom layer
            	bottom_list_im = ['{}8.png'.format(filePath), '{}7.png'.format(filePath), '6.png'.format(filePath), '{}5.png'.format(filePath)]
            	bottom_imgs = [PIL.Image.open(i) for i in bottom_list_im]
            	min_shape = sorted( [(np.sum(i.size), i.size ) for i in bottom_imgs])[0][1]
            	bottom_img_layer = np.hstack((np.asarray(i.resize(min_shape)) for i in bottom_imgs))
            	bottom_imgs_layer = PIL.Image.fromarray(bottom_img_layer)
            	bottom_imgs_layer.save('/home/jhsrobo/Pictures/photomosaicBottomLayer.png')

            	# Combined top and bottom layer
            	combine_list_im = ['/home/jhsrobo/Pictures/photomosaicTopLayer.png', '/home/jhsrobo/Pictures/photomosaicBottomLayer.png']
            	combine_imgs = [PIL.Image.open(i) for i in combine_list_im]
            	min_shape = sorted([(np.sum(i.size), i.size) for i in combine_imgs])[0][1]
            	combine_img_layers = np.vstack((np.asarray(i.resize(min_shape)) for i in combine_imgs))
            	combined_image = PIL.Image.fromarray(combine_img_layers)
            	combined_image.save('/home/jhsrobo/Pictures/Photomosaic.png')

            	# Display photomosaic
            	photomosaic = cv2.imread("/home/jhsrobo/Pictures/Photomosaic.png", cv2.IMREAD_COLOR)
            	displayImage = True
		while displayImage:
		    cv2.imshow('image', photomosaic)
            	    q = cv2.waitKey(1)
		    if q == ord('q'):
            		cv2.destroyAllWindows()
			break

def enable_front_lasers(msg, cb_args=0):
    global gpio_pub, old_msg
    if old_msg != msg.data:
        gpio_pub.publish(21)
        gpio_pub.publish(20)
        old_msg = msg.data

def enable_bottom_lasers(msg, cb_args=0):
    global gpio_pub, old_msg_2
    if old_msg_2 != msg.data:
        gpio_pub.publish(26)
        gpio_pub.publish(6)
        old_msg_2 = msg.data
        
def main():
    global pubFishLength, pubShipwreckLength, pubFrontLasers, pubBottomLasers, pubPhotomosaic, pubShipwreck, subFishLength, subShipwreckLength, subFrontLasers, subBottomLasers, subPhotomosaic, subShipwreck, gpio_pub, subPhotomosaic
    rospy.init_node('ops_interface')
    
    # Subscribers
    subFishLength = rospy.Subscriber('ops/fish_toggle', Bool, fishLengthCallback)
    subShipwreckLength = rospy.Subscriber('ops/shipwreck_toggle', Bool, shipwreckLengthCallback)
    subFrontLasers = rospy.Subscriber('ops/toggle_front_lasers', Bool, enable_front_lasers)
    subBottomLasers = rospy.Subscriber('ops/toggle_bottom_lasers', Bool, enable_bottom_lasers)
    subPhotomosaic = rospy.Subscriber('ops/photomosaic_toggle', Bool, photomosaicCallback)
    
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
      
      
      pubFrontLasers.publish(toggle_front_lasers)
      pubBottomLasers.publish(toggle_bottom_lasers)
      pubFishLength.publish(config.fish_length_finder)
      pubShipwreckLength.publish(config.shipwreck_length_finder)
      pubPhotomosaic.publish(config.photomosaic)
      pubShipwreck.publish(config.shipwreck_mapper)
      
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
