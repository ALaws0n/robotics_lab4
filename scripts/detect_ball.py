#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#Image received flag
img_received = False
#Create blank image?
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")

#Define function for processing the color of the image?

def get_image(ros_img):
	global rgb_img
	global img_received
	# process and recieve the image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	img_received = True

def process_image(image):
	lower_yellow_rgb = np.array([100,100,0])
	upper_yellow_rgb = np.array([255,255,75])
	
	yellow_mask = cv2.inRange(image, lower_yellow_rgb, upper_yellow_rgb)
	
	return yellow_mask
	
if __name__ == '__main__':
	#
	rospy.init_node('detect_ball', anonymous = True)
	# Subscriber to camera data
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	# Publisher for the mono color image
	img_pub = rospy.Publisher("/ball_2D", Image, queue_size = 1)
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		
		if img_received:
			#make a call out to our image processing function
			#convert the image back to rosmsg and publish it
			new_image = process_image(rgb_img)
			img_msg = CvBridge().cv2_to_imgmsg(new_image, encoding="mono8")
			img_pub.publish(img_msg)
			
		rate.sleep()
