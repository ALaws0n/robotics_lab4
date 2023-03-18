#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#Image received flag
img_received = False
#Create blank image
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


#Function for receiving image
def get_image(ros_img):
	global rgb_img
	global img_received
	# process and recieve the RGB image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	img_received = True

def process_image(image):
	# Convert the RGB image to the HSV color space
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	# Define HSV bounds
	lower_yellow_hsv = np.array([22,1,1])
	upper_yellow_hsv = np.array([60,255,255])
	# Mask the image by our defined bounds
	yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
	# Return the masked image
	return yellow_mask
	
if __name__ == '__main__':
	# Initialize the ball detection node
	rospy.init_node('detect_ball', anonymous = True)
	# Subscribe to camera data
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	# Publisher for the mono color image
	img_pub = rospy.Publisher("/ball_2D", Image, queue_size = 1)
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		
		if img_received:
			#make a call out to our image processing function
			new_image = process_image(rgb_img)
			#convert the image back to rosmsg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(new_image, encoding="mono8")
			img_pub.publish(img_msg)
			
		rate.sleep()
