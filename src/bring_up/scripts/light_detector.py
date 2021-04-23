#!/usr/bin/env python 

import numpy
import cv2
import rospy 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image

def callback_image(msg):
	bridge = CvBridge()
	#Original image 
	img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8") 

	lower_green = numpy.array([0,40,0])
	upper_green = numpy.array([35,250,35])

	#Binary image, black and white image
	img_bin = cv2.inRange(img_bgr, lower_green, upper_green)

	#Noise filter 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
	img_filtered = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)

	cv2MajorVersion = cv2.__version__.split(".")[0]
	
	if int(cv2MajorVersion) >= 4:
		ctrs, hier = cv2.findContours(img_filtered.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	else:
		im2, ctrs, hier = cv2.findContours(img_filtered.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# sort contours
	sorted_ctrs = sorted(ctrs, key=lambda ctr: cv2.boundingRect(ctr)[0])

	for i, ctr in enumerate(sorted_ctrs):
		# Get bounding box
		x, y, w, h = cv2.boundingRect(ctr)
		# Getting ROI
		roi = img_bgr[y:y + h, x:x + w]
		#Draw rectangles on the original image
		cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
		#Draw circles on the original image
		#cv2.circle(img_bgr, (x+w/2,y+h/2),20, (0,255,0),3)

	cv2.imshow("Original image", img_bgr)
	#cv2.imshow("Binary image", img_filtered)
	
	cv2.waitKey(33)

def main():
	rospy.init_node("practice09")
	rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_image)
	loop = rospy.Rate(30)
	while not rospy.is_shutdown():
		loop.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass