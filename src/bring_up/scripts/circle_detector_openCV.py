#!/usr/bin/env python
import rospy
import cv2 
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

def callback_image(msg):
	bridge = CvBridge()

	img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")

	lower_green = np.array([0,40,0])
	upper_green = np.array([35,250,35])

	#Binary image, black and white image
	img_bin = cv2.inRange(img_bgr, lower_green, upper_green)

	#Noise filter 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(4,4))
	img_filtered = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)
	img_filtered_color = cv2.bitwise_and(img_bgr,img_bgr,mask=img_bin)

	#################################################################################
	
	gray = cv2.cvtColor(img_filtered_color, cv2.COLOR_BGR2GRAY)

	gray = cv2.medianBlur(gray, 5)

	rows = gray.shape[0]

	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
							   param1=1, param2=20,
							   minRadius=6, maxRadius=20)

	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
			# circle center
			cv2.circle(img_bgr, center, 1, (0, 100, 100), 3)
			# circle outline
			radius = i[2]
			cv2.circle(img_bgr, center, radius, (255, 0, 255), 3)

	#cv2.imshow("Gray Image", res1)
	cv2.imshow("Original Image", img_bgr)
	cv2.waitKey(33)


def main():
	print("Circle detector with openCV node")
	rospy.init_node("circle_detector_node")
	rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_image)
	loop = rospy.Rate(30)
	
	while not rospy.is_shutdown():
		loop.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
