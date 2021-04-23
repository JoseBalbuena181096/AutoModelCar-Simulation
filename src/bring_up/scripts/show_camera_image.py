#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

def callback_image(msg):
	bridge = CvBridge()
	img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")

	img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

	cv2.imshow("Raw Image", img_bgr)
	#cv2.imshow("BGR image", img_bgr)
	cv2.waitKey(33)

def main():
	print("show_camera_image_node")
	rospy.init_node("show_camera_image_node")
	rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_image)
	loop = rospy.Rate(30)

	while not rospy.is_shutdown():
		loop.sleep()
 
if __name__ == '__main__':
 	try:
 		main()
 	except rospy.ROSInterruptException:
 		pass