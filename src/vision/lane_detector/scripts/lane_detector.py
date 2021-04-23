#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

def callback_rgb_raw(msg):
    bridge = cv_bridge.CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    edges = cv2.Canny(img_bgr,100,200)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 150, None, 0, 0)
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(img_bgr, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    cv2.imshow("Image BGR", img_bgr)
    cv2.imshow("Cany Borders", edges)
    cv2.waitKey(1)

def callback_laser_scan(msg):
    print("Laser scan received with " + str(len(msg.ranges)) + " received")

def main():
    print("INITIALIZING SIMPLE CONTROL BY MARCOSOFT...")
    rospy.init_node("lane_finder")
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_rgb_raw)
    #rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
