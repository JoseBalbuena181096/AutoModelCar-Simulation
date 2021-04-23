#!/usr/bin/env python
import rospy
import cv2 
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

def region_of_interest(img_edges):
    height = img_edges.shape[0]
    width = img_edges.shape[1]
    mask = np.zeros_like(img_edges)
    triangle = np.array([[
        (-850, height),     #first point, bottom left 
        (320, 200),         #middle point, intermediate part
        (900, height)       #last point, bottom right
    ]], np.int32)
    cv2.fillPoly(mask, triangle, 255)               #mask overything, exept the portion defined with the triangle
    img_masked = cv2.bitwise_and(img_edges, mask)   #removes masked region and keeps internal region of the triangel
    return img_masked

#get lines
def hough_lines(img_cropped):
    #probability of where the lines are
    lines = cv2.HoughLinesP(
    img_cropped,
    rho=1.0,
    theta=np.pi/180, 
    threshold=60,       
    lines=None, 
    minLineLength=175, #The bigger the number there will be less lines
    maxLineGap=150)    #The bigger the number there will be longer straight lines
                       #The smaller the number there will be shorter straigth lines 
    return lines

def display_lines(img_bgr, average_lines):
    #draw lines
     #number "3" represent the 3 rgb chanels
    img_lines = np.zeros((img_bgr.shape[0], img_bgr.shape[1],3),dtype=np.uint8)
    line_color = [255, 0, 0]    #line color wil be blue
    line_thickness = 4
    dot_color = [0, 0, 255]     #dot color will be red
    dot_size = 5 

    for line in average_lines:
        for  x1, y1, x2, y2 in line:
            cv2.line(img_lines, (x1, y1), (x2, y2), line_color, line_thickness)
            cv2.circle(img_lines, (x1, y1), dot_size, dot_color, -1)
            cv2.circle(img_lines, (x2, y2), dot_size, dot_color, -1)
    
    return img_lines


def add_weighted(img_bgr, img_lines):
    #make lines more visible
    return cv2.addWeighted(src1=img_bgr, alpha=0.8, src2=img_lines, beta=1.0, gamma=0.0)

def callback_image(msg):
    bridge = CvBridge()

    img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")

                        #[HUE, SATURATION, VALUE]
    lower_white = np.array([177,177,177])
    upper_white = np.array([255,255,255])
    img_bin = cv2.inRange(img_bgr, lower_white, upper_white)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    img_noise_filter = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel) #noise filter

    #detect edges
    img_edges = cv2.Canny(img_noise_filter, 100, 200)     #image with edges
    img_cropped = region_of_interest(img_edges)           #image of the region of interest
    
    lines = hough_lines(img_cropped)
    
    img_lines = display_lines(img_bgr, lines)
    img_overlayed = add_weighted(img_bgr, img_lines)      #original image "img_bgr" with lane lines drawn

    """ cv2.imshow("BRG image", img_bgr)
    cv2.imshow("Binary image", img_bin)
    cv2.imshow("Filtered image", img_noise_filter) """
    """ cv2.imshow("Edges image", img_edges) """
    """ cv2.imshow("Cropped image", img_cropped) """
    cv2.imshow("Combo image", img_overlayed)
    cv2.waitKey(33)

def main():
    print("Lane detector with openCV node")
    rospy.init_node("lane_detector_node")
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_image)
    loop = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
