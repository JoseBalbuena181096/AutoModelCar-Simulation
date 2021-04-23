#!/usr/bin/env python
import cv2 
import math
import rospy
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Int16
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

    #when there is no longer lane "average_lines" is not iterable, so try-except is used 
    try:
        for line in average_lines:
            for  x1, y1, x2, y2 in line:
                cv2.line(img_lines, (x1, y1), (x2, y2), line_color, line_thickness)
                cv2.circle(img_lines, (x1, y1), dot_size, dot_color, -1)
                cv2.circle(img_lines, (x2, y2), dot_size, dot_color, -1)
        
        return img_lines
    except:
        print("THERE IS NO LONGER LANE")

def calculate_inclination_angle_list(lines):
    #sometimes len(lines) is equal to 1
    try:
        if len(lines) == 0:
            print("No lines found")
            pass
        else:
            slope_list = []
            for line in lines:
                x1 = line[0][0]
                y1 = line[0][1]
                x2 = line[0][2]
                y2 = line[0][3]
    
                #slope calculation
                slope = (float(y2)-float(y1)) / (float(x2)-float(x1))
                #angle calculation
                slope = (math.atan(slope)*180)/math.pi
                slope_list.append(slope)
        return slope_list
    except:
        pass

def add_weighted(img_bgr, img_lines):
    #make lines more visible
    try:
        return cv2.addWeighted(src1=img_bgr, alpha=0.8, src2=img_lines, beta=1.0, gamma=0.0)
    except:
        pass

def callback_image(msg):
    bridge = CvBridge()

    img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")

                        #[HUE, SATURATION, VALUE]
    lower_white = np.array([177,177,177])
    upper_white = np.array([255,255,255])

    #binary image 
    img_bin = cv2.inRange(img_bgr, lower_white, upper_white)

    #filtered image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    img_noise_filter = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel) #noise filter

    #detect edges
    img_edges = cv2.Canny(img_noise_filter, 100, 200)     #image with edges
    img_cropped = region_of_interest(img_edges)           #image of the region of interest
    
    #lines list
    lines = hough_lines(img_cropped)

    img_lines = display_lines(img_bgr, lines)
    img_overlayed = add_weighted(img_bgr, img_lines)      #original image "img_bgr" with lane lines drawn

    try:
        #cv2.imshow("Lines image", img_lines)
        cv2.imshow("Combo image", img_overlayed)
        cv2.imshow("Original Image", img_bgr)

    except:
        pass

    cv2.waitKey(33)

    #--------------------------------------------------------------------------------------------------------------------------------
    
    #car velocity has a range between -2000 to 2000 
    vel = Int16() 
    direction = Int16()

    pub_vel = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed", Int16, queue_size = 10)
    pub_direction = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size = 10)

    #angle in degrees with sign
    angle_list = calculate_inclination_angle_list(lines)

    if isinstance(angle_list, list) == True: #This is one way to know if "angle_list" is iterable or not 
   
        upper_angle_list = []

        for angle in angle_list:
            #If angle is less than 0, we are receiving the right lane lines 
            if angle < 0:
                upper_angle_list.append(angle)
                    
        upper_angle_list.sort()
        #upper_angle_list[0] is the largest slope in the list 
        if abs(upper_angle_list[0]) > 36.5:
            print("The car IS straight")
            vel.data = -500
            direction.data = 90
        else:
            print("The car is NOT straight")
            #Error calculation
            angle_error = 39 - abs(upper_angle_list[0])
            vel.data = -350
            direction.data = 90 - (int(angle_error)*90)/39

        pub_vel.publish(vel)
        pub_direction.publish(direction)
    else:
        pass
    
    #--------------------------------------------------------------------------------------------------------------------------------

def main():
    print("Lane follower node")
    rospy.init_node("Lane_follower_node")
    rospy.Subscriber("/app/camera/rgb/image_raw", Image, callback_image)
    loop = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    