#!/usr/bin/env python

# Created By Dr. Paul Robinette
# HW 8 modified by John Lutz on 17 Dec 2019

# mini project 3
# should "Contains a node that:
# subscribes to "image_transformer_node/corrected_image/compressed"
# Publishes the yellow and a white hough transform images as completed in HW8 in their own topics (you define)"


import sys
import cv2
import numpy as np

import rospy
import roslib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


# Publishes the yellow and a white hough transform images in their own topics I define

# convert compressed image to cv2 compatible format
image_convert = CvBridge() # used in other DB image message processing


def get_lines(original_image, filtered_image):
    # do our hough transform on the white image
    # resolution: 1 pixel radius, 1 degree rotational
    r_res = 1
    theta_res = np.pi/180
    # threshold: number of intersections to define a line
    thresh = 10
    # min_length: minimum number of points to form a line
    min_length = 10
    # max_gap: maximum gap between two points to be considered a line
    max_gap = 12
    lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)
    
    output = np.copy(original_image)
    if lines is not None:
        # grab the first line
        for i in range(len(lines)):
            print(lines[i])
            l = lines[i][0]
            cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
    return output


def lane_filter(image, white_output, yellow_output): # image is cv2_image
    # The incoming image is BGR format, convert it to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # OpenCV uses H: 0-179, S: 0-255, V: 0-255

    # Filter for only white pixels. Experiment with values as needed
    white_filter = cv2.inRange(hsv, (0,50,0), (255,180,255))
    # cv2.imshow("White Filter", white_filter) # dont need to see these for this
    # cv2.imwrite("white_filter.png", white_filter) # dont need to see these for this
    
    # Filter for only yellow pixels. Experiment with values as needed
    yellow_filter = cv2.inRange(hsv, (0,50,0), (140,255,255))
    # cv2.imshow("Yellow Filter", yellow_filter) # dont need to see these for this
    # cv2.imwrite("yellow_filter.png", yellow_filter) # dont need to see these for this
    
    # Create a kernel to dilate the image. 
    # Experiment with the numbers in parentheses (optional)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    
    # Dilate both the white and yellow images. 
    # No need to experiment here.
    white_dilate = cv2.dilate(white_filter, kernel)
    # cv2.imshow("Dilate White", white_dilate) # dont need to see these for this
    # cv2.imwrite("white_dilate.png", white_dilate) # dont need to see these for this
    yellow_dilate = cv2.dilate(yellow_filter, kernel)
    # cv2.imshow("Dilate Yellow", yellow_dilate) # dont need to see these for this
    # cv2.imwrite("yellow_dilate.png", yellow_dilate) # dont need to see these for this
    
    # Perform edge detection on the original image. 
    # Experiment with the first two numbers. Aperture size experimentation optional
    edges = cv2.Canny(image, 0, 300, apertureSize=3)
    # cv2.imshow("Edges", edges) # dont need to see these for this
    # cv2.imwrite("edges.png", edges) # dont need to see these for this
    
    # Use the edges to refine the lines in both white and yellow images
    # No need to experiment here 
    white_edges = cv2.bitwise_and(white_dilate, edges)
    # cv2.imshow("White Edges", white_edges) # dont need to see these for this
    # cv2.imwrite("white_edges.png", white_edges) # dont need to see these for this
    yellow_edges = cv2.bitwise_and(yellow_dilate, edges)
    # cv2.imshow("Yellow Edges", yellow_edges) # dont need to see these for this
    # cv2.imwrite("yellow_edges.png", yellow_edges) # dont need to see these for this
    
    # these need to get published
    white_output = get_lines(image, white_edges)
    yellow_output = get_lines(image, yellow_edges)
    
    cv2.imshow("White Output", white_output) # dont need to see these for this
    cv2.imwrite("white_output.png", white_output) # dont need to see these for this
    cv2.imshow("Yellow Output", yellow_output) # dont need to see these for this
    cv2.imwrite("yellow_output.png", yellow_output) # dont need to see these for this
    
    # Wait for key press to close images
    cv2.waitKey() # didnt open the images


#callback - convert and publish
def image_rx_callback(msg_compImg): # take in compressed image message from the subscriber

    cv2_image = image_convert.compressed_imgmsg_to_cv2(msg_compImg, "bgr8") # image_convert of type CVbridge converts compressed image message to cv2 format
    
    white_image_holder = None
    yellow_image_holder = None
    
    lane_filter(cv2_image, white_image_holder, yellow_image_holder) # does all of the lane filter stuff. creates white and yellow outputs that live in the scope of that function
  
  # need to publish white & yellow output images
    white_out_pub.publish(image_convert.cv2_to_imgmsg(white_image_holder, "bgr8"))
    yellow_out_pub.publish(image_convert.cv2_to_imgmsg(yellow_image_holder, "bgr8"))


if __name__ == "__main__":

    #if len(sys.argv) != 2:
    #   print("Usage: %s image_filename.png" % sys.argv[0])
    #   exit() # handles filename input which isnt used
        
    #image_filename = sys.argv[1]
    #image = cv2.imread(image_filename)
    #lane_filter(image) # not how the image is being brought in

    # make the node & make sure there is only one running and it has this name
    rospy.init_node('lutz_lane_follow', anonymous=False)
    # node needs to exist prior to doing following stuff

    # subscribes to "image_transformer_node/corrected_image/compressed"
    comp_image_sub = rospy.Subscriber("jmouse/image_transformer_node/corrected_image/compressed", CompressedImage, image_rx_callback)
    # this node wants message from this topic and when it receives them it calls the callback function 
    # compressed image topic comes from DB

    white_out_pub = rospy.Publisher('/pub_edges/white'  , Image, queue_size=1) #makes a publisher stored in white_out_pub
    yellow_out_pub = rospy.Publisher('/pub_edges/yellow' , Image, queue_size=1)

    rospy.spin() # allows the node to continuously process messages
    # exits once ROS is shut down
