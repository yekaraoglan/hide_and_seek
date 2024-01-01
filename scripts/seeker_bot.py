#!/usr/bin/env python3

"""
    This script is the main script for the seeker bot. 
    Seeker bot is an agent that is responsible for finding the target bot.
    It has 3 modes of operation: 
        1. Waiting mode: It counts for a period of time for the hider bot to hide.
        2. Searching mode: It searches for the hider bot by traversing the map.
        3. Chasing mode: It chases the hider bot once it is found.

    TODO:
        [~] 1. Waiting mode.
        [X]     1.1. Implement the timer or have a subscriber to the hider bot.
        [X]     1.2. Implement the state machine.
        [ ] 2. Searching mode.
        [ ]     2.1. Traversing algorithm.
        [ ]         2.1.1. Subscribe to move_base topic
        [ ]         2.1.2. Use slam map to traverse the map.
        [ ]         2.1.3. Generate a list of waypoints to traverse the map.
        [~]     2.2. Detecting hider bot.
        [X]         2.2.1. Subscribe to the camera topic.
        [X]         2.2.2. Implement detection method.
        [X]         2.2.3. Compute hider bot coordinates.
        [ ]     2.3. Implement the state machine.
        [ ] 3. Chasing mode.
        [ ] 4. Implement the state machine.
        [ ] 5. Implement the main function.
"""

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import math
import time
import sys
import os

# Seeker bot class:
class SeekerBot:
    def __init__(self):
        # Initialize the node:
        rospy.init_node('seeker_bot', anonymous=True)

        # Initialize the publisher:
        self.pub = rospy.Publisher('/seeker/cmd_vel', Twist, queue_size=10)

        # Initialize the subscriber:
        self.sub = rospy.Subscriber('/seeker/odom', Odometry, self.odom_callback)
        self.sub = rospy.Subscriber('/seeker/scan', LaserScan, self.scan_callback)
        self.sub = rospy.Subscriber('/seeker/camera/rgb/image_raw', Image, self.image_callback)

        # Initialize the timer:
        self.start_time = rospy.get_time()
        self.waiting_time = 10.0 # seconds
        self.timer = rospy.Timer(rospy.Duration(self.waiting_time), self.timer_callback)

        # Initialize the state:
        self.state = 'waiting' # waiting, searching, chasing

        # Initialize the variables:
        # Seeker bot:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan = []
        self.image = None
        self.bridge = CvBridge()

        # Hider bot:
        self.hider_relative_angle = 0.0
        self.hider_distance = 0.0
        self.hider_coordinates = (0.0, 0.0)
        

        # Initialize the parameters:
        # ...
        
        
    # Timer callback:
    def timer_callback(self, event):
        elapsed_time = rospy.get_time() - self.start_time
        # Check if the waiting time is over.
        if  elapsed_time > self.waiting_time and self.state == 'waiting':
            self.state = 'searching'
            rospy.loginfo('Waiting time is over!')
            rospy.loginfo('You can hide but you cannot run, Jerry!')
        elif self.state == 'waiting':
            self.state = 'waiting'
            rospy.loginfo('Waiting for Jerry to hide!')
        else:
            pass
        

    # Odom callback:
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z

    # Scan callback:
    def scan_callback(self, msg):
        self.scan = msg.ranges

        if self.state == 'chasing':
            # Calculate the distance to the hider bot.
            self.hider_distance = msg.ranges[int(self.hider_relative_angle)]

            # Calculate hider bot coordinates using the distance and the angle and the seeker bot coordinates.
            self.hider_coordinates = self.calculate_hider_coordinates()

            
    def calculate_hider_coordinates(self):
        # Calculate the hider bot coordinates using the distance and the angle and the seeker bot coordinates.
        hider_x = self.x + self.hider_distance * math.cos(self.hider_relative_angle)
        hider_y = self.y + self.hider_distance * math.sin(self.hider_relative_angle)

        return hider_x, hider_y

    # Image callback:
    def image_callback(self, msg):
        try:
            if self.state != 'waiting':    
                self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Blue_Green_Red 8-bit format
                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV) # Convert BGR to Hue_Saturation_Value:

                ## Detect red color:

                # Define a lower and upper range for the red color in the HSV color space. 
                # Red can span both low and high values due to its position on the color wheel.
                lower_red = np.array([0, 50, 50])
                upper_red = np.array([10, 255, 255])
                # Define a second range to account for red wrapping around the color wheel
                lower_red2 = np.array([160, 100, 100])
                upper_red2 = np.array([180, 255, 255])
                # Create a binary mask by applying the inRange function to filter out the red color.
                mask1 = cv2.inRange(hsv, lower_red, upper_red)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                # Combine the masks to handle the wrap-around case
                final_mask = cv2.bitwise_or(mask1, mask2)
                # Apply the mask to the original image to extract the red colored objects
                # result = cv2.bitwise_and(self.image, self.image, mask=final_mask)

                # Check if red object detected.
                # Calculate moments of the binary image
                M = cv2.moments(final_mask)
                # Change state from 'searching' to 'chasing' if red object detected.
                if M["m00"] > 0:
                    # From moments, calculate x,y coordinate of center
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Calculate the angle between the robot and the red object.
                    # Using the robot yaw and the x coordinate of the red object.
                    # WIP
                    fov = 90 # Field of view of the camera in degrees. --> Check this value.

                    # Image width
                    image_width = self.image.shape[1]

                    # Calculate the normalized image coordinate (-1 to 1)
                    normalized_cX = 2 * (cX / image_width - 0.5)

                    # Calculate the angle (in degrees)
                    theta = normalized_cX * fov

                    # Calculate the angle relative to the robot
                    self.hider_relative_angle = theta - self.yaw

                    self.state = 'chasing'
                    rospy.loginfo('Jerry detected!')

                else:
                    self.state = 'searching'
                    rospy.loginfo('Jerry not detected!')


                # Show the images:
                cv2.imshow('Original Image', self.image)
                # cv2.imshow('Red Objects Detection', result)
                cv2.waitKey(0)
                cv2.destroyAllWindows()


            



        except CvBridgeError as e:
            print(e)


    
