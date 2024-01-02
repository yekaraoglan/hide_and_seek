#!/usr/bin/env python

import rospy
import random
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from send_goal import movebase_client
import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage, signal
from matplotlib import pyplot as plt
class Args:
    robot = "hider"
    x = None
    y = None
    yaw = None


class HiderBot:
    def __init__(self):
        rospy.init_node('hider_bot')
        rospy.Subscriber('/hider/odom', Odometry, self.odometry_callback)
        rospy.Subscriber('/hider/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/hider/move_base/global_costmap/costmap', OccupancyGrid, self.map_callback)
        self.cmd_vel_pub = rospy.Publisher('/hider/cmd_vel', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.state = "Moving"
        self.timeState = "Exploring"
        self.clump_id = 0
        self.start_time = time.time()
        self.idle_time = 0
        self.twist = Twist()
        self.args = Args()
        self.map_data = None
        self.start_yaw = 0
        self.centers = None
        
    def timer_callback(self, event):
        rospy.loginfo_throttle(1, self.timeState)
        elapsed_time = time.time() - self.start_time
        idle_time = time.time() - self.idle_time
        if elapsed_time > 15 and self.timeState != "Hiding" and self.timeState != "Moving to spot":
            self.timeState = "Finding spot"
        if self.timeState == "Hiding":
            if idle_time > 5:
                self.timeState = "Finding spot"
        

    def map_callback(self, msg):
        rospy.loginfo_throttle(1, "Map callback")
        self.map_data = np.array(msg.data).reshape((400, 400))
        # Process the map data here
        
            
    def find_biggest_clumps(self, num_clumps=5):
        print("Finding clumps")
        binary_image = (self.map_data == 0).astype(int)
        convolved = signal.convolve2d(binary_image, np.ones((50, 50)), mode='same', boundary='fill', fillvalue=0)
        mask = binary_image * convolved
        # centers = np.argmax(mask, num_clumps)
        centers = mask.flatten().argsort()[-num_clumps:][::-1]
        centers = np.unravel_index(centers, mask.shape)
        print("Found clumps")
        
        # structuring_element_size = 40
        # eroded_image = ndimage.binary_erosion(binary_image, structure=np.ones((structuring_element_size, structuring_element_size)))
        # dilated_image = ndimage.binary_dilation(eroded_image, structure=np.ones((structuring_element_size, structuring_element_size)))
        # labeled_array, num_features = ndimage.label(dilated_image > 0)
        # component_sizes = np.bincount(labeled_array.ravel())
        # largest_components_indices = np.argsort(component_sizes)[::-1][1:4]
        # largest_components = np.isin(labeled_array, largest_components_indices)
        # centers = []
        # for i in range(1, 4):
        #     coordinates = np.argwhere(labeled_array == i)
        #     center = np.mean(coordinates, axis=0)
        #     centers.append(center)
        #     plt.figure(figsize=(10, 5))

        #     plt.subplot(1, 3, 1)
        #     plt.imshow(binary_image, cmap='viridis', origin='upper', interpolation='nearest')
        #     plt.title('Original Array')
        #     for center in centers:
        #         plt.plot(center[1], center[0], 'ro')  # Swap indices for plotting
        #     plt.title('Centers of the Three Biggest Circular Patches of Zeros')

        #     plt.subplot(1, 3, 2)
        #     plt.imshow(eroded_image, cmap='viridis', origin='upper', interpolation='nearest')
        #     for center in centers:
        #         plt.plot(center[1], center[0], 'ro')  # Swap indices for plotting
        #     plt.title('Centers of the Three Biggest Circular Patches of Zeros')

        #     plt.subplot(1, 3, 3)
        #     plt.imshow(dilated_image, cmap='viridis', origin='upper', interpolation='nearest')
        #     plt.title('Dilated Image')
        #     for center in centers:
        #         plt.plot(center[1], center[0], 'ro')  # Swap indices for plotting
        #     plt.title('Centers of the Three Biggest Circular Patches of Zeros')


        #     plt.show()
        return centers

    def turn_degrees(self, degrees):
        desired_theta = self.start_yaw + math.radians(degrees)
        angle = desired_theta - self.current_pose[2]
        self.twist.linear.x = 0.0
        if angle < 0:
            angle = angle + 2*math.pi
        if angle >= 0 and angle <= math.pi:
            self.twist.angular.z = 0.35*math.ceil(angle*2.5)
        elif angle > math.pi and angle <= 2*math.pi:
            angle = 2*math.pi - angle
            self.twist.angular.z = -0.35*math.ceil(angle*2.5)
        self.cmd_vel_pub.publish(self.twist) #start rotation
        if abs(angle) < 0.1:
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.state = "Moving"
    

    def odometry_callback(self, msg):
        if self.timeState == "Exploring" or self.timeState == "Hiding":
            pass
        if self.timeState == "Finding spot":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            if (self.clump_id == 0):
                self.centers = np.array(self.find_biggest_clumps()).T
            print(self.centers/20.0 - 10.0)
            center = self.centers[self.clump_id] / 20.0 - 10.0
            self.clump_id = (self.clump_id + 1) % 5
            self.timeState = "Moving to spot"
        if self.timeState == "Moving to spot":
            self.args.x = center[0]
            self.args.y = center[1]
            self.args.yaw = self.current_pose[2]
            movebase_client(self.args)
            self.timeState = "Hiding"
            self.idle_time = time.time()
        
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self.timeState == "Exploring":
            if self.state == "Moving":
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
            elif self.state == "Turning Left":
                degrees = random.uniform(90, 150)
                self.turn_degrees(degrees)
            elif self.state == "Turning Right":
                degrees = random.uniform(90, 150)
                self.turn_degrees(-degrees)

    def scan_callback(self, msg):
        if self.state == "Moving" and self.timeState == "Exploring":
            if (min(msg.ranges[:30]+msg.ranges[-30:]) < 1.5):
                self.twist.linear.x = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.start_yaw = self.current_pose[2]
                if np.argmin(msg.ranges[:30]+msg.ranges[-30:]) > 30:
                    self.state = "Turning Left"
                else:
                    self.state = "Turning Right"
        # print(self.state)
            


if __name__ == '__main__':
    hider_bot = HiderBot()
    rospy.spin()
