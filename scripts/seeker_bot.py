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
        [~] 2. Searching mode.
        [~]     2.1. Traversing algorithm.
        [X]         2.1.1. Subscribe to move_base topic
        [X]         2.1.2. Use occupancy grid to traverse the map.
        [X]         2.1.3. Generate waypoints to traverse the map.
        [~]     2.2. Detecting hider bot.
        [X]         2.2.1. Subscribe to the camera topic.
        [X]         2.2.2. Implement detection method.
        [X]         2.2.3. Compute hider bot coordinates.
        [~]     2.3. Implement the searching state machine.
        [~] 3. Chasing mode.
        [~] 4. Test the state machine.
        [ ] 5. Add safety protocol.
"""

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import quaternion_from_euler

import numpy as np
import cv2
import math
import time
import sys
import os

# Helper functions:

def eucledian_distance(x1, y1, x2, y2):
    # Calculate the distance between two points.
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def degree_to_radian(angle):
    # Convert angle from degrees to radians.
    return angle * np.pi / 180.0

# Seeker bot class:
class SeekerBot:
    def __init__(self):
        # Initialize the publisher:
        self.pub = rospy.Publisher('/seeker/cmd_vel', Twist, queue_size=10)

        # Initialize the subscriber:
        self.odom_sub = rospy.Subscriber('/seeker/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/seeker/scan', LaserScan, self.scan_callback)
        self.cam_sub = rospy.Subscriber('/seeker/camera/rgb/image_raw', Image, self.image_callback)
        self.occ_grid_sub = rospy.Subscriber('/seeker/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)

        # Initialize the timer:
        self.start_time = rospy.get_time()
        self.waiting_time = 5.0 # seconds
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

        # Traversing random waypoint:
        self.waypoint = MoveBaseGoal() # (x, y), yaw

        # Map:
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        self.show_map = True

        # Initialize the move_base client:
        self.move_base = actionlib.SimpleActionClient('/seeker/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Initialize the traversing flag:
        self.traversing = False

        # Initialize the safety protocol:
        self.protocol = 'none' # none, pursuit, evasion

        # Initialize the parameters:
        # ...
        
    ### Methods:
        
    def calculate_hider_coordinates(self):
        # Calculate the hider bot coordinates using the distance and the angle and the seeker bot coordinates.
        hider_x = self.x + self.hider_distance * math.cos(self.hider_relative_angle)
        hider_y = self.y + self.hider_distance * math.sin(self.hider_relative_angle)

        return hider_x, hider_y   

    def calculate_random_index(self, free_indices):
        # Randomly select a free cell.
        random_index = np.random.choice(free_indices[0].shape[0]) # Choose a random index from the free cells indices.
        random_cell = free_indices[0][random_index], free_indices[1][random_index] # Get the random cell indices.

        return random_cell
    
    def is_frontal(self, cell):
        # Check if the cell is in the frontal area of the seeker bot.
        robot_angle = self.yaw
        cell_angle = math.atan2(cell[1] - self.y, cell[0] - self.x)
        angle_difference = np.abs((cell_angle - robot_angle + np.pi) % (2*np.pi) - np.pi)
        if angle_difference < np.pi/2:
            return True, cell_angle
        else:
            return False, cell_angle
    
    def is_free(self, cell):
        # Check if the cell is free.
        if self.map_data[cell[0], cell[1]] == 0:
            return True
        else:
            return False
        
    def convert_coordinates_to_cell(self, coordinates):
        # Convert the coordinates to a map cell.
        # coordinates: (x, y)
        # cell: (x, y)
        # Convert the coordinates to a cell in the map frame.
        cell = ((coordinates[0] - self.map_origin.position.x) / self.map_resolution, (coordinates[1] - self.map_origin.position.y) / self.map_resolution)

        return cell

    def convert_cell_to_pose(self, cell, angle=0.0):
        # Convert the map cell to a pose.
        # cell: (x, y)
        # pose: (x, y, yaw)
        # Convert the cell coordinates to a pose in the world frame.
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = cell[0] * self.map_resolution + self.map_origin.position.x
        pose.pose.position.y = cell[1] * self.map_resolution + self.map_origin.position.y
        pose.pose.orientation.w = angle  # Assuming the robot can move in any direction.

        return pose

    def random_waypoint(self):
        # Calculate the waypoint coordinates using the map data.
        # Get the indices of the free cells.
        free_cells_indices = np.where(self.map_data == 0)

        # Randomly select a free cell in the map. (x, y)
        random_cell = self.calculate_random_index(free_cells_indices)
        cell_direction, cell_angle = self.is_frontal(random_cell)

        # Check if the free cells are in the frontal area of the seeker bot.
        while cell_direction == False:
            random_cell = self.calculate_random_index(free_cells_indices)
            cell_direction, cell_angle = self.is_frontal(random_cell)

        # Convert the random cell to a pose.
        waypoint_pose = self.convert_cell_to_pose(random_cell, cell_angle)
        self.set_waypoint(waypoint_pose.pose.position.x, waypoint_pose.pose.position.y, waypoint_pose.pose.orientation.w)
        rospy.loginfo('Waypoint: ' + str(round(waypoint_pose.pose.position.x,2)) + ', ' + str(round(waypoint_pose.pose.position.y,2)))

    def set_waypoint(self, x, y, yaw):
        self.waypoint = MoveBaseGoal()
        self.waypoint.target_pose.header.frame_id = "map"
        self.waypoint.target_pose.header.stamp = rospy.Time.now()
        self.waypoint.target_pose.pose.position.x = float(x)
        self.waypoint.target_pose.pose.position.y = float(y)
        self.waypoint.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        self.waypoint.target_pose.pose.orientation.x = q[0]
        self.waypoint.target_pose.pose.orientation.y = q[1]
        self.waypoint.target_pose.pose.orientation.z = q[2]
        self.waypoint.target_pose.pose.orientation.w = q[3]

    def start_moving_to_waypoint(self):
        # Send the goal to the robot's navigation system.
        rospy.loginfo("# Debugging move_base send_goal called")
        rospy.loginfo("Here I come, Jerry!")
        if self.move_base.get_state() is not actionlib.GoalStatus.ACTIVE:
            # It's safe to send a new goal
            self.move_base.send_goal(self.waypoint)
        else:
            rospy.logwarn("Tried to send a new goal while move_base was still processing the current goal")
        self.traversing = True
        wait = self.move_base.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base.get_state()
        
    def move_directly_to_hider(self):
        rospy.loginfo_throttle(1,'#!# Debugging move_directly_to_hider: angle:' + str(self.hider_relative_angle))

        """# Return linear and angular velocities to move directly to the hider bot.
        rospy.loginfo('#!# Debugging move_directly_to_hider: distance:' + str(self.hider_distance) + ', angle:' + str(self.hider_relative_angle))
        rospy.loginfo('#!# Debugging move_directly_to_hider: x-' + str(self.hider_coordinates[0]) + ', y-' + str(self.hider_coordinates[1]))
        # Distance to the hider bot:
        self.hider_distance = eucledian_distance(self.x, self.y, self.hider_coordinates[0], self.hider_coordinates[1])

        # Calculate the angle between the robot and the hider bot.
        self.hider_relative_angle = math.atan2(self.hider_coordinates[1] - self.y, self.hider_coordinates[0] - self.x)"""

        # Linear velocity:
        linear_velocity = 0.75 # min(self.hider_distance, 2.5) # 4.5 m/s
        # Angular velocity:
        angular_velocity = self.hider_relative_angle * 0.5 # theta * 0.5 rad/s

        return linear_velocity, angular_velocity
    
    def handle_move_base_state(self, state):
        # Handle the move_base state.
        if state == actionlib.GoalStatus.SUCCEEDED:
            # The action succeeded, do something...
            self.traversing = False
            rospy.loginfo("Goal execution done!")
        elif state == actionlib.GoalStatus.ACTIVE:
            # The action is still in progress, do something else...
            self.traversing = True
            rospy.loginfo_throttle(10,"Goal execution in progress.")
        elif state == actionlib.GoalStatus.PENDING:
            # The goal has yet to be processed by the action server
            self.traversing = False
            rospy.loginfo("Goal execution pending.")
        elif state == actionlib.GoalStatus.PREEMPTED:
            # The goal received a cancel request after it started executing
            # and has since completed its execution (Terminal State)
            self.traversing = False
            rospy.loginfo("Goal execution preempted.")
        elif state == actionlib.GoalStatus.RECALLED:
            # The goal received a cancel request before it started executing,
            # but the action server has not yet confirmed that the goal is canceled
            self.traversing = False
            rospy.loginfo("Goal execution recalled.")
        elif state == actionlib.GoalStatus.REJECTED:
            self.traversing = False
            # The action has failed
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        elif state == actionlib.GoalStatus.ABORTED:
            self.traversing = False
            # The action was canceled by the action client
            rospy.loginfo("Goal execution aborted.")
        elif state == actionlib.GoalStatus.LOST:
            self.traversing = False
            # Unknown state
            rospy.loginfo("Goal execution lost.")
        else:
            self.traversing = False
            # The action has failed
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
    
    # State machine:
    # For controlling the seeker bot movement and behavior.
    def state_machine(self):
        rospy.loginfo_throttle(5, '## Debugging state machine: ' + self.state)
        # Waiting state:
        if self.state == 'waiting':
            self.pub.publish(Twist(linear=Point(x=0), angular=Point(z=0)))
            # self.pub.publish(Twist(linear=Point(x=0.0), angular=Point(z=90.0))) # Spin in place.
        # Searching state:
        elif self.state == 'searching':
            if self.traversing == False and self.move_base.get_state() is not actionlib.GoalStatus.ACTIVE:
                # Randomly select a free cell in the map.
                rospy.loginfo('I am going... this way!')
                self.random_waypoint()

                # Send the goal to the robot's navigation system.
                move_state = self.start_moving_to_waypoint()
                self.handle_move_base_state(move_state)
            else:
                rospy.loginfo_throttle(10, 'Traversing...')
                move_state = self.move_base.get_state()
                self.handle_move_base_state(move_state) 
        # Chasing state:
        elif self.state == 'chasing':
            self.traversing = False
            rospy.loginfo_throttle(5, 'I\'m gonna getcha, Jerry!')
            # Calculate the linear and angular velocities to move directly to the hider bot.
            linear_velociy_x, angular_velocity_z = self.move_directly_to_hider()
            # Publish the velocity commands.
            self.pub.publish(Twist(linear=Point(x=linear_velociy_x), angular=Point(z=angular_velocity_z)))
        else:
            pass

    # Detect red object (hider bot):
    def detect_red_object(self):
        # Convert BGR to Hue_Saturation_Value:
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV) 

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
        return cv2.moments(final_mask)

    def calculate_hider_relative_angle(self, M):
        # From moments, calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Using the robot yaw and the x coordinate of the red object.
        fov = 82.3/2 # Field of view of the camera in degrees. --> Check this value.

        # Image width
        image_width = self.image.shape[1]

        # Calculate the normalized image coordinate (-1 to 1)
        normalized_cX = 2 * (cX / image_width - 0.5)

        # Calculate the angle (in degrees)
        theta = normalized_cX * fov

        rospy.loginfo_throttle(5, '#!# Hider angle: ' + str(theta))
        # rospy.loginfo_throttle(5, '#!# Seeker yaw: ' + str(self.yaw)) 
        self.hider_relative_angle = degree_to_radian(theta)


        # Calculate the angle relative to the robot
        # self.hider_relative_angle = theta - self.yaw
        # rospy.loginfo_throttle(5, 'Hider relative angle: ' + str(self.hider_relative_angle))

    # Check if the seeker bot is in a safe position.
    def safety_check(self):
        pass
        """
        # Use scan data to check if the seeker bot is near a wall.
        scan = np.array(self.scan)
        if np.min((scan[0:15], scan[-15:-1])) < 0.5:
            rospy.loginfo('I am too close to a wall!')
            return False

        # Use map data to check if the seeker bot is in a free cell.
        # Get the seeker bot cell indices.
        seeker_cell = self.convert_coordinates_to_cell((self.x, self.y))
        # Check if the seeker bot cell is free.
        if self.is_free(seeker_cell):
            return True
        else:
            rospy.loginfo('I am stuck!')
            return False
        """


    ## Callbacks:

    # Occupancy grid callback:
    def costmap_callback(self, msg):
        rospy.loginfo_throttle(5, '### Debugging Costmap_callback callback: ' + str(type(msg)))

        # Get the map data.
        self.map_data = msg.data
        # Get the map width.
        self.map_width = msg.info.width
        # Get the map height.
        self.map_height = msg.info.height
        # Get the map resolution.
        self.map_resolution = msg.info.resolution
        # Get the map origin.
        self.map_origin = msg.info.origin

        # Convert the map data to a numpy array.
        self.map_data = np.array(self.map_data)
        # Reshape the map data to a 2D array.
        self.map_data = self.map_data.reshape(self.map_width, self.map_height)

        # Check if the map data is valid.
        if self.map_data is not None:
            # ...
            if self.show_map:    
                # Show the map data.
                self.map_img = cv2.convertScaleAbs(self.map_data)
                # cv2.imshow('Occupancy Grid', self.map_img)
                # cv2.waitKey(1)
                # cv2.destroyAllWindows()
                self.show_map = False # Show the map only once.
        else:
            rospy.loginfo('Map data is not valid!')
        
    # Timer callback:
    def timer_callback(self, event):
        # Calculate the elapsed time.
        elapsed_time = rospy.get_time() - self.start_time
        # Check if the waiting time is over.
        if  elapsed_time > self.waiting_time and self.state == 'waiting':
            self.state = 'searching'
            self.traversing = False
            rospy.loginfo('Waiting time is over!')
            rospy.loginfo('You can hide but you cannot run, Jerry!')
            # self.pub.publish(Twist(angular=Point(z=0.0))) # Stop spinning.
        elif self.state == 'waiting':
            rospy.loginfo('Waiting for Jerry to hide. 1 2 3 ... ')
        else:
            pass

    # Odom callback:
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z

        rospy.loginfo_throttle(10, '## Debugging odom_callback callback: ' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.yaw))

        # TODO: Where to put this code?
        self.state_machine()

    # Scan callback:
    def scan_callback(self, msg):
        self.scan = msg.ranges
        if self.state == 'searching':
            if self.safety_check():
                pass
            else:
                # safety protocol
                pass

        elif self.state == 'chasing':
            # Calculate the distance to the hider bot.
            self.hider_distance = msg.ranges[int(self.hider_relative_angle)]

            # Calculate hider bot coordinates using the distance and the angle and the seeker bot coordinates.
            # self.hider_coordinates = self.calculate_hider_coordinates()

    # Image callback:
    def image_callback(self, msg):
        try:
            if self.state == 'searching' or self.state == 'chasing':    
                # rospy.loginfo('### Debugging image callback' + str(type(msg)))
                # Convert the image to OpenCV format.
                self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Blue_Green_Red 8-bit format

                M = self.detect_red_object()
                
                # Change state from 'searching' to 'chasing' if red object detected.
                if M["m00"] > 0:
                    rospy.loginfo_throttle(5, 'Haha, Jerry I see you!')
                    # Calculate the angle between the robot and the red object.
                    self.calculate_hider_relative_angle(M)
                    # Cancel the current goal.
                    if self.state != 'chasing' and self.move_base.get_state() not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.SUCCEEDED]:
                        rospy.loginfo_once('Canceling the current goal')
                        self.move_base.cancel_goal()
                        self.move_base.cancel_all_goals()
                    self.state = 'chasing'
                    self.traversing = False

                    rospy.loginfo_throttle(5, 'You better run!')
                    # self.set_waypoint(self.hider_coordinates[0], self.hider_coordinates[1], self.yaw)
                    lin_vec, ang_vec = 0.0, 0.0 # self.move_directly_to_hider()
                    self.pub.publish(Twist(linear=Point(x=lin_vec), angular=Point(z=ang_vec)))
                    # if self.start_moving_to_waypoint():
                    #   rospy.loginfo("Goal execution done!")
                else:
                    self.state = 'searching'
                    rospy.loginfo_throttle(5, 'Jerry not detected!')
                    

                # Show the images:
                # cv2.imshow('Original Image', self.image)
                # cv2.imshow('Red Objects Detection', result)
                # cv2.waitKey(1)
                # cv2.destroyAllWindows()
            else:
                pass
        except CvBridgeError as e:
            print(e)

# Main function:
def main(args):
    try:
        # Create an instance of the SeekerBot class:
        # Initialize the node:
        rospy.init_node('seeker', anonymous=False)
        seeker = SeekerBot()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.loginfo('Seeker bot Tom ready to roll!')
    main(sys.argv)



    
