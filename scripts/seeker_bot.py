#!/usr/bin/env python3

"""
    This script is the main script for the seeker bot. 
    Seeker bot is an agent that is responsible for finding the target bot.
    It has 3 modes of operation: 
        1. Waiting mode: It counts for a period of time for the hider bot to hide.
        2. Searching mode: It searches for the hider bot by traversing the map.
        3. Chasing mode: It chases the hider bot once it is found.

    TODO:
        [X] 1. Waiting mode.
        [X]     1.1. Implement the timer or have a subscriber to the hider bot.
        [X]     1.2. Implement the state machine.
        [~]     1.3. Wait according to referee callback
        [X] 2. Searching mode.
        [X]     2.1. Traversing algorithm.
        [X]         2.1.1. Subscribe to move_base topic
        [X]         2.1.2. Use occupancy grid to traverse the map.
        [X]         2.1.3. Generate waypoints to traverse the map.
        [X]     2.2. Detecting hider bot.
        [X]         2.2.1. Subscribe to the camera topic.
        [X]         2.2.2. Implement detection method.
        [X]         2.2.3. Compute hider bot angle.
        [X]         2.2.4. Compute hider bot distance.
        [X]         2.2.5. Compute hider bot coordinates.
        [X]     2.3. Implement the searching state machine.
        [X] 3. Chasing mode.
        [X]     3.1. Implement the chasing state machine.
        [X]     3.2. Case when loosing sight while chasing.
        [X]     3.3. Implement the pursuit state where the seeker bot goes to the last known position of the hider bot.
        [X] 4. Test the state machine.
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

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
import cv2
import math
import sys

# Helper functions:

def eucledian_distance(x1, y1, x2, y2):
    # Calculate the distance between two points.
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def angle_between_points(x1, y1, x2, y2):
    return math.atan2(y2-y1,x2-x1)

def angle_difference(angle1, angle2):
    angle_diff = angle1 - angle2
    # Handle the case when the angle difference is bigger than 180 degrees
    if (angle_diff > math.pi):
        angle_diff -= 2 * math.pi
    elif (angle_diff < -math.pi):
        angle_diff += 2 * math.pi
    return angle_diff

def degree_to_radian(angle):
    # Convert angle from degrees to radians.
    return angle * np.pi / 180.0

def radian_to_degree(angle):
    # Convert angle from radians to degrees.
    return angle * 180.0 / np.pi

def calc_contr_linear_velocity(distance, angle_diff):
    # When the angle difference is too big, slow the robot
    if abs(angle_diff) > math.pi:     # ∠ > 180 degrees  
        linear_velocity = 0.0005  # Smoothly
    elif abs(angle_diff) > math.pi/2: # ∠ > 90 degrees
        linear_velocity = 0.005
    elif abs(angle_diff) > math.pi/4: # ∠ > 45 degrees
        linear_velocity = 0.1
    elif abs(angle_diff) > math.pi/8: # ∠ > 22.5 degrees
        linear_velocity = distance + 0.1 if (distance < 2.5) else min(distance, 0.45) 
    else:
        linear_velocity = distance + 0.25 if (distance < 2.5) else min(distance, 1.25)  # Tune the velocity
    return linear_velocity

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
        self.waiting_time = 3.0 # seconds
        self.timer = rospy.Timer(rospy.Duration(self.waiting_time), self.timer_callback)

        # Initialize the state:
        self.state = 'waiting' # waiting, searching, chasing, pursuit

        # Initialize the variables:
        # Seeker bot:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan = []
        self.image = None
        self.bridge = CvBridge()

        # Hider bot:
        self.hider_relative_angle = 0.0 # Angle between the robot and the hider bot.
        self.hider_distance = 0.0   # Distance to the hider bot.
        self.hider_coordinates = (0.0, 0.0) # (x, y)
        self.spot_distance = 0.0    # Distance to the last known position of the hider bot.

        # Traversing random waypoint:
        self.waypoint = MoveBaseGoal() # (x, y), yaw

        # Map:
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        self.show_map = True

        # Traversal Limits:
        self.max_coord = 10.0
        self.max_scan_range = 10.0 # thğis is the max safe range of the laser scan

        # Initialize the move_base client:
        self.move_base = actionlib.SimpleActionClient('/seeker/move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        # Initialize the traversing flag:
        self.traversing = False
        self.traversing_start_time = 0.0
        self.max_search_period = 60.0

        # Initialize the safety protocol:
        # self.protocol = 'none' # none, pursuit, evasion

        # Initialize the parameters:
        # ...
        
    ### Methods:
        
    def calculate_hider_coordinates(self):
        # Calculate the hider bot coordinates using the distance and the angle and the seeker bot coordinates.
        angle = -self.hider_relative_angle + self.yaw
        hider_x = self.x + self.hider_distance * math.cos(angle)
        hider_y = self.y + self.hider_distance * math.sin(angle)
        rospy.loginfo_throttle(1, '#!# Hider coordinates: x=' + str(round(hider_x,3)) + ', y=' + str(round(hider_y,3)) + ', yaw=' + str(round(radian_to_degree(angle),2)) + ' degrees')
        return hider_x, hider_y   

    def select_random_index(self, indices):
        # Randomly select a cell.
        random_index = np.random.choice(indices[0].shape[0]) # Choose a random index from the free cells indices.
        random_cell = indices[0][random_index], indices[1][random_index] # Get the random cell indices.

        return random_cell
    
    def is_frontal(self, coordinates):
        # Check if the cell is in the frontal area of the seeker bot.
        x_r, y_r = coordinates
        robot_angle = self.yaw
        angle = angle_between_points(self.x, self.y, x_r, y_r) # FIXME
        angle_diff = angle_difference(angle, robot_angle)
        """
        rospy.loginfo_throttle(1, '#>x<# Cell Angle: ' + str(round(radian_to_degree(angle),2)) + ' degrees')
        rospy.loginfo_throttle(1, '#^n^# Angle difference: ' + str(round(radian_to_degree(angle_diff),2)) + ' degrees')
        """
        if abs(angle_diff) > math.pi/2: # ∠ > 90 degrees
            return False, angle    # The cell is not in the frontal area of the seeker bot.
        else:
            return True, angle     # The cell is in the frontal area of the seeker bot.
    
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
        cell = ((coordinates[0] - self.map_origin.position.x) // self.map_resolution, (coordinates[1] - self.map_origin.position.y) // self.map_resolution)

        return cell
    
    def convert_cell_to_coordinates(self, cell):
        # Convert the map cell to coordinates.
        # cell: (x, y)
        # coordinates: (x, y)
        # Convert the cell coordinates to coordinates in the world frame.
        coordinates = (cell[0] * self.map_resolution + self.map_origin.position.x, cell[1] * self.map_resolution + self.map_origin.position.y)

        return coordinates

    def convert_cell_to_pose(self, cell, yaw=0.0):
        # Convert the map cell to a pose.
        # cell: (x, y)
        # pose: (x, y, yaw)
        # Convert the cell coordinates to a pose in the world frame.
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = cell[0] * self.map_resolution + self.map_origin.position.x
        pose.pose.position.y = cell[1] * self.map_resolution + self.map_origin.position.y
        self.waypoint.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        self.waypoint.target_pose.pose.orientation.x = q[0]
        self.waypoint.target_pose.pose.orientation.y = q[1]
        self.waypoint.target_pose.pose.orientation.z = q[2]
        self.waypoint.target_pose.pose.orientation.w = q[3]

        return pose
    
    def convert_coordinates_to_pose(self, coordinates, yaw=0.0):
        # Convert the coordinates to a pose.
        # coordinates: (x, y)
        # pose: (x, y, yaw)
        # Convert the coordinates to a pose in the world frame.
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = coordinates[0]
        pose.pose.position.y = coordinates[1]
        self.waypoint.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        self.waypoint.target_pose.pose.orientation.x = q[0]
        self.waypoint.target_pose.pose.orientation.y = q[1]
        self.waypoint.target_pose.pose.orientation.z = q[2]
        self.waypoint.target_pose.pose.orientation.w = q[3]

        return pose
    


    def random_waypoint(self):
        # Calculate the waypoint coordinates using the map data.
        # Get the indices of the free cells.
        free_cells_indices = np.where(self.map_data == 0)

        # Randomly select a free cell in the map. (x, y)
        random_cell = self.select_random_index(free_cells_indices)
        random_coord = self.convert_cell_to_coordinates(random_cell)
        cell_direction, cell_angle = self.is_frontal(random_coord)

        # Check if the free cells are in the frontal area of the seeker bot.
        while cell_direction == False:
            rospy.loginfo_throttle(1, '>Cell is not frontal! x=' + str(round(random_coord[0],2)) + ', y=' + str(round(random_coord[1],2)))
            random_cell = self.select_random_index(free_cells_indices)
            random_coord = self.convert_cell_to_coordinates(random_cell)
            cell_direction, cell_angle = self.is_frontal(random_coord)

        # Check if the waypoint is within the map limits.
        # Generate a new waypoint if the waypoint is out of the map limits.
        while abs(random_coord[0]) > self.max_coord or abs(random_coord[1]) > self.max_coord:
            rospy.loginfo_throttle(1, '>Cell is out of bounds! x=' + str(round(random_coord[0],2)) + ', y=' + str(round(random_coord[1],2)))
            random_cell = self.select_random_index(free_cells_indices)
            random_coord = self.convert_cell_to_coordinates(random_cell)
            cell_direction, cell_angle = self.is_frontal(random_coord)
            while cell_direction == False:
                random_cell = self.select_random_index(free_cells_indices)
                random_coord = self.convert_cell_to_coordinates(random_cell)
                cell_direction, cell_angle = self.is_frontal(random_coord)

        # Convert the random cell to a pose.
        waypoint_pose = self.convert_coordinates_to_pose(random_coord, cell_angle)

        return waypoint_pose

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
        # FIXME: Should not enter here more than once
        rospy.loginfo("# Debugging move_base send_goal called")
        rospy.loginfo("Here I come, Jerry!")
        if self.move_base.get_state() is not actionlib.GoalStatus.ACTIVE and self.traversing is False:
            # It's safe to send a new goal
            self.move_base.send_goal(self.waypoint)
            self.traversing = True
        else:
            rospy.logwarn("Tried to send a new goal while move_base was still processing the current goal")
        """wait = self.move_base.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:"""
        return self.move_base.get_state()
        
    def move_towards_hider(self):
        rospy.loginfo_throttle(1,'#~# Debugging move_directly_to_hider: angle= ' + str(round(self.hider_relative_angle, 3)) + ' rad')

        # Linear velocity:
        linear_velocity = min(self.hider_distance, 0.75) # 4.5 m/s
        # Angular velocity:
        angular_velocity = self.hider_relative_angle * 0.5 # theta * 0.5 rad/s

        return linear_velocity, angular_velocity
    
    def move_to_hider_last_point(self):
        # Calculate distance to the hider bot.
        self.spot_distance = eucledian_distance(self.x, self.y, self.hider_coordinates[0], self.hider_coordinates[1])

        # Calculate the angle between the robot and the hider bot.
        angle = angle_between_points(self.x, self.y, self.hider_coordinates[0], self.hider_coordinates[1])

        # Calculate the angle difference between the robot and the point.
        angle_diff = angle_difference(angle, self.yaw)

        # Calculate the linear velocity
        linear_velocity = calc_contr_linear_velocity(self.spot_distance, angle_diff) # min(distance + 0.1, 0.5) # 0.5 m/s

        # Angular velocity:
        angular_velocity = angle_diff * 0.5 # theta * 0.5 rad/s

        return linear_velocity, angular_velocity

    
    def handle_move_base_state(self, state):
        # Handle the move_base state.
        if state == actionlib.GoalStatus.SUCCEEDED:
            # The action succeeded, do something...
            self.traversing = False
            rospy.loginfo_throttle(1, "||| Goal execution done!")
        elif state == actionlib.GoalStatus.ACTIVE:
            # The action is still in progress, do something else...
            self.traversing = True
            rospy.loginfo_throttle(5,"||| Goal execution in progress.")
        elif state == actionlib.GoalStatus.PENDING:
            # The goal has yet to be processed by the action server
            rospy.loginfo("||| Goal execution pending.")
        elif state == actionlib.GoalStatus.PREEMPTED:
            # The goal received a cancel request after it started executing
            # and has since completed its execution (Terminal State)
            rospy.loginfo_throttle(1,"||| Goal execution preempted.")
            self.traversing = False
        elif state == actionlib.GoalStatus.RECALLED:
            # The goal received a cancel request before it started executing,
            # but the action server has not yet confirmed that the goal is canceled
            rospy.loginfo_throttle(1,"||| Goal execution recalled.")
        elif state == actionlib.GoalStatus.REJECTED:
            # The action has failed
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        elif state == actionlib.GoalStatus.ABORTED:
            # The action was canceled by the action client
            rospy.loginfo_throttle(1,"||| Goal execution aborted.")
            self.traversing = False
        elif state == actionlib.GoalStatus.LOST:
            # Unknown state
            rospy.loginfo("||| Goal execution lost.")
        else:
            self.traversing = False
            # The action has failed
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
    
    # State machine:
    # For controlling the seeker bot movement and behavior.
    def state_machine(self):
        rospy.loginfo_throttle(1, '[%] State machine: ' + self.state)
        # Waiting state:
        if self.state == 'waiting':
            self.pub.publish(Twist(linear=Point(x=0), angular=Point(z=0)))
            # self.pub.publish(Twist(linear=Point(x=0.0), angular=Point(z=90.0))) # Spin in place.
        # Searching state:
        elif self.state == 'searching':
            if self.traversing == False and self.move_base.get_state() not in [actionlib.GoalStatus.ACTIVE]:# , actionlib.GoalStatus.PENDING]:
                # Randomly select a free cell in the map.
                rospy.loginfo('I am going... this way!')
                waypoint_pose = self.random_waypoint()
                rospy.loginfo('Waypoint: ' + str(round(waypoint_pose.pose.position.x,2)) + ', ' + str(round(waypoint_pose.pose.position.y,2)))
                self.set_waypoint(waypoint_pose.pose.position.x, waypoint_pose.pose.position.y, waypoint_pose.pose.orientation.w)

                # Send the goal to the robot's navigation system.
                move_state = self.start_moving_to_waypoint()
                self.handle_move_base_state(move_state)
                self.traversing_start_time = rospy.get_time()
            else:
                # TODO: Implement the case when the seeker bot is stuck.
                elapsed_time = rospy.get_time() - self.traversing_start_time
                rospy.loginfo_throttle(10, 'Traversing for '+ str(round(elapsed_time)) + ' seconds...')
                rospy.loginfo_throttle(10, 'Target: ' + str(round(self.waypoint.target_pose.pose.position.x,2)) + ', ' + str(round(self.waypoint.target_pose.pose.position.y,2)))
                move_state = self.move_base.get_state()
                self.handle_move_base_state(move_state) 
                if elapsed_time > self.max_search_period:
                    rospy.loginfo('I am tired of searching!')
                    self.traversing = False
                    # Cancel the current goal.
                    self.move_base.cancel_all_goals()
                    self.pub.publish(Twist(linear=Point(x=0), angular=Point(z=0)))
        # Chasing state:
        elif self.state == 'chasing':
            self.traversing = False
            rospy.loginfo_throttle(5, 'I\'m gonna getcha, Jerry!')
            rospy.loginfo_throttle(1, 'Measures: angle ' + str(round(radian_to_degree(self.hider_relative_angle),2)) + ' degrees, distance ' + str(round(self.hider_distance,3)))
            if self.hider_distance < 1.0:
                rospy.loginfo('I caught you, Jerry. Muahahaha!')
                rospy.loginfo('Success: angle ' + str(round(radian_to_degree(self.hider_relative_angle),2)) + ' degrees, distance ' + str(round(self.hider_distance,3)))
                rospy.loginfo('Hider spot: x=' + str(round(self.hider_coordinates[0],2)) + ', y=' + str(round(self.hider_coordinates[1],3)))
                self.state = 'done'
                
                self.traversing = False
                linear_velociy_x, angular_velocity_z = 0., 0.
            else:
                # Calculate the linear and angular velocities to move directly to the hider bot.
                linear_velociy_x, angular_velocity_z = self.move_towards_hider()
            # Publish the velocity commands.
            self.pub.publish(Twist(linear=Point(x=linear_velociy_x), angular=Point(z=angular_velocity_z)))
        # Pursuit state:
        elif self.state == 'pursuit':
            self.traversing = False
            rospy.loginfo_throttle(5, 'I will pursue you till the end!')
            rospy.loginfo_throttle(1, 'Hider last seen: x=' + str(round(self.hider_coordinates[0],2)) + ', y=' + str(round(self.hider_coordinates[1],3)))
            # Move towards the last known position of the hider bot.
            # Calculate the linear and angular velocities to move directly to the hider bot.
            linear_velociy_x, angular_velocity_z = self.move_to_hider_last_point()
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

        rospy.loginfo_throttle(1, '#!# Hider rel. angle: ' + str(round(theta,2)) + ' degrees')
        # rospy.loginfo_throttle(5, '#!# Seeker yaw: ' + str(self.yaw)) 
        self.hider_relative_angle = degree_to_radian(theta)


        # Calculate the angle relative to the robot
        # self.hider_relative_angle = theta - self.yaw
        # rospy.loginfo_throttle(5, 'Hider relative angle: ' + str(self.hider_relative_angle))
    
    def calculate_hider_distance(self):
        # FIXME: Problem calculating distance
        # Degree correct
        # Scan distance incorrect
        # Use scan array data and the hider relative angle to calculate the distance to the hider bot.
        d = int(radian_to_degree(self.hider_relative_angle))
        # neighboring indices to d
        neighbors = [d-2, d-1, d, d+1, d+2]
        selected_scan_data = [self.scan[i] for i in neighbors]
        self.hider_distance = min(selected_scan_data + [self.max_scan_range])   # 10.0 is the max range of the laser scan
        rospy.loginfo_throttle(1, '#!# Hider distance: ' + str(round(self.hider_distance,3)))
        rospy.loginfo_throttle(1, '#?# Periphery angle degrees: \n' + str(neighbors))
        rospy.loginfo_throttle(1, '#?# Periphery distances: \n' + str([round(self.scan[i],2) for i in neighbors]))
        

    # Check if the seeker bot is in a safe position.
    def safety_check(self):
        """scan = np.array(self.scan)
        if np.min(scan) < 0.2:"""
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

        # rospy.loginfo_throttle(5, '### Debugging Costmap_callback callback: map width=' + str(self.map_width) + ', map height=' + str(self.map_height) + ', map resolution=' + str(self.map_resolution) + ', map origin=' + str(self.map_origin))

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
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        rospy.loginfo_throttle(10, '## Debugging odom_callback callback: x=' + str(round(self.x,3)) + ', y=' + str(round(self.y,3)) + ', yaw=' + str(round(self.yaw,3)))

        # NOTE: Where to put this code?
        self.state_machine()

    # Scan callback:
    def scan_callback(self, msg):
        self.scan = msg.ranges
        # rospy.loginfo_throttle(5, '??? Frontal distances: \n' + str([round(self.scan[i],2) for i in range(-15, 15)]))
        if self.state == 'searching':
            if self.safety_check():
                pass
            else:
                # safety protocol
                pass

        elif self.state == 'chasing':
            pass
            # Calculate the distance to the hider bot.
            # self.hider_distance = msg.ranges[int(self.hider_relative_angle)]

            # Calculate hider bot coordinates using the distance and the angle and the seeker bot coordinates.
            # self.hider_coordinates = self.calculate_hider_coordinates()

    # Image callback:
    def image_callback(self, msg):
        try:
            if self.state == 'searching' or self.state == 'chasing' or self.state == 'pursuit':    
                # rospy.loginfo('### Debugging image callback' + str(type(msg)))
                # Convert the image to OpenCV format.
                self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Blue_Green_Red 8-bit format

                M = self.detect_red_object()
                
                # Change state from 'searching' to 'chasing' if red object detected.
                if M["m00"] > 0:
                    rospy.loginfo_throttle(5, 'Haha, Jerry I see you!')
                    rospy.loginfo_throttle(1, '#># Odom : x=' + str(round(self.x, 3)) + ', y=' + str(round(self.y, 3)) + ', yaw=' + str(round(radian_to_degree(self.yaw), 2)) + ' degrees')
                    # Calculate the angle between the robot and the red object.
                    self.calculate_hider_relative_angle(M)
                    # Calculate the distance to the red object.
                    self.calculate_hider_distance()
                    # Calculate hider bot coordinates using the distance and the angle and the seeker bot coordinates.
                    if self.state == 'chasing' and self.hider_distance > 9.9:
                        pass
                    else:
                        self.hider_coordinates = self.calculate_hider_coordinates()

                    # Cancel the current goal.
                    if self.state != 'chasing' :
                        rospy.loginfo('Canceling the current goal')
                        # self.move_base.cancel_goal()
                        self.move_base.cancel_all_goals()

                        lin_vec, ang_vec = 0., 0. # self.move_directly_to_hider()
                        self.pub.publish(Twist(linear=Point(x=lin_vec), angular=Point(z=ang_vec)))
                    self.state = 'chasing'
                    self.traversing = False

                    rospy.loginfo_throttle(5, 'You better run!')
                    # self.set_waypoint(self.hider_coordinates[0], self.hider_coordinates[1], self.yaw)
                    
                    # if self.start_moving_to_waypoint():
                    #   rospy.loginfo("Goal execution done!")
                elif M["m00"] == 0 and self.state == 'chasing':
                    rospy.loginfo_throttle(1, 'I lost you, Jerry!')
                    self.state = 'pursuit'
                    self.traversing = False
                    # self.set_waypoint(self.hider_coordinates[0], self.hider_coordinates[1], self.yaw)
                    # self.start_moving_to_waypoint()
                elif M["m00"] == 0 and self.state == 'pursuit':
                    if self.spot_distance < 0.6:
                        rospy.loginfo_throttle(1, 'Where did you go!?')
                        self.state = 'searching'
                        self.traversing = False
                        lin_vec, ang_vec = 0., 0. # Stop to reassess the situation.
                        self.pub.publish(Twist(linear=Point(x=lin_vec), angular=Point(z=ang_vec)))
                else:
                    self.state = 'searching'
                    rospy.loginfo_throttle(5, 'Jerry not detected.')
                    

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



    
