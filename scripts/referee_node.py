#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import time
from std_msgs.msg import Bool

class RefereeNode:
    def __init__(self) -> None:
        rospy.init_node("referee_node")
        self.hider_odom_sub = rospy.Subscriber("/hider/odom", Odometry, self.hider_odom_callback)
        self.seeker_odom_sub = rospy.Subscriber("/seeker/odom", Odometry, self.seeker_odom_callback)
        self.start_game_pub = rospy.Publisher("/start_game", Bool, queue_size=10)
        self.end_game_pub = rospy.Publisher("/end_game", Bool, queue_size=10)
        
        self.hider_odom = None
        self.seeker_odom = None
        
        self.rate = rospy.Rate(10)

        self.game_duration = rospy.get_param("~game_duration", 60)
        self.caught_distance = rospy.get_param("~caught_distance", 1.0)
        self.preparation_time = rospy.get_param("~preparation_time", 15)

    def run(self) -> None:
        self.start_time = time.time()
        while time.time() - self.start_time < self.preparation_time:
            rospy.loginfo("PREPARATION TIME REMAINING: " + str(self.preparation_time - (time.time() - self.start_time)) + " seconds")
            self.rate.sleep()
        self.start_game_pub.publish(True)
        rospy.loginfo("PREPARATION TIME COMPLETE, CHASING BEGINS!")
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            if elapsed_time > self.game_duration:
                rospy.loginfo("Game time has expired, Hider WINS!")
                self.end_game_pub.publish(True)
                break
            if self.hider_odom is not None and self.seeker_odom is not None:
                hider_x = self.hider_odom.pose.pose.position.x
                hider_y = self.hider_odom.pose.pose.position.y
                seeker_x = self.seeker_odom.pose.pose.position.x
                seeker_y = self.seeker_odom.pose.pose.position.y
                distance = ((hider_x - seeker_x)**2 + (hider_y - seeker_y)**2)**0.5
                if distance < self.caught_distance:
                    rospy.loginfo("Seeker has caught the hider, Seeker WINS!")
                    self.end_game_pub.publish(True)
                    break
            self.rate.sleep()

    def hider_odom_callback(self, msg) -> None:
        self.hider_odom = msg

    def seeker_odom_callback(self, msg) -> None:
        self.seeker_odom = msg

if __name__ == "__main__":
    node = RefereeNode()
    node.run()