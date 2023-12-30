#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

import tf

class TFPublisher:
    def __init__(self):
        rospy.init_node('odom_to_baselink')
        self.robot_name = rospy.get_param('~robot_name', 'robot')
        print(f'Robot name: {self.robot_name}')
        self.odom_sub = rospy.Subscriber(f'/{self.robot_name}/odom', Odometry, self.odom_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def odom_callback(self, msg):
        self.tf_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            f'{self.robot_name}/base_footprint',
            'odom'
        )

if __name__ == '__main__':
    tf_publisher = TFPublisher()
    rospy.spin()