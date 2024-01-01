#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import argparse
from tf.transformations import quaternion_from_euler

def movebase_client(args):

    seeker_client = actionlib.SimpleActionClient('/seeker/move_base',MoveBaseAction)
    hider_client = actionlib.SimpleActionClient('/hider/move_base',MoveBaseAction)
    seeker_client.wait_for_server()
    hider_client.wait_for_server()
    if args.robot == 'seeker':
        client = seeker_client
    elif args.robot == 'hider':
        client = hider_client
    else:
        rospy.logerr("Invalid robot name!")
        rospy.signal_shutdown("Invalid robot name!")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(args.x)
    goal.target_pose.pose.position.y = float(args.y)
    goal.target_pose.pose.orientation.w = quaternion_from_euler(0, 0, float(args.yaw))[3]

    rospy.loginfo("Sending goal")
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--robot', type=str, help='robot name')
        parser.add_argument('--x', type=float, help='x coordinate')
        parser.add_argument('--y', type=float, help='y coordinate')
        parser.add_argument('--yaw', type=float, help='yaw angle')
        args = parser.parse_args()
        rospy.init_node('movebase_client_py')
        result = movebase_client(args)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
