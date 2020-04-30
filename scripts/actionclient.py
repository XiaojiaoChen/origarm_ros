#! /usr/bin/env python

import roslib
roslib.load_manifest('extensa')
import rospy
import actionlib

from extensa.msg import FeedbackAction, FeedbackGoal

if __name__ == '__main__':
    rospy.init_node('feedback_client')
    client = actionlib.SimpleActionClient('feedback', FeedbackAction)
    client.wait_for_server()

    goal = FeedbackGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))