#! /usr/bin/env python3

import roslib
roslib.load_manifest('extensa')
import rospy
import actionlib

from extensa.msg import feedbackAction

class server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('extensqa', feedbackAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('server')
  server = server()
  rospy.spin()