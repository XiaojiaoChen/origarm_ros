#!/usr/bin/env python3
#coding=utf-8

import rospy
from origarm_ros.msg import *
from origarm_ros.srv import ik

class commander:
    def __init__(self):
        self.command_source = 0
        self.control_mode = 0
        self.data = 0
        self.commander()

    def commander(self):
        rospy.init_node('command_node', anonymous=True)

        self.pub = rospy.Publisher('Command_Position', Command_Position, queue_size=100)
    

        self.rate = rospy.Rate(10) # 10hz

        # test code
        # command = Command_Position()
        command.pose.position.x = 2.0
        command.pose.position.y = 3.0
        command.pose.position.z = 4.0
        command.pose.orientation.x = 1
        command.pose.orientation.w = 1
        

        self.data = command
        print("commander is ready")
        while not rospy.is_shutdown():
            # working with PC
            self.pub.publish(self.data)
            self.rate.sleep()

    def console(self, data):
        # pre-programme / PC command / Joystick
        # position / pressure /openning / forward
        self.data = data

    def ik_client(self, x, y):
        rospy.wait_for_service('ik')
        try:
            ik_srv = rospy.ServiceProxy('ik', ik)
            resp1 = ik_srv(x, y)
            print(resp1.desired)
            return resp1.desired
        except rospy.ServiceException as exc:
            print("Service call failed:"+str(exc))


if __name__ == '__main__':
    try:
        COMMANDER = commander()
        
    except rospy.ROSInterruptException as exc:
        print("Commander call failed:"+str(exc))