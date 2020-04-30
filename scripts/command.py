#!/usr/bin/env python3
#coding=utf-8

import rospy
from extensa.msg import Mode, Pressure, Command
from extensa.srv import ik

class commander:
    def __init__(self):
        self.command_source = 0
        self.control_mode = 0
        self.data = 0
        self.commander()

    def commander(self):
        rospy.init_node('command_node', anonymous=True)

        self.pub = rospy.Publisher('command', Command, queue_size=100)
        
        self.sub = rospy.Subscriber('console', Command, self.console)

        self.rate = rospy.Rate(10) # 10hz

        # test code
        command = Command()
        command.command_source = 1
        command.command_type = 2
        self.data = command
        while not rospy.is_shutdown():
            if self.data.command_source == 1:
                # working with PC
                self.pub.publish(self.data)
            else:
                # individual mode
                1
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
        print("commander is finished")
    except rospy.ROSInterruptException as exc:
        print("Commander call failed:"+str(exc))