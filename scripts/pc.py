#!/usr/bin/env python3
#coding=utf-8

import rospy
from extensa.msg import Mode, Seg

class console:
    def __init__(self):
        self.working_mode = 1
        self.commander()

    def commander(self):
        rospy.init_node('commander', anonymous=True)

        self.pub = rospy.Publisher('command', Seg, queue_size=50)
        
        self.sub = rospy.Subscriber('console', Mode, self.callback)

        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.working_mode:
                # working without PC
                1
            else:
                # individual mode
                1
            self.rate.sleep()

    def callback(self, data):
        if data.live:
            self.PC_live = 1
            # pre-programme
            if data.mode == 0:
                1
            # PC command
            if data.mode == 1:
                1
            # Joystick
            if data.mode == 2:
                1
        else:
            self.PC_live = 0

if __name__ == '__main__':
    try:
        CONSOLE = console()
        rospy.loginfo("console is finished")
    except rospy.ROSInterruptException:
        print(rospy.ROSInternalException)