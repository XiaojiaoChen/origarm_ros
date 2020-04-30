#!/usr/bin/env python
# coding=utf-8

import sys
import rospy
from extensa.srv import ik
from extensa.msg import IKpose

def ik_client(pose):
    rospy.wait_for_service('ik')
    try:
        ik_srv = rospy.ServiceProxy('ik', ik)
        print(pose)
        resp1 = ik_srv(pose)
        print('finish')
        return resp1.output
    except rospy.ServiceException as exc:
        print( "Service call failed:" + str(exc))

if __name__ == "__main__":
    InPut = IKpose()
    InPut.pose.position.x=1.0
    InPut.pose.position.y=1.0
    InPut.pose.position.z=1.0

    print(ik_client(InPut))
