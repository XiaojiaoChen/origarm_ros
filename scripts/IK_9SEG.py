#!/usr/bin/env python3
#coding=utf-8

import numpy as np
import time
from math import cos, sin, sqrt, pi, atan, asin, atan, atan2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from angles import normalize_angle
import rospy
from origarm_ros.srv import ik
from origarm_ros.msg import *
import traceback

class ik_solver:
    def __init__(self):
        self.seg = 0
        self.pts = [0,0,0]
        self.A = 0
        self.N = 0
        self.flag = 0
        self.desired = 0
        self.ik_srv_setup()

    def handle_ik_srv(self, req):
        result = self.inverse_kinematic(
            req.input.pose.position,
            req.input.pose.orientation,
            req.input.ABL,
            req.mode
        ) 
        # return Command_ABL
        return result

    def ik_srv_setup(self):
        rospy.init_node('ik_srv')
        s = rospy.Service('ik', ik, self.handle_ik_srv)

        rospy.Subscriber("Cmd_ABL_joy", Command_ABL, self.update_ABL)

        rospy.spin()

    def update_ABL(self, ABL):
        self.seg = ABL.segment
        x0 = [
                    self.seg[0].A*2, self.seg[0].B, 2*self.seg[0].L, 
                    self.seg[2].A*2, self.seg[2].B, 2*self.seg[2].L, 
                    self.seg[4].A*2, self.seg[4].B, 2*self.seg[4].L
                ]
        self.pts = self.position(x0)
        self.N, self.A = self.forwarding_orientation(x0)
        # print('update')
        # print(self.pts)

    def inverse_kinematic(self, pts, quat, seg, mode):
        def test_square(dst, a, n):  # a1 a2 a3 b1 b2 b3 r1 r2 r3
            def Pos2ABLD():
                x = dst[0]
                y = dst[1]
                z = dst[2]
                mod = x**2 + y**2

                alphaD, betaD, lengthD = 0,0,0
                if x==.0 and y==.0:
                    alphaD = 0
                    betaD = 0
                    lengthD = z
                else:
                    if mod <= z**2:
                        alphaD = asin(2*sqrt(mod)*z/(z**2+mod))
                        #print(self.alphaD)
                        if x == .0: #根据x的正负性质 确定beta atan输出值域(-90,90)
                            if y > .0:
                                betaD = -pi/2
                            else:
                                betaD = pi/2
                        elif x > 0:
                            betaD = atan(y/x)
                        elif x < 0:
                            betaD = atan(y/x) + pi
                        lengthD = (z**2+mod)/(2*sqrt(mod))*alphaD
                    elif mod >z**2:
                        betaD = atan(y / x)

                return [alphaD, betaD, lengthD]
            def single(x):
                a = float(x[0])
                b = float(x[1])
                l = float(x[2])
                if a != 0:
                    return [
                        l/a*(1-cos(a))*cos(b),
                        l/a*(1-cos(a))*sin(b),
                        l/a*sin(a)
                    ]
                else:
                    return [
                        0,
                        0,
                        l
                    ]
            def string_type(x):
                a1 = float(x[0])
                b1 = float(x[1])
                lm1 = float(x[2])
                a2 = float(x[3])
                b2 = float(x[4])
                lm2 = float(x[5])
                a3 = float(x[6])
                b3 = float(x[7])
                lm3 = float(x[8])

                if a1 == 0:
                    l1 = lm1
                else:
                    l1 = 2*lm1*a1/sin(a1/2)
                if a2 == 0:
                    l2 = lm2
                else:
                    l2 = 2*lm2*a2/sin(a2/2)
                if a3 == 0:
                    l3 = lm3
                else:
                    l3 = 2*lm3*a3/sin(a3/2)
                result = np.array(
                    [lm1 * sin(a1 / 2) * cos(b1) - (1 - cos(a1)) * (
                            lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                        b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                        b2) * cos(
                        a3 / 2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) + (
                                lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                            a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1)-dst[0],
                        lm1 * sin(a1 / 2) * sin(b1) - (1 - cos(a1)) * (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                            b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                            b2) * cos(a3 / 2)) + (
                                lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                            a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1)-dst[1],
                        lm1 * cos(a1 / 2) + (
                                    lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(
                                a2) * sin(
                                a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * cos(a1) - (
                                lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                            b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                            b2) * cos(a3 / 2)) * sin(a1) * sin(b1) - (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) * sin(a1) * cos(b1)-dst[2],
                        (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                            b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(
                            b1)) * cos(
                            a3) + (
                                (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3) - a[0],
                        (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                            b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                            a2)) * cos(
                            a3) + ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * sin(b1) * sin(b2)) * sin(a3) * sin(b3) - a[1],
                        (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                            a2)) * cos(
                            a3) + (
                                (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                            a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                                (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                                -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                            a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3) - a[2],
                        -(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                1 - cos(a2)) * (
                                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(
                            a2) * sin(b2) * cos(
                            b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                                (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(
                            b1)) * sin(
                            a3) * cos(b3) - n[0],
                        -(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                                -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                    1 - cos(a2)) * (
                                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                            b1) * cos(
                            b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                            a2)) * sin(
                            a3) * cos(b3) - n[1],
                        -(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                                -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(
                            a1)) * sin(
                            b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                                (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                            a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                                -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                            a1) * cos(
                            a2)) * sin(a3) * cos(b3) - n[2],
                        (l1-l2)/10,
                        (l2-l3)/10,
                        1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                    (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                            (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                        1/3*((b1-b2)**2+(b1-b3)**2+(b3-b2)**2)/200,
                        ]
                )
                
                return result.astype('float64')

             def string_type_onlyZ(x):
                a1 = float(x[0])
                b1 = float(x[1])
                lm1 = float(x[2])
                a2 = float(x[3])
                b2 = float(x[4])
                lm2 = float(x[5])
                a3 = float(x[6])
                b3 = float(x[7])
                lm3 = float(x[8])

                if a1 == 0:
                    l1 = lm1
                else:
                    l1 = 2*lm1*a1/sin(a1/2)
                if a2 == 0:
                    l2 = lm2
                else:
                    l2 = 2*lm2*a2/sin(a2/2)
                if a3 == 0:
                    l3 = lm3
                else:
                    l3 = 2*lm3*a3/sin(a3/2)
                result = np.array(
                    [lm1 * sin(a1 / 2) * cos(b1) - (1 - cos(a1)) * (
                            lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                        b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                        b2) * cos(
                        a3 / 2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) + (
                                lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                            a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1)-dst[0],
                        lm1 * sin(a1 / 2) * sin(b1) - (1 - cos(a1)) * (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                            b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                            b2) * cos(a3 / 2)) + (
                                lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                            a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1)-dst[1],
                        lm1 * cos(a1 / 2) + (
                                    lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(
                                a2) * sin(
                                a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * cos(a1) - (
                                lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                            b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                            b2) * cos(a3 / 2)) * sin(a1) * sin(b1) - (
                                lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                            b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                            a3 / 2) * cos(b2)) * sin(a1) * cos(b1)-dst[2],
                        (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                            b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(
                            b1)) * cos(
                            a3) + (
                                (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3) - a[0],
                        (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                            b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                            a2)) * cos(
                            a3) + ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                                -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                            a2) * sin(b1) * sin(b2)) * sin(a3) * sin(b3) - a[1],
                        (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                            a2)) * cos(
                            a3) + (
                                (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                            a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                                (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                                -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                            a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3) - a[2]
                        (l1-l2)/10,
                        (l2-l3)/10,
                        1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                    (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                            (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                        1/3*((b1-b2)**2+(b1-b3)**2+(b3-b2)**2)/200,
                        ]
                )
                
                return result.astype('float64')
            
            def tranformation_string(res):
                result = [0]*len(res)
                for i in range(int(len(res)/3)):
                    if 0 > res[3*i]:
                        result[3*i] = -res[3*i ]
                        result[3*i+1] = res[3*i+1] + pi
                    elif 0 < res[3*i]:
                        result[3*i] = res[3*i]  # a1
                        result[3*i+1] = res[3*i+1]  # b1
                    # lm1
                    result[3*i] = normalize_angle(result[3*i])
                    result[3*i+1] = normalize_angle(result[3*i+1])

                    result[3*i+2] = res[3*i+2] * res[3*i] / sin(res[3*i] / 2) / 2
                return result
            # now = time.time()
            
            re = Command_ABL()
            
            if mode.modeNumber == 3:
                res = Pos2ABLD()
                result = [
                    res[0], res[1], res[2]
                ]
                # FOR 3 SEG
                for i in range(1):
                    for j in range(1):
                        re.segment[2*i+j].A = result[3*i]
                        re.segment[2*i+j].B = result[3*i+1]
                        re.segment[2*i+j].L = result[3*i+2]
                for i in range(1,9):
                    re.segment[i].A = 0
                    re.segment[i].B = 0
                    re.segment[i].L = 0.055 

            elif mode.modeNumber == 4:
                # string type
                # initial value for 9 SEG
                x0 = [
                    self.seg[0].A*2, self.seg[0].B, 2*self.seg[0].L, 
                    self.seg[2].A*2, self.seg[2].B, 2*self.seg[2].L, 
                    self.seg[4].A*2, self.seg[4].B, 2*self.seg[4].L
                ]

                # print(x0)
               
                x0_rosenbrock = np.array(x0).astype('float64')
       
                res = least_squares(string_type, x0_rosenbrock,
                                    bounds=([-1.1*pi, -2*pi, 0.06, -1.1*pi, -2*pi, 0.06, -1.1*pi, -2*pi, 0.06],
                                            [1.1*pi, 2*pi, 0.16, 1.1*pi, 2*pi, 0.16, 1.1*pi, 2*pi, 0.16]))
                new = np.array([res.x[0], res.x[1], res.x[2],
                                res.x[3], res.x[4], res.x[5],
                                res.x[6], res.x[7], res.x[8]
                                ]).astype('float64')  # a1 b1 l1 a2 b2 l2 a3 b3 l3
                result = tranformation_string(new)
                # FOR 6 SEG
                for i in range(3):
                    for j in range(2):
                        re.segment[2*i+j].A = result[3*i]/2
                        re.segment[2*i+j].B = result[3*i+1]/2
                        re.segment[2*i+j].L = result[3*i+2]/2
                for i in range(6,9):
                    re.segment[i].A = 0
                    re.segment[i].B = 0
                    re.segment[i].L = 0.055
                self.seg = re.segment
                # Display the forward result #
                print('IK')
                print('result error',self.position(result)-pts)
                print('x0',x0)
                print('result',result)

                # Display the result directly
                # print(np.degrees(result[0]))
                # print(np.degrees(result[1]))
                # print(np.degrees(result[3]))
                # print(np.degrees(result[4]))
                # print(np.degrees(result[6]))
                # print(np.degrees(result[7]))
                # print(result[8])

            # print('time cost:', time.time() - now)
            # end
            return re

        # a1 a2 a3 b1 b2 b3 l1 l2 l3
        # print('z',pts.z)   

        # if not self.flag:
        #     pts = [pts.x, pts.y, pts.z]
        #     self.pts = pts
        #     quat = [quat.x, quat.y, quat.z, quat.w]

        #     # n, a = self.quat_transform(quat)
        #     n = self.N
        #     a = self.A
            
        #     self.desired = test_square(pts, a, n)
        #     self.flag = 1
        # elif self.flag:
        #     if (pts.x != self.pts[0]) or (pts.y != self.pts[1]) or (pts.z != self.pts[2]): 
        #         pts = [pts.x, pts.y, pts.z]
        #         self.pts = pts
        #         quat = [quat.x, quat.y, quat.z, quat.w]

        #         # n, a = self.quat_transform(quat)
        #         n = self.N
        #         a = self.A
                
        #         self.desired = test_square(pts, a, n)
        #         self.flag = 0
        #         return self.desired
        #     else:
        #         return self.desired

        # if pts.x != 0 or pts.y != 0 or pts.z != 0: 
        #     pts = [self.pts[0]+pts.x, self.pts[1]+pts.y, self.pts[2]+pts.z]
        #     self.pts = pts
        #     quat = [quat.x, quat.y, quat.z, quat.w]

        #     # n, a = self.quat_transform(quat)
        #     n = self.N
        #     a = self.A
            
        #     self.desired = test_square(pts, a, n)
        #     return self.desired
        # else:/
        #     return self.desired

        pts = [pts.x, pts.y, pts.z]
        
        quat = [quat.x, quat.y, quat.z, quat.w]

        # n, a = self.quat_transform(quat)
        n = self.N
        a = self.A
        
        self.desired = test_square(pts, a, n)
        return self.desired
   

    def quat_transform(self, qua): # alpha beta gamma
        R1 = Rotation.from_quat(qua).as_matrix()
        # N1 = [R1[0][0],R1[1][0],R1[2][0]]
        # A1 = [R1[0][2],R1[1][2],R1[2][2]]
        N1 = [1, 0, 0]
        A1 = [0, 0, 1]
        #print(R1)

        return N1, A1

    def position(self, x):
        # a1 = result[0]
        # a2 = result[3]
        # a3 = result[6]
        # b1 = result[1]
        # b2 = result[4]
        # b3 = result[7]
        # lm1 = result[2]
        # lm2 = result[5]
        # lm3 = result[8]
        a1 = float(x[0])
        b1 = float(x[1])
        lm1 = float(x[2])
        a2 = float(x[3])
        b2 = float(x[4])
        lm2 = float(x[5])
        a3 = float(x[6])
        b3 = float(x[7])
        lm3 = float(x[8])

        if -1e-4<a1 < 1e-4:
            l1 = lm1
        else:
            l1 = 2*lm1*a1/sin(a1/2)
        if -1e-4<a2 < 1e-4:
            l2 = lm2
        else:
            l2 = 2*lm2*a2/sin(a2/2)
        if -1e-4<a3 < 1e-4:
            l3 = lm3
        else:
            l3 = 2*lm3*a3/sin(a3/2)
        position = np.array(
            [lm1 * sin(a1 / 2) * cos(b1) - (1 - cos(a1)) * (
                    lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                b2) * cos(
                a3 / 2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                        lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                    b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                    a3 / 2) * cos(b2)) + (
                        lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                    a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1),
                lm1 * sin(a1 / 2) * sin(b1) - (1 - cos(a1)) * (
                        lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                    b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                    a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                        lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                    b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                    b2) * cos(a3 / 2)) + (
                        lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                    a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1),
                lm1 * cos(a1 / 2) + (
                            lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(
                        a2) * sin(
                        a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * cos(a1) - (
                        lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                    b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                    b2) * cos(a3 / 2)) * sin(a1) * sin(b1) - (
                        lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                    b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                    a3 / 2) * cos(b2)) * sin(a1) * cos(b1)])
        # position = np.array(
        #             [-(1 - cos(a1)) * (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(b1) * cos(b1) + (
        #                     -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) + (
        #                     -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(a2) * cos(
        #                 b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
        #                 a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * cos(b1) + lm1 * (1 - cos(a1)) * cos(b1) / a1,
        #             - (1 - cos(a1)) * (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(b1) * cos(b1) + (
        #                     -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) + (
        #                     -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(a2) * cos(
        #                 b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
        #                 a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * sin(b1) + lm1 * (1 - cos(a1)) * sin(b1) / a1,
        #             ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
        #                     -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(a2) * cos(
        #                 b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
        #                 a3) / a3 + lm2 * sin(a2) / a2) - (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(a1) * cos(b1) - (
        #                     -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
        #                         1 - cos(a3)) * (
        #                             -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
        #                 b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(a1) * sin(b1) + lm1 * sin(a1) / a1]
        #         )
        return position

    def forwarding_orientation(self, x):
        a1 = x[0]
        a2 = x[2]
        a3 = x[4]
        b1 = x[1]
        b2 = x[3]
        b3 = x[5]
        R1 = np.array(
            [[-(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                          (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                      a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                a3) * cos(b3),
              - (1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(a2) * cos(
                b1) * cos(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                          -(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                      b1)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                a3) * sin(b3),
              (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                          -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                  b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(a3) + (
                          (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                      a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3)],
             [-(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                    a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                          -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                      b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                    a3) * cos(b3),
              - (1 - cos(a3)) * (
                          -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                      b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                          (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                      a2) * sin(b1) * sin(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                    a3) * sin(b3),
              (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                          -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                  b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * cos(
                    a3) + ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                    a2) * sin(b1) * sin(b2)) * sin(a3) * sin(b3)],
             [-(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(
                    b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                          (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                      a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                          -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                      a2)) * sin(a3) * cos(b3),
              - (1 - cos(a3)) * (
                          (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                      a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(b3) * cos(b3) + (
                          -(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                          (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                      a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) - (
                          -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                      a2)) * sin(a3) * sin(b3),
              (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(a2)) * cos(
                    a3) + (
                          (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                      a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                          (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                      a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)]]
        )
        N1 = [R1[0][0],R1[1][0],R1[2][0]]
        A1 = [R1[0][2],R1[1][2],R1[2][2]]
        #N1 = [1, 0, 0]
        #A1 = [0, 0, 1]
        #print(R1)

        return N1, A1

if __name__ == '__main__':
    try:
        IK = ik_solver()
        print('IK is finished')
    except rospy.ServiceException as exc:
        print("IK call failed:"+str(exc))
