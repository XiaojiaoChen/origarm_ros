#!/usr/bin/env python3
#coding=utf-8

import numpy as np
import time
from math import cos, sin, sqrt, pi
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from angles import normalize_angle
import rospy
from extensa.srv import ik
from extensa.msg import *
import traceback

class softArm(object):
    def __init__(self, c1 =6 * 400 * 0.09 / 2.3e-3 / 2, lengthD=0.403,length=0.403
                 ,length0=0.403,alphaD=.0,betaD=.0,alpha=.0,beta=.0,actuator_type='small'):
        # meter radian
        self.initialBellowConfigurationAngle = [.0, pi/3, 2*pi/3, pi, -2*pi/3 ,-pi/3]

        self.bellowConfigurationAngleCos = np.array([.0]*6)
        self.bellowConfigurationAngleSin = np.array([.0]*6)

        self.pressureD = [.0]*6
        self.pressure  = [.0]*6
        self.pressureLimit_Upper = 210  #Kpa
        self.pressureLimit_Lower = -100
        if actuator_type == 'big':
            self.c1 = c1
            self.radR = 0.09
        elif actuator_type == 'small':
            self.c1 = c1
            self.radR = 0.0615

        self.angleULimit = pi
        self.angleDLimit = 0
        self.lengthULimit = 10
        self.lengthDLimit = 0
        self.posVector = np.array([.0, .0, .0])
        self.posVector_D = np.array([.0, .0, .0])
        self.speedVector = np.array([.0, .0, .0])
        self.angleVelocity = np.array([.0, .0, .0])

        self.lengthD = lengthD
        self.length = length
        self.length0 = length0

        self.alphaD = alphaD
        self.betaD = betaD
        self.alpha = alpha
        self.beta = beta
        self.flag = 0

        for i in range(6):
            self.bellowConfigurationAngleCos[i] = cos(self.initialBellowConfigurationAngle[i])
            self.bellowConfigurationAngleSin[i] = sin(self.initialBellowConfigurationAngle[i])

        self.ABLD2PD()

    def constriant(self, *args):
        if self.angleDLimit <= args[0] <= self.angleULimit\
            and self.lengthDLimit <= args[1] <= self.lengthULimit:
            return 1
        else:
            return 0

    def ABLD2PD(self):
        b1 = 2 * self.c1 * (self.lengthD - self.length0) / self.radR / 6
        btemp = self.c1 * self.alphaD / 6
        b2 = btemp * cos(self.betaD)
        b3 = 1.7320508 * btemp * sin(self.betaD)
        self.pressureD[0] = b1 + b2 * 2
        self.pressureD[1] = b1 + b2 + b3
        self.pressureD[2] = b1 - b2 + b3
        self.pressureD[3] = b1 - b2 * 2
        self.pressureD[4] = b1 - b2 - b3
        self.pressureD[5] = b1 + b2 - b3
        return self.pressureD

    def PD2ABLD(self):
        phycD = np.dot(self.pressureD,self.bellowConfigurationAngleCos)
        physD = np.dot(self.pressureD,self.bellowConfigurationAngleSin)
        self.alphaD = sqrt(phycD**2+physD**2)/self.c1
        self.betaD = atan2(physD,phycD)
        self.lengthD = (self.pressureD[0]+self.pressureD[1]+self.pressureD[2]+self.pressureD[3]+
                   self.pressureD[4]+self.pressureD[5])/self.c1/2*self.radR+self.length0
        return self.alphaD, self.betaD, self.lengthD

    def ABL2P(self):
        b1 = 2 * self.c1 * (self.length - self.length0) / self.radR / 6
        btemp = self.c1 * self.alpha / 6
        b2 = btemp * cos(self.beta)
        b3 = 1.7320508 * btemp * sin(self.beta)
        self.pressure[0] = b1 + b2 * 2
        self.pressure[1] = b1 + b2 + b3
        self.pressure[2] = b1 - b2 + b3
        self.pressure[3] = b1 - b2 * 2
        self.pressure[4] = b1 - b2 - b3
        self.pressure[5] = b1 + b2 - b3
        return self.pressure

    def P2ABL(self):
        phyc = np.dot(self.pressure,self.bellowConfigurationAngleCos)
        phys = np.dot(self.pressure,self.bellowConfigurationAngleSin)
        self.alpha = sqrt(phyc**2+phys**2)/self.c1
        self.beta = atan2(phys,phyc)
        self.length = (self.pressure[0]+self.pressure[1]+self.pressure[2]+self.pressure[3]+
                   self.pressure[4]+self.pressure[5])/self.c1/2*self.radR+self.length0
        return self.alpha, self.beta, self.length

    def Acturate(self):
        self.alpha += self.angleVelocity[0]
        self.beta += self.angleVelocity[1]
        self.length += self.angleVelocity[2]

        if self.alpha >= pi/2:
            self.alpha = pi/2
        if self.alpha <= 0:
            self.alpha = 0

    def SetW(self, alphaW=None, betaW=None, length=None): #设定alpha beta角速度
        if alphaW != None:
            self.angleVelocity[0] = alphaW
        if betaW != None:
            self.angleVelocity[1] = betaW
        if length != None:
            self.angleVelocity[2] = length

    def SetPos(self, X=None, Y=None, Z=None):
        if X != None:
            self.posVector[0] = X
        if Y != None:
            self.posVector[1] = Y
        if Z != None:
            self.posVector[2] = Z

    def SetPara(self, A=None, B=None, L=None):
        if A != None:
            self.alpha = A
        if B != None:
            self.beta = B
        if L != None:
            self.length = L

    def GetPara(self, mode=0): #供animated3Dplot 获取ABL
        self.Acturate()
        self.ABL2P()
        return self.alpha, self.beta, self.length

    def GetPressureD(self): #返回理想ABL 对应的pressure
        self.ABLD2PD()
        return self.pressureD

    def UpdateD(self, alpha, beta, length): #更新理想值数据
        if self.constriant(alpha, length):
            self.alpha = alpha
            self.beta = beta
            self.length = length
            self.ABL2P()
            return 1
        else:
            return 0
    def UpdateP(self, pre):
        for i in range(6):
            self.pressure[i] = pre[i]
        self.P2ABL()

class SoftObject(object):
    def __init__(self, *Arms):
        self.num = len(Arms)
        self.seg = dict()
        self.pts = dict()
        self.dst_pos = [0,0,0]
        self.dst_dir = [0,0,0]
        self.desired = 0
        for i in range(self.num):
            self.seg[i] = Arms[i]

    def inverse_kinematic(self, pts, quat, x0, single = 0):
        def test_square(dst, x0, a, n, R, single = 0):  # a1 a2 a3 b1 b2 b3 r1 r2 r3
            def single(x): # x y z
                a1 = float(x[0])
                b1 = float(x[1])
                l1 = float(x[2])

                result = np.array([
                    l1/a1*(1-cos(a1))*cos(b1) - pts[0],
                    l1/a1*(1-cos(a1))*sin(b1) - pts[1],
                    l1/a1*sin(a1) - pts[2],
                ])
                return result.astype('float64')
            def string_type(x):
                a1 = float(x[0])
                a2 = float(x[1])
                a3 = float(x[2])
                b1 = float(x[3])
                b2 = float(x[4])
                b3 = float(x[5])
                lm1 = float(x[6])
                lm2 = float(x[7])
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
                     1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                 (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                          (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                     1/3*((b1-b2)**2+(b1-b3)**2+(b3-b2)**2)/200,
                        (l1-l2),
                        (l2-l3)
                     ]
                )
                return result.astype('float64')

                result = [0]*9
                result[0] = new[0]
                result[1] = new[1]
                result[2] = new[6]

                result[3] = new[2]
                result[4] = new[3]
                result[5] = new[6]

                result[6] = new[4]
                result[7] = new[5]
                result[8] = new[6]
                return result
            
            
            def tranformation_string(res):
                result = [0]*len(res)
                for i in range(int(len(res)/3)):
                    if 0 > res[3*i]:
                        result[3*i] = -res[3*i]
                        result[3*i+1] = res[3*i+1] + pi
                    elif 0 < res[3*i]:
                        result[3*i] = res[3*i]  # a1
                        result[3*i+1] = res[3*i+1]  # b1
                    # lm1
                    result[3*i] = normalize_angle(result[3*i])
                    result[3*i+1] = normalize_angle(result[3*i+1])
 
                    result[3*i+2] = res[3*i+2] * res[3*i] / sin(res[3*i] / 2) / 2
                return result
            now = time.time()
            x0_rosenbrock = np.array(x0).astype('float64')
            # string type
            if single:
                res = least_squares(single, x0_rosenbrock,
                                bounds=([-pi, -2*pi, 0.0],
                                        [pi, 2*pi, 5]))
                print(res.x)
                result = np.array([
                    res.x[0], res.x[1], res.x[2]
                ])
            else:
                res = least_squares(string_type, x0_rosenbrock,
                                    bounds=([-pi, -pi, -pi, -2*pi, -2*pi, -2*pi, 0.05, 0.05, 0.05],
                                            [pi, pi, pi, 2*pi, 2*pi, 2*pi, 1, 1, 1]))
                new = np.array([res.x[0], res.x[3], res.x[6],
                                res.x[1], res.x[4], res.x[7],
                                res.x[2], res.x[5], res.x[8]
                                ]).astype('float64')  # a1 b1 l1 a2 b2 l2 a3 b3 l3
                result = tranformation_string(new)

            print('time cost:', time.time() - now)
            # end
            return result

        # a1 a2 a3 b1 b2 b3 l1 l2 l3
        pts = [pts.x, pts.y, pts.z]
        
        quat = [quat.x, quat.y, quat.z, quat.w]

        R = self.quat_transform(quat)
        n = [R[0][0],R[1][0],R[2][0]]
        a = [R[0][2],R[1][2],R[2][2]]
        # print(R)

        # normal type
        '''pos_now = [self.seg[0].alpha*3, self.seg[0].beta,
                    self.seg[3].alpha*3, self.seg[3].beta,
                    self.seg[6].alpha*3, self.seg[6].beta,
                    self.seg[6].length*3
                    ]'''
        # string type
        x0 = [
            x0[0].A+0.01, x0[0].B, x0[0].L
        ]
        # x0 = [ self.seg[0].alpha * 3, self.seg[0].beta,
        #             self.seg[3].alpha * 3, self.seg[3].beta,
        #             self.seg[6].alpha * 3, self.seg[6].beta,
        #             self.seg[0].length * 2 / self.seg[0].alpha * sin(self.seg[0].alpha / 6),
        #             self.seg[3].length * 2 / self.seg[3].alpha * sin(self.seg[3].alpha / 6),
        #             self.seg[6].length * 2 / self.seg[6].alpha * sin(self.seg[6].alpha / 6)
        #             ]
        self.desired = test_square(pts, x0, a, n, R, single)

        return self.desired

    def quat_transform(self, qua): # alpha beta gamma
        R1 = Rotation.from_quat(qua).as_matrix()
        return R1

    def outputPressure(self):
        1

class ik_solver:
    def __init__(self):
        x = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2]

        soft1 = softArm(alpha=x[0], beta=x[3], length=x[6], actuator_type='big')
        soft2 = softArm(alpha=x[1], beta=x[3], length=x[7], actuator_type='big')
        soft3 = softArm(alpha=x[2], beta=x[3], length=x[8], actuator_type='big')
        soft4 = softArm(alpha=x[0], beta=x[4], length=x[6])
        soft5 = softArm(alpha=x[1], beta=x[4], length=x[7])
        soft6 = softArm(alpha=x[2], beta=x[4], length=x[8])
        soft7 = softArm(alpha=x[0], beta=x[5], length=x[6])
        soft8 = softArm(alpha=x[1], beta=x[5], length=x[7])
        soft9 = softArm(alpha=x[2], beta=x[5], length=x[8])

        self.softArms = SoftObject(soft1, soft2, soft3, soft4, soft5, soft6, soft7, soft8, soft9)
        self.ik_srv_setup()

    def handle_ik_srv(self, req):
        result = self.softArms.inverse_kinematic(
            req.input.pose.position,
            req.input.pose.orientation,
            req.input.ABL.segment,
            1
        )

        re = Command_ABL()
        re.segment[0].A = result[0]
        re.segment[0].B = result[1]
        re.segment[0].L = result[2]
        # print(np.degrees(result[0]))
        # print(np.degrees(result[1]))
        # print(np.degrees(result[3]))
        # print(np.degrees(result[4]))
        # print(np.degrees(result[6]))
        # print(np.degrees(result[7]))
        # print(result[8])
        return re

    def ik_srv_setup(self):
        rospy.init_node('ik_srv')
        s = rospy.Service('ik', ik, self.handle_ik_srv)
        print('ready for ik service')
        rospy.spin()
      

if __name__ == '__main__':
    try:
        IK = ik_solver()
        print('IK is finished')
    except rospy.ServiceException as exc:
        print("IK call failed:"+str(exc))
