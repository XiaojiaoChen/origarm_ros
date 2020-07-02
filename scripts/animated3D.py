# -*- coding: utf-8 -*-
"""
    Animated 3D sinc function
"""
from PyQt5.QtCore import  pyqtSignal, QObject, QThread
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QWidget, QApplication
#from PyQt5 import QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from angles import normalize_angle
import numpy as np
from numpy import cos, sin, pi, degrees, arccos, sqrt
from math import acos, atan2
import time
from scipy import linalg
from softArm import softArm
from SoftObject import SoftObject
import sys
import threading
from scipy.spatial.transform import Rotation
from myJoyStick import myJoyStick
from origarm_ros.msg import *

try:
    import rospy
    ros_OK = 0
except:
    ros_OK = 0

class Signal(QObject):
    ValueSign = pyqtSignal()

class Thread(QThread):
    def __init__(self):
        super(Thread, self).__init__()
        self.ABL = Command_ABL()
        for i in range(9):
            self.ABL.segment[i].L = 0.055

        rospy.init_node('Display_Node', anonymous=True)
        rospy.Subscriber("Cmd_ABL_joy", Command_ABL, self.update_ABL)
        rospy.Subscriber("Cmd_ABL_ik", Command_ABL, self.update_ABL)
        self.rate = rospy.Rate(60) # 10hz

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def update_ABL(self, ABL):
        self.ABL = ABL

class Visualizer(QObject):
    def __init__(self, sf, ABL):
        super().__init__()
        #set parameters
        self.T = Thread()
        self.T.start()
        self.ABL = ABL
        self.joystick = myJoyStick()
        self.Sign = Signal()
        self.Arms = sf
        self.soft = sf.getArms()
        self.w = gl.GLViewWidget()
        self.w.pan(0,0,-17)
        self.w.opts['distance'] = 40
        self.w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
        self.flag = 0

        # create the background grids
        gx = gl.GLGridItem()
        gx.rotate(90, 0, 1, 0)
        gx.translate(-10, 0, -10)
        self.w.addItem(gx)
        gy = gl.GLGridItem()
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -10, -10)
        self.w.addItem(gy)
        gz = gl.GLGridItem()
        gz.translate(0, 0, 0)
        self.w.addItem(gz)
        self.w.setGeometry(20, 20, 900, 900)

        self.pts = dict()

        self.flag1 = 0
        self.display = 1
        self.multi = 20

        self.dst_pos = [5, 5, 5]
        self.dst_dir = [0, 1, 0]

        self.traces = dict()
        self.incre_alpha = dict()
        self.incre_beta = dict()
        self.incre_length = dict()
        self.pointer = dict()

        self.normal = [.0, .0, .0]
        self.angle = dict()

        self.incre = 0

        self.num_seg = sf.num
        self.num_pipe = 3
        self.calculate_flag = {0:0, 1:0, 2:0, 3:0, 4:0, 5:0, 6:0, 7:0, 8:0}
        self.angle = {0:[0, 1, 0],1:[0, 1, 0],2:[0, 1, 0]}
        self.vector_display([1,0,0], [0,0,0], 7, 2, 1)
        self.vector_display([0,-1,0], [0,0,0], 8, 2, 2)
        self.vector_display([0,0,-1], [0,0,0], 9, 2, 3)

        for i in range(3):
            for j in range(3):
                self.traces[3*i+j] = gl.GLLinePlotItem(color=pg.glColor((40+30*(i+1), 100+10*(i+1))),
                                                width=2, antialias=True)
                self.w.addItem(self.traces[3*i+j])
        self.circle_show = 1
        if self.circle_show:
            self.create_circle()
        self.alpha = dict()
        self.beta = dict()
        self.lm = dict()

        self.nx = {0:[1, 0, 0]}
        self.nz = {0:[0, 0, 1]}
        self.n = [0,0,0]
        self.a = [0,0,0]
        self.o = [0,0,0]
        self.start_3 = 0
        self.oz = 0
        self.euler_beta = 0
        self.euler_alpha = 0
        self.euler_gamma = 0

        self.update()

        # using under ROS

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QApplication.instance().exec_()
            self.alpha = self.alpha

    def update(self):
        self.ROS()
        for i in range(self.num_seg):
            if self.num_seg == 9:
                if i in [0,1,2]:
                    self.soft[i].SetW(alphaW = self.joystick.Y*pi/30, betaW = self.joystick.X*pi/10,
                                      length = self.joystick.Z)
                if self.joystick.Y or self.joystick.X or self.joystick.Z:
                    self.Sign.ValueSign.emit()
                if i in [3,4,5]:
                    self.soft[i].SetW(alphaW=self.joystick.RY*pi/30, betaW=self.joystick.RX*pi/10,
                                      length=self.joystick.TZ)
                if self.joystick.RY or self.joystick.RX or self.joystick.TZ:
                    self.Sign.ValueSign.emit()
            self.alpha[i], self.beta[i], self.lm[i] = self.soft[i].GetPara(self.joystick.flag)

        self.transfer_line()

        # self.orientation()
        # self.vector_display(self.n, self.pts[8][18], 1, 1.5, 1)
        # self.vector_display(self.o, self.pts[8][18], 2, 1.5, 2)
        # self.vector_display(self.a, self.pts[8][18], 3, 1.5, 3)
        '''self.vector_display(self.pts[2][18], self.pts[0][0], 4, 1, 1)
        self.vector_display(self.pts[5][18]-self.pts[2][18], self.pts[2][18], 5, 1, 2)
        self.vector_display(self.pts[8][18]-self.pts[5][18], self.pts[5][18], 6, 1, 3)'''

        if self.incre:
            self.Sign.ValueSign.emit()
            self.incre -= 1

    def create_Line(self, seg, alpha, beta, lm): # 扩展接口：
        x = np.array([.0]*19)
        z = np.array([.0]*19)
        if not alpha:
            lm = np.linspace(0, lm, 19)
            for i in range(19):
                z[i] = lm[i]
                x[i] = 0
        else:
            theta = np.linspace(0, alpha, 19)
            for i in range(19):
                # 根据alpha lm计算y=0 平面上图形
                x[i] = lm/alpha*(1-cos(theta[i]))
                z[i] = lm/alpha*sin(theta[i])
        y = np.array([0]*19)
        transB = np.array([[cos(beta), sin(beta), 0],
                  [-sin(beta), cos(beta), 0],
                  [0, 0, 1]])
        self.pts[seg] = np.vstack([x, y, z]).transpose().dot(transB)
        self.nx[seg+1] = [cos(beta)*cos(alpha), sin(beta)*cos(alpha), -sin(alpha)]
        self.nz[seg+1] = [cos(beta)*sin(alpha), sin(beta)*sin(alpha), cos(alpha)]

    def orientation(self):
        a1 = self.alpha[0] * 3
        a2 = self.alpha[3] * 3
        a3 = self.alpha[6] * 3
        b1 = self.beta[0]
        b2 = self.beta[3]
        b3 = self.beta[6]

        R = np.array(
            [[-(1 - cos(a3)) * (
                        -(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                        -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                    b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                      (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                  a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                a3) * cos(b3),
              - (1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                  a2) * cos(
                  b1) * cos(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                      -(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                  b1)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                  a3) * sin(b3),
              (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                  b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(
                  a3) + (
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
              (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                  a2)) * cos(
                  a3) + (
                      (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                  a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                      (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                  a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)]]
        )
        self.n = [R[0][0], -R[1][0], -R[2][0]]
        self.o = [R[0][1], -R[1][1], -R[2][1]]
        self.a = [R[0][2], -R[1][2], -R[2][2]]
        #self.euler_alpha, self.euler_beta, self.euler_gamma = self.Arms.desired_orientation_euler()
        self.euler_beta = atan2(sqrt(float(R[2][0] ** 2 + R[2][1] ** 2)), R[2][2])
        if -pi/180<self.euler_beta<pi/180:
            self.euler_beta=0
        if sin(self.euler_beta) != 0 :
            self.euler_alpha = atan2(R[1][2] / sin(self.euler_beta), R[0][2] / sin(self.euler_beta ))
            self.euler_gamma = atan2(R[2][1] / sin(self.euler_beta), -R[2][0] / sin(self.euler_beta ))
        elif self.euler_beta == 0:
            self.euler_alpha = 0
            self.euler_gamma = atan2(-R[0][1], R[0][0])
        elif 99*pi/100<self.euler_beta <= pi:
            self.euler_gamma = atan(R[0][1], -R[0][0])
            self.euler_alpha = 0

    def create_circle(self):
        def create(L, r):
            num_res = 120
            pos3 = np.zeros((num_res, num_res, 3))
            pos3[:, :, :2] = np.mgrid[:num_res, :num_res].transpose(1, 2, 0) * [-0.1, 0.1]
            pos3 = pos3.reshape(num_res**2, 3)
            d3 = (pos3 ** 2).sum(axis=1) ** 0.5
            area = L #产生备选点的区域
            ring_res = 0.08 #环粗细
            for i in range(num_res):
                pos3[i * num_res:num_res * (i + 1), 0] = -area + 2*area*i/num_res
                pos3[i * num_res:num_res * (i + 1), 1] = np.linspace(-area, area, num_res)
                pos3[i * num_res:num_res * (i + 1), 2] = 0
            count = 0
            list1 = list()
            rad_ring = r #环圆心距离
            ring = 0.029*10 #环半径
            for i in range(num_res**2):
                if  (ring - ring_res) ** 2 < ((pos3[i, 1]) ** 2 + (pos3[i, 0]-rad_ring) ** 2 )< ring**2  or\
                    (ring - ring_res) ** 2 < ((pos3[i, 1]+rad_ring*0.866) ** 2 + (pos3[i, 0]-rad_ring/2) ** 2)<ring**2 or\
                    (ring - ring_res) ** 2 < ((pos3[i, 1]+rad_ring*0.866) ** 2 + (pos3[i, 0]+rad_ring/2) ** 2)< ring**2  or\
                    (ring - ring_res) ** 2 < ((pos3[i, 1]-rad_ring*0.866) ** 2 + (pos3[i, 0]-rad_ring/2) ** 2)< ring**2  or\
                    (ring - ring_res) ** 2 < ((pos3[i, 1]-rad_ring*0.866) ** 2 + (pos3[i, 0]+rad_ring/2) ** 2)< ring**2  or\
                    (ring - ring_res) ** 2 < ((pos3[i, 1]) ** 2 + (pos3[i, 0]+rad_ring) ** 2)< ring**2  :
                    list1.append(i)
            backup = list()
            for i in list1:
                backup.append(pos3[i])
            return backup

        self.backup = create(L = 2, r = 0.09*10)
        self.backup1 = create(L = 2, r = 0.0615*10)
        self.sp = list()

        color = {0:pg.glColor(40,20,5),1:pg.glColor(40,20,5),2:pg.glColor(40,20,5),3:pg.glColor(40,40,0),4:pg.glColor(40,40,0),5:pg.glColor(40,40,0),6:pg.glColor(0,40,40),7:pg.glColor(0,40,40),8:pg.glColor(0,40,40)}
        for i in range(self.num_seg):
            for j in range(self.num_pipe*i, self.num_pipe*(i+1)):
                if i <= 2:
                    self.sp.append(gl.GLScatterPlotItem(pos=self.backup, size=0.08, pxMode=False, color = color[i]))
                else:
                    self.sp.append(gl.GLScatterPlotItem(pos=self.backup1, size=0.08, pxMode=False, color = color[i]))
                self.w.addItem(self.sp[j])

    def vector_display(self, vector, pos, num, multipy=0, rgb=0):
        color = [0,pg.glColor(255,0,0),pg.glColor(0,255,0),pg.glColor(0,0,255)]
        if not num in self.pointer.keys():
            if not rgb:
                self.pointer[num] = gl.GLLinePlotItem(color=pg.glColor((40*num, 50)),
                                                width=2, antialias=True)
            else:
                self.pointer[num] = gl.GLLinePlotItem(color=color[rgb],
                                                width=2, antialias=True)
            self.w.addItem(self.pointer[num])
        length = 1
        if multipy:
            length = multipy
        x = np.linspace(0, float(vector[0])*length, 10)
        y = np.linspace(0, float(vector[1])*length, 10)
        z = np.linspace(0, float(vector[2])*length, 10)
        pts = np.vstack([x, y, z]).transpose() + pos
        self.pointer[num].setData(pos=pts)

    def update_circle(self, seg):
        #for seg in range(self.num_seg):
        vector1 = np.subtract(self.pts[seg][1], self.pts[seg][0])
        vector2 = np.subtract(self.pts[seg][2], self.pts[seg][0])

        result = -np.cross(vector1, vector2)
        # 化为单位向量
        mod = np.sqrt(np.square(result[0])+np.square(result[1])+np.square(result[2]))
        if mod:
            result = np.divide(result, mod)
        # 旋转轴
        if not seg:
            #母本
            data = self.backup
        elif seg<=2:
            data = np.subtract(self.sp[self.num_pipe*seg-1].pos, self.pts[seg-1][18])
        elif seg == 3:
            data = np.dot(self.backup1, self.spin)
        elif seg > 3:
            data = np.subtract(self.sp[self.num_pipe * seg - 1].pos, self.pts[seg - 1][18])
        for i in range(self.num_pipe):
            spin = -np.array(linalg.expm(np.multiply((i+1)*self.alpha[seg]/self.num_pipe, self.hat(result))))
            if seg==2 and i==2:
                self.spin = -np.array(linalg.expm(np.multiply((i+1)*self.alpha[seg], self.hat(result))))
            self.sp[i+self.num_pipe*seg].setData(pos=np.add(np.dot(data, spin), self.pts[seg][6*(i+1)][:]))

    def hat(self, vector):
        hat = np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])
        return hat

    def transfer(self) :
        # 每个seg 两次旋转
        for seg in range(1, self.num_seg):
            #print(seg)
            angle_x = acos(np.dot(self.nx[seg], self.nx[0]))
            #前一个节点的x轴和后一个节点的x轴叉乘
            axis_x = np.cross(self.nx[seg], self.nx[0])
            mod = np.sqrt(axis_x[0]**2+axis_x[1]**2+axis_x[2]**2)
            if mod:
                axis_x = np.divide(axis_x, mod)
            spin_x = np.array(linalg.expm(np.multiply(angle_x, self.hat(axis_x))))

            nz = np.dot(self.nz[0], spin_x)

            angle_z = arccos(np.clip(np.dot(nz, self.nz[seg]), -1.0, 1.0))
            #对比旋转后结果 不符合即反转
            right = 1
            while right:
                spin_z = np.array(linalg.expm(np.multiply(angle_z, self.hat(self.nx[seg]))))
                check = np.dot(nz, spin_z) - self.nz[seg]
                if -0.005<check[0] <0.005 and -0.005<check[1] <0.005 and -0.005<check[2] <0.005:
                    right = 0
                else:
                    angle_z = -angle_z

            self.pts[seg] = np.dot(np.dot(self.pts[seg], spin_x), spin_z)
            self.pts[seg] += self.pts[seg-1][18]

            self.nx[seg+1] = np.dot(np.dot(self.nx[seg+1], spin_x), spin_z)
            self.nz[seg+1] = np.dot(np.dot(self.nz[seg+1], spin_x), spin_z)

            self.angle[seg] = self.nz[seg+1]

        for i in range(self.num_seg):
            for j in range(19):  # 翻转z坐标y坐标
                self.pts[i][j][1] = -self.pts[i][j][1]
                self.pts[i][j][2] = -self.pts[i][j][2]
            self.traces[i].setData(pos=self.pts[i])
        if self.circle_show:
            self.update_circle()
    # 基于向量

    def transfer_line(self):
        def transA(alpha):
            result = np.array(
                [[cos(alpha), 0, -sin(alpha)],
                [0, 1, 0],
                [sin(alpha), 0, cos(alpha)]]
                            )
            return result
        def transB(beta):
            result = np.array(
                [[cos(beta), sin(beta), 0],
                 [-sin(beta), cos(beta), 0],
                 [0, 0, 1]]
            )
            return result
        def transA_(alpha, base):
            result = np.array(
                [[cos(alpha), 0, -sin(alpha), 0],
                [0, 1, 0, 0],
                [sin(alpha), 0, cos(alpha), 0],
                [base[0], base[1], base[2], 1]]
                            )
            return result
        def transB_(beta):
            result = np.array(
                [[cos(beta), sin(beta), 0, 0],
                 [-sin(beta), cos(beta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
            )
            return result
        def hat(vector, theta):
            trans = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            hat = np.array([
                [0, -vector[2], vector[1]],
                [vector[2], 0, -vector[0]],
                [-vector[1], vector[0], 0]
            ])
            result = trans + hat * sin(theta) + (1 - cos(theta)) * hat.dot(hat)
            result[2][2] = cos(theta)
            return result

        transform = np.eye(4)

        a = dict()
        b = dict()
        multi = self.multi
        for i in range(self.num_seg):
            a[i] = self.alpha[i]
        for i in range(self.num_seg):
            b[i] = self.beta[i]
        for i in range(self.num_seg):
            self.create_Line(i, self.alpha[i], b[i], self.lm[i] * multi)

        for i in range(1, self.num_seg):
            for j in reversed(range(i)):
                transform = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]). \
                    dot(hat(np.array([-sin(b[j]), cos(b[j]), 0]), a[j])).transpose()
                if self.alpha[j] == 0:
                    base = [
                        0, 0,
                        self.lm[j] * multi
                    ]
                else:
                    base = np.array([self.lm[j] / self.alpha[j] * (1 - cos(self.alpha[j])) * multi,
                            0,
                            self.lm[j] / self.alpha[j] * sin(self.alpha[j]) * multi
                            ]).dot(transB(b[j]))
                self.pts[i] = self.pts[i].dot(transform)
                self.pts[i] += base

        for seg in range(self.num_seg):
            for j in range(19):
                self.pts[seg][j][1] = -self.pts[seg][j][1]
                self.pts[seg][j][2] = -self.pts[seg][j][2]
            if self.circle_show:
                self.update_circle(seg)
            self.traces[seg].setData(pos=self.pts[seg])
    # 基于方程迭代

    def move(self):
        if self.incre:
            for i in range(3):
                for j in range(3):
                    self.soft[3*i+j].alpha += self.incre_alpha[i]
                    self.soft[3*i+j].beta += self.incre_beta[i]
                    self.soft[3*i+j].length += self.incre_length[i]
    
    def ROS(self):
        for i in range(3):
            if self.T.ABL.segment[2*i].A < 0:
                for j in range(3):
                    self.soft[3*i+j].alpha = -self.T.ABL.segment[2*i].A*2/3
                    self.soft[3*i+j].beta = normalize_angle(self.T.ABL.segment[2*i].B + pi)
                    self.soft[3*i+j].length = self.T.ABL.segment[2*i].L*2/3
            elif self.T.ABL.segment[2*i].A > 0:
                for j in range(3):
                    self.soft[3*i+j].alpha = self.T.ABL.segment[2*i].A*2/3
                    self.soft[3*i+j].beta = self.T.ABL.segment[2*i].B
                    self.soft[3*i+j].length = self.T.ABL.segment[2*i].L*2/3
            elif self.T.ABL.segment[2*i].A == 0:
                for j in range(3):
                    self.soft[3*i+j].alpha = 0
                    self.soft[3*i+j].beta = 0
                    self.soft[3*i+j].length = self.T.ABL.segment[2*i].L*2/3
        self.Sign.ValueSign.emit()

    def inverse_kinematic(self, pts=[0,0,0], n=[0,0,0], a=[0,0,0], euler=0, model=1, input=[],length=2):
        result = self.Arms.inverse_kinematic(pts=pts, n=n, a=a, euler=euler, model=model, input=input,length=length)

        self.incre = 1

        # # 3X3 控制模式
        for i in range(3):
            incre_alpha = result[3*i] - self.alpha[3 * i] * 3
            incre_beta = result[1 + 3*i] - self.beta[3 * i]
            incre_length = result[2 + 3*i] - self.lm[3 * i] * 3
            self.incre_alpha[i] = incre_alpha / self.incre / 3
            self.incre_beta[i] = incre_beta / self.incre
            self.incre_length[i] = incre_length / self.incre / 3

        return 0

    def animation(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(40)


class parameter:
    def __init__(self):
        self.alpha = 0
        self.beta = 0
        self.length = 0

# Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    x = [pi/18, pi/18, pi/18, 0, 0, 0, 0.4, 0.4, 0.4]
    soft1 = softArm(alphaD=x[0], betaD=x[3], lengthD=x[6])
    soft2 = softArm(alphaD=x[1], betaD=x[4], lengthD=x[7])
    soft3 = softArm(alphaD=x[2], betaD=x[5], lengthD=x[8])
    soft4 = softArm(alphaD=x[0], betaD=x[3], lengthD=x[6])
    soft5 = softArm(alphaD=x[1], betaD=x[4], lengthD=x[7])
    soft6 = softArm(alphaD=x[2], betaD=x[5], lengthD=x[8])
    soft7 = softArm(alphaD=x[0], betaD=x[3], lengthD=x[6])
    soft8 = softArm(alphaD=x[1], betaD=x[4], lengthD=x[7])
    soft9 = softArm(alphaD=x[2], betaD=x[5], lengthD=x[8])

    softArms = SoftObject(soft1, soft2, soft3, soft4, soft5, soft6, soft7, soft8, soft9)
    app = QApplication(sys.argv)
    test =  Visualizer(softArms)
    test.w.show()
    test.animation()
    pts = [1, 1, 1]
    n = [0, 0, 1]
    a = [1, 0, 0]
    test.Arms.inverse_kinematic(pts, n, a)

    sys.exit(app.exec_())


