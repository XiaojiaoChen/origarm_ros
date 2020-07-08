from softArm import softArm
import numpy as np
from math import cos, sin, atan2, sqrt, pi
from scipy.optimize import least_squares, differential_evolution, brute, basinhopping, shgo, dual_annealing
from scipy.spatial.transform import Rotation
from angles import normalize_angle
import time
import traceback

class SoftObject(object):
    def __init__(self, *Arms, type = '3X3'):
        self.num = len(Arms)
        self.seg = dict()
        self.pts = dict()
        self.dst_pos = [0,0,0]
        self.dst_dir = [0,0,0]
        self.desired = 0
        for i in range(self.num):
            self.seg[i] = Arms[i]

    def getArms(self):
        return self.seg

    def inverse_kinematic(self, pts=[0, 3, 5], n=[0, 1, 0], a=[0, 0, 1], euler=[pi/2, pi/2, -pi/2], model=1, input=[], length=2):
        def test_square(dst, x0, a, n, R):  # a1 a2 a3 b1 b2 b3 r1 r2 r3
            def test_3_7dofs(x):
                a1 = float(x[0])
                b1 = float(x[1])
                a2 = float(x[2])
                b2 = float(x[3])
                a3 = float(x[4])
                b3 = float(x[5])
                lm1 = float(x[6])

                result = np.array([
                    (-((lm1 * (1 - cos(a3)) * cos(a2) * cos(b3 - b2) / a3 + lm1 * sin(a2) * sin(a3) / a3 + lm1 * (
                            1 - cos(a2)) / a2) * sin(b2 - b1) + lm1 * (1 - cos(a3)) * sin(b3 - b2) * cos(
                        b2 - b1) / a3) * sin(
                        b1) + (((lm1 * (1 - cos(a3)) * cos(a2) * cos(b3 - b2) / a3 + lm1 * sin(a2) * sin(
                        a3) / a3 + lm1 * (1 - cos(a2)) / a2) * cos(b2 - b1) - lm1 * (1 - cos(a3)) * sin(b2 - b1) * sin(
                        b3 - b2) / a3) * cos(a1) + (
                                       -lm1 * (1 - cos(a3)) * sin(a2) * cos(b3 - b2) / a3 + lm1 * sin(a3) * cos(
                                   a2) / a3 + lm1 * sin(a2) / a2) * sin(a1) + lm1 * (1 - cos(a1)) / a1) * cos(b1)) - dst[
                        0],
                   (((lm1 * (1 - cos(a3)) * cos(a2) * cos(b3 - b2) / a3 + lm1 * sin(a2) * sin(a3) / a3 + lm1 * (
                            1 - cos(a2)) / a2) * sin(b2 - b1) + lm1 * (1 - cos(a3)) * sin(b3 - b2) * cos(
                        b2 - b1) / a3) * cos(
                        b1) + (((lm1 * (1 - cos(a3)) * cos(a2) * cos(b3 - b2) / a3 + lm1 * sin(a2) * sin(
                        a3) / a3 + lm1 * (1 - cos(a2)) / a2) * cos(b2 - b1) - lm1 * (1 - cos(a3)) * sin(b2 - b1) * sin(
                        b3 - b2) / a3) * cos(a1) + (
                                       -lm1 * (1 - cos(a3)) * sin(a2) * cos(b3 - b2) / a3 + lm1 * sin(a3) * cos(
                                   a2) / a3 + lm1 * sin(a2) / a2) * sin(a1) + lm1 * (1 - cos(a1)) / a1) * sin(b1)) - dst[
                        1],
                    (- ((lm1 * (1 - cos(a3)) * cos(a2) * cos(b3 - b2) / a3 + lm1 * sin(a2) * sin(a3) / a3 + lm1 * (
                            1 - cos(a2)) / a2) * cos(b2 - b1) - lm1 * (1 - cos(a3)) * sin(b2 - b1) * sin(
                        b3 - b2) / a3) * sin(
                        a1) + (-lm1 * (1 - cos(a3)) * sin(a2) * cos(b3 - b2) / a3 + lm1 * sin(a3) * cos(
                        a2) / a3 + lm1 * sin(a2) / a2) * cos(a1) + lm1 * sin(a1) / a1) - dst[2],
                (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                        b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(
                        a3) + (
                            (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3) - a[0],
                    (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * cos(
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
                        a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)- a[2],
                    -(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                1 - cos(a2)) * (
                                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(
                        a2) * sin(b2) * cos(
                        b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                        a3) * cos(b3) - n[0],
                    -(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                        a3) * cos(b3) - n[1],
                    -(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(
                        b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                        a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                            -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                        a1) * cos(
                        a2)) * sin(a3) * cos(b3)- n[2],
                    1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                            (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                     (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                    1 / 3 * ((b1 - b2) ** 2 + (b1 - b3) ** 2 + (b3 - b2) ** 2) / 200,
                ])
                return result.astype('float64')
            def test_3_7dofs_dir(x):
                a1 = float(x[0])
                a2 = float(x[1])
                a3 = float(x[2])
                b1 = float(x[3])
                b2 = float(x[4])
                b3 = float(x[5])
                lm1 = float(x[6])
                result = np.array([
                    2-(-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(b2) + sin(
                        a2) * sin(
                        b2) * cos(a3 / 2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                            -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(b2) + sin(
                        a2) * cos(a3 / 2) * cos(b2)) + (
                            -sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(
                        b3) + cos(
                        a2 / 2) + cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1) + sin(a1 / 2) * cos(b1))/dst[0],
                    2-(- (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(b2) + sin(
                        a2) * cos(
                        a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                            -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(b2) + sin(
                        a2) * sin(b2) * cos(a3 / 2)) + (
                            -sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(
                        b3) + cos(
                        a2 / 2) + cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1) + sin(a1 / 2) * sin(b1))/dst[1] ,
                    2-((-sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(b3) + cos(
                        a2 / 2) + cos(
                        a2) * cos(a3 / 2)) * cos(a1) - (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(b2) + sin(
                        a2) * cos(
                        a3 / 2) * cos(b2)) * sin(a1) * cos(b1) - (
                            -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(
                        b2) + sin(a2) * sin(
                        b2) * cos(a3 / 2)) * sin(a1) * sin(b1) + cos(a1 / 2))/dst[2] ,
                (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                        b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(
                        a3) + (
                            (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3) - a[0],
                    (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * cos(
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
                        a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)- a[2],
                    -(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                1 - cos(a2)) * (
                                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(
                        a2) * sin(b2) * cos(
                        b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                        a3) * cos(b3) - n[0],
                    -(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                        a3) * cos(b3) - n[1],
                    -(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(
                        b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                        a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                            -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                        a1) * cos(
                        a2)) * sin(a3) * cos(b3)- n[2]
                ])
                return result.astype('float64')
            def test_9seg_string(x):
                a1 = float(x[0])
                a2 = float(x[1])
                a3 = float(x[2])
                b1 = float(x[3])
                b2 = float(x[4])
                b3 = float(x[5])
                lm1 = float(x[6])
                result = np.array([
                    2 - (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(b2) + sin(
                        a2) * sin(
                        b2) * cos(a3 / 2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                                 -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(
                             b2) + sin(
                             a2) * cos(a3 / 2) * cos(b2)) + (
                                 -sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(
                             b3) + cos(
                             a2 / 2) + cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1) + sin(a1 / 2) * cos(b1)) / dst[0],
                    2 - (- (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(b2) + sin(
                        a2) * cos(
                        a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                 -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                                 -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(
                             b2) + sin(
                             a2) * sin(b2) * cos(a3 / 2)) + (
                                 -sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(
                             b3) + cos(
                             a2 / 2) + cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1) + sin(a1 / 2) * sin(b1)) / dst[1],
                    2 - ((-sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - sin(a2) * sin(a3 / 2) * cos(b2) * cos(b3) + cos(
                        a2 / 2) + cos(
                        a2) * cos(a3 / 2)) * cos(a1) - (-(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + sin(a2 / 2) * cos(b2) + sin(
                        a2) * cos(
                        a3 / 2) * cos(b2)) * sin(a1) * cos(b1) - (
                                 -(1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(b3) + (
                                 -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + sin(a2 / 2) * sin(
                             b2) + sin(a2) * sin(
                             b2) * cos(a3 / 2)) * sin(a1) * sin(b1) + cos(a1 / 2)) / dst[2],
                    (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                        b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(
                        a3) + (
                            (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3) - a[0],
                    (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * cos(
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
                            -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
                        a3) * cos(b3) - n[0],
                    -(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                        a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                        b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                            -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                        a3) * cos(b3) - n[1],
                    -(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(
                        b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                            (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                        a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                            -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                        a1) * cos(
                        a2)) * sin(a3) * cos(b3) - n[2]
                ])
                return result.astype('float64')
            def Global(x):
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
                        (l1 - l2)/10,
                        (l2-l3)/10,
                        l1 /200,
                     1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                             (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                      (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                     1 / 3 * ((b1 - b2) ** 2 + (b1 - b3) ** 2 + (b3 - b2) ** 2) / 200,
                     ]
                ).astype('float64')
                re = 0
                for i in range(len(result)):
                    re += np.abs(result[i])
                return re
            def string_type_onlyZ(x):
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
                    l1 = 2 * lm1 * a1 / sin(a1 / 2)
                if a2 == 0:
                    l2 = lm2
                else:
                    l2 = 2 * lm2 * a2 / sin(a2 / 2)
                if a3 == 0:
                    l3 = lm3
                else:
                    l3 = 2 * lm3 * a3 / sin(a3 / 2)
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
                         a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * cos(b1) - dst[0],
                     lm1 * sin(a1 / 2) * sin(b1) - (1 - cos(a1)) * (
                             lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                         b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                         a3 / 2) * cos(b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                             lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                         b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                         b2) * cos(a3 / 2)) + (
                             lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(a2) * sin(
                         a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * sin(a1) * sin(b1) - dst[1],
                     lm1 * cos(a1 / 2) + (
                             lm2 * cos(a2 / 2) - lm3 * sin(a2) * sin(a3 / 2) * sin(b2) * sin(b3) - lm3 * sin(
                         a2) * sin(
                         a3 / 2) * cos(b2) * cos(b3) + lm3 * cos(a2) * cos(a3 / 2)) * cos(a1) - (
                             lm2 * sin(a2 / 2) * sin(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * cos(b2) * cos(
                         b3) + lm3 * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3 / 2) * sin(b3) + lm3 * sin(a2) * sin(
                         b2) * cos(a3 / 2)) * sin(a1) * sin(b1) - (
                             lm2 * sin(a2 / 2) * cos(b2) - lm3 * (1 - cos(a2)) * sin(a3 / 2) * sin(b2) * sin(b3) * cos(
                         b2) + lm3 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3 / 2) * cos(b3) + lm3 * sin(a2) * cos(
                         a3 / 2) * cos(b2)) * sin(a1) * cos(b1) - dst[2],
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
                     (l1 - l2) / 10,
                     (l2 - l3) / 10,
                     1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                             (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                                      (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                     1 / 3 * ((b1 - b2) ** 2 + (b1 - b3) ** 2 + (b3 - b2) ** 2) / 200
                     ]
                )
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
                     (l1 - l2) / 10,
                     (l2 - l3) / 10,
                     (a1-a2)**2/200+
                     (a1-a3)**2/200+
                     (a2-a3)**2/200,
                     1 / 3 * (((2 * a1 ) / (pi * 2 / 4)) ** 2 + (
                             (2 * a2 ) / (pi * 2 / 4)) ** 2 + (
                                      (2 * a3 ) / (pi * 2 / 4)) ** 2) / 200,
                     1 / 3 * ((b1 - b2) ** 2 + (b1 - b3) ** 2 + (b3 - b2) ** 2) / 200,
                     ]
                )
                # 1 / 3 * (((2 * a1 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                #         (2 * a2 - pi * 2 / 4) / (pi * 2 / 4)) ** 2 + (
                #                  (2 * a3 - pi * 2 / 4) / (pi * 2 / 4)) ** 2) / 200,
                # 1 / 3 * ((b1 - b2) ** 2 + (b1 - b3) ** 2 + (b3 - b2) ** 2) / 200,
                return result.astype('float64')
            def test_3_9dofs(x):
                a1 = float(x[0])
                a2 = float(x[1])
                a3 = float(x[2])
                b1 = float(x[3])
                b2 = float(x[4])
                b3 = float(x[5])
                lm1 = float(x[6])
                lm2 = float(x[7])
                lm3 = float(x[8])
                result = np.array([
                    -(1 - cos(a1)) * (
                            -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                1 - cos(a3)) * (
                                    -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                        b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(b1) * cos(b1) + (
                             -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) + (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * cos(b1) + lm1 * (1 - cos(a1)) * cos(b1) / a1 - dst[0],
                     - (1 - cos(a1)) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(b1) * cos(b1) + (
                             -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                         b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) + (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * sin(b1) + lm1 * (1 - cos(a1)) * sin(b1) / a1 - dst[1],
                     ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) - (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(a1) * cos(b1) - (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                         b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(a1) * sin(b1) + lm1 * sin(a1) / a1-dst[2],
                    - (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                                               (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(
                        b2)) * sin(
                        b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                            -(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                                    (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(b2)) + (
                            ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                            (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                        b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) * sin(a1) * cos(b1) - a[0],
                    - (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                                               (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(
                        b2)) * sin(
                        b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                            -(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                                    (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(b2)) + (
                            ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                            (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                        b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) * sin(a1) * sin(b1) - a[1],
                    ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                            ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                            (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                        b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) - (
                            -(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                                    (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(b2)) * sin(
                        a1) * cos(b1) - (-(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                                                 (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(
                        b2)) * sin(a1) * sin(b1) - a[2],
                    -(1 - cos(a1)) * (-(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (
                            1 - cos(a3)) * (
                                              -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(
                        a3) * sin(b2) * cos(
                        b3)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                            (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(a2) * sin(
                        a3) * cos(b2) * cos(b3)) + ((1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                            (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                                                            -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(
                        b2)) * sin(
                        a1) * cos(b1) - n[0],
                    -(1 - cos(a1)) * ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(a2) * sin(
                        a3) * cos(b2) * cos(b3)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                            -(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b2) * cos(
                        b3)) + ((1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                            (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                                        -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(b2)) * sin(a1) * sin(b1) - n[
                        1],
                    ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                            (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                            (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                                    -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(b2)) - (
                            -(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                            -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b2) * cos(
                        b3)) * sin(a1) * sin(b1) - (
                            (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                            -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(
                        a2) * sin(
                        a3) * cos(b2) * cos(b3)) * sin(a1) * cos(b1) - n[2]
                ])
                return result.astype('float64')
            def pos(x):
                a1 = x[0]
                b1 = x[1]
                #lm1 = x[2]
                a2 = x[2]
                b2 = x[3]
                #lm2 = x[5]
                a3 = x[4]
                b3 = x[5]
                lm1 = x[6]
                lm2 = x[6]
                lm3 = x[6]
                result = np.array(
                    [-(1 - cos(a1)) * (
                            -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                1 - cos(a3)) * (
                                    -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                        b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(b1) * cos(b1) + (
                             -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) + (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * cos(b1) + lm1 * (1 - cos(a1)) * cos(b1) / a1,
                     - (1 - cos(a1)) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(b1) * cos(b1) + (
                             -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                         b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) + (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) * sin(a1) * sin(b1) + lm1 * (1 - cos(a1)) * sin(b1) / a1,
                     ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                             -lm3 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm3 * (1 - cos(a3)) * sin(
                         a2) * cos(
                         b2) * cos(b3) / a3 + lm3 * ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(
                         a3) / a3 + lm2 * sin(a2) / a2) - (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm3 * sin(a2) * sin(a3) * cos(
                         b2) / a3 + lm2 * (1 - cos(a2)) * cos(b2) / a2) * sin(a1) * cos(b1) - (
                             -lm3 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm3 * (
                                 1 - cos(a3)) * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm3 * sin(a2) * sin(a3) * sin(
                         b2) / a3 + lm2 * (1 - cos(a2)) * sin(b2) / a2) * sin(a1) * sin(b1) + lm1 * sin(a1) / a1]
                )
                return result
            def forwarding_orientation(x):
                a1 = float(x[0])
                b1 = float(x[1])
                a2 = float(x[2])
                b2 = float(x[3])
                a3 = float(x[4])
                b3 = float(x[5])
                rotation_3 = np.array(
                    [[-(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                1 - cos(a2)) * (
                                                -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(
                        a2) * sin(b2) * cos(
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
                              -(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                  1 - cos(a2)) * (
                                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                          b2) * cos(
                          b1)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(
                          b1)) * sin(
                          a3) * sin(b3),
                      (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                          b2) * cos(
                          b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(
                          b1)) * cos(a3) + (
                              (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                          a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3)],
                     [-(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                             -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                         a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                              -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                  1 - cos(a2)) * (
                                      -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                          b1) * cos(
                          b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                             -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                         a2)) * sin(
                         a3) * cos(b3),
                      - (1 - cos(a3)) * (
                              -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (
                                  1 - cos(a2)) * (
                                      -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                          b1) * cos(
                          b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                              (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                          a2) * sin(b1) * sin(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                          a2)) * sin(
                          a3) * sin(b3),
                      (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(
                          b1) * cos(
                          b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(
                          a2)) * cos(
                          a3) + ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                              -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                          a2) * sin(b1) * sin(b2)) * sin(a3) * sin(b3)],
                     [-(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                             -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(
                         a1)) * sin(
                         b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                              (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                  -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                          a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                              -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                          a1) * cos(
                          a2)) * sin(a3) * cos(b3),
                      - (1 - cos(a3)) * (
                              (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                  -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                          a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(b3) * cos(b3) + (
                              -(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                              (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                                  -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                          a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) - (
                              -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(
                          a1) * cos(
                          a2)) * sin(a3) * sin(b3),
                      (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                          a2)) * cos(
                          a3) + (
                              (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (
                                  -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                          a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                              (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                                  -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                          a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)]]
                )
                return rotation_3
            def total_jacobian(x, pos, ori):
                a1 = x[0]
                b1 = x[1]

                a2 = x[2]
                b2 = x[3]

                a3 = x[4]
                b3 = x[5]
                lm1= x[6]

                if a1 < 0.001:
                    a1 = pi/180
                if a2 < 0.001:
                    a2 = pi/180
                if a3 < 0.001:
                    a3 = pi/180
                q = np.array([a1, b1, a2, b2, a3, b3, lm1])

                def hat(alpha, beta):
                    vector = np.array([-sin(beta), cos(beta), 0])
                    trans = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                    hat = np.array([
                        [0, -vector[2], vector[1]],
                        [vector[2], 0, -vector[0]],
                        [-vector[1], vector[0], 0]
                    ])
                    result = trans + hat * sin(alpha) + (1 - cos(alpha)) * hat.dot(hat)
                    result[2][2] = cos(alpha)
                    return result
                def angular(alpha, beta):
                    #length = 2 - 2 * cos(alpha)
                    return np.array([
                        [-sin(beta), -cos(beta) * sin(alpha)],
                        [cos(beta), -sin(beta) * sin(alpha)],
                        [0, (1 - cos(alpha))]
                    ])

                result0 = angular(a1, b1)

                base1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]).dot(hat(a1, b1))
                result1 = base1.dot(angular(a2, b2))

                base2 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]).dot(hat(a2, b2))
                result2 = base1.dot(base2).dot(angular(a3, b3))

                desired = np.array([pos[0], pos[1], pos[2], ori[0], ori[1], ori[2]])
                desired = desired/np.sqrt(float(desired[0])**2+float(desired[1])**2+float(desired[2])**2+
                                          float(desired[3])**2+float(desired[4])**2+float(desired[5])**2)

                J = np.matrix([
                    [(-lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm1 * (1 - cos(a3)) * sin(a2) * cos(
                        b2) * cos(b3) / a3 + lm1 * sin(a3) * cos(a2) / a3 + lm1 * sin(a2) / a2) * cos(a1) * cos(b1) - (
                                 -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm1 * sin(
                             a2) * sin(a3) * cos(b2) / a3 + lm1 * (1 - cos(a2)) * cos(b2) / a2) * sin(a1) * cos(
                        b1) ** 2 - (-lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm1 * sin(a2) * sin(
                        a3) * sin(b2) / a3 + lm1 * (1 - cos(a2)) * sin(b2) / a2) * sin(a1) * sin(b1) * cos(
                        b1) + lm1 * sin(a1) * cos(b1) / a1 - lm1 * (1 - cos(a1)) * cos(b1) / a1 ** 2,
                     2 * (1 - cos(a1)) * (
                                 -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + lm1 * sin(
                             a2) * sin(a3) * cos(b2) / a3 + lm1 * (1 - cos(a2)) * cos(b2) / a2) * sin(b1) * cos(b1) - (
                                 cos(a1) - 1) * (
                                 -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm1 * sin(
                             a2) * sin(a3) * sin(b2) / a3 + lm1 * (1 - cos(a2)) * sin(b2) / a2) * sin(b1) ** 2 + (
                                 cos(a1) - 1) * (
                                 -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + lm1 * sin(
                             a2) * sin(a3) * sin(b2) / a3 + lm1 * (1 - cos(a2)) * sin(b2) / a2) * cos(b1) ** 2 - (
                                 -lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm1 * (1 - cos(a3)) * sin(
                             a2) * cos(b2) * cos(b3) / a3 + lm1 * sin(a3) * cos(a2) / a3 + lm1 * sin(a2) / a2) * sin(
                         a1) * sin(b1) - lm1 * (1 - cos(a1)) * sin(b1) / a1, (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 -lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b2) / a3 - lm1 * (
                                     1 - cos(a3)) * sin(a2) * cos(b2) ** 2 * cos(b3) / a3 + lm1 * sin(a3) * cos(
                             a2) * cos(b2) / a3 + lm1 * sin(a2) * cos(b2) / a2 - lm1 * (1 - cos(a2)) * cos(
                             b2) / a2 ** 2) + (cos(a1) - 1) * (
                                 -lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) ** 2 * sin(b3) / a3 - lm1 * (
                                     1 - cos(a3)) * sin(a2) * sin(b2) * cos(b2) * cos(b3) / a3 + lm1 * sin(a3) * sin(
                             b2) * cos(a2) / a3 + lm1 * sin(a2) * sin(b2) / a2 - lm1 * (1 - cos(a2)) * sin(
                             b2) / a2 ** 2) * sin(b1) * cos(b1) + (
                                 -lm1 * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(a2) / a3 - lm1 * (1 - cos(a3)) * cos(
                             a2) * cos(b2) * cos(b3) / a3 - lm1 * sin(a2) * sin(a3) / a3 + lm1 * cos(
                             a2) / a2 - lm1 * sin(a2) / a2 ** 2) * sin(a1) * cos(b1),
                     (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) ** 2 * sin(b3) / a3 + 2 * lm1 * (
                                     1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * (
                                             1 - cos(a2)) * (1 - cos(a3)) * sin(b3) * cos(b2) ** 2 / a3 - lm1 * sin(
                             a2) * sin(a3) * sin(b2) / a3 - lm1 * (1 - cos(a2)) * sin(b2) / a2) + (
                                 lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * cos(b3) / a3 - lm1 * (1 - cos(a3)) * sin(
                             a2) * sin(b3) * cos(b2) / a3) * sin(a1) * cos(b1) + (cos(a1) - 1) * (
                                 lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) ** 2 * cos(b3) / a3 - 2 * lm1 * (
                                     1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 - lm1 * (
                                             1 - cos(a2)) * (1 - cos(a3)) * cos(b2) ** 2 * cos(b3) / a3 + lm1 * sin(
                             a2) * sin(a3) * cos(b2) / a3 + lm1 * (1 - cos(a2)) * cos(b2) / a2) * sin(b1) * cos(b1),
                     (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 -lm1 * (1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) / a3 + lm1 * sin(a2) * cos(
                             a3) * cos(b2) / a3 + lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(
                             b2) / a3 ** 2 - lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(
                             b3) / a3 ** 2 - lm1 * sin(a2) * sin(a3) * cos(b2) / a3 ** 2) + (cos(a1) - 1) * (
                                 -lm1 * (1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) / a3 + lm1 * sin(a2) * sin(
                             b2) * cos(a3) / a3 + lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(
                             b3) / a3 ** 2 - lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                             b3) / a3 ** 2 - lm1 * sin(a2) * sin(a3) * sin(b2) / a3 ** 2) * sin(b1) * cos(b1) + (
                                 -lm1 * sin(a2) * sin(a3) * sin(b2) * sin(b3) / a3 - lm1 * sin(a2) * sin(a3) * cos(
                             b2) * cos(b3) / a3 + lm1 * cos(a2) * cos(a3) / a3 + lm1 * (1 - cos(a3)) * sin(a2) * sin(
                             b2) * sin(b3) / a3 ** 2 + lm1 * (1 - cos(a3)) * sin(a2) * cos(b2) * cos(
                             b3) / a3 ** 2 - lm1 * sin(a3) * cos(a2) / a3 ** 2) * sin(a1) * cos(b1),
                     (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b3) / a3) + (
                                 -lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * cos(b3) / a3 + lm1 * (1 - cos(a3)) * sin(
                             a2) * sin(b3) * cos(b2) / a3) * sin(a1) * cos(b1) + (
                                 lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                     1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * cos(b3) / a3) * (
                                 cos(a1) - 1) * sin(b1) * cos(b1), (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                                 -(1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + (1 - cos(a3)) * (
                                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 + sin(a2) * sin(a3) * cos(
                             b2) / a3 + (1 - cos(a2)) * cos(b2) / a2) + (cos(a1) - 1) * (
                                 -(1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 + (1 - cos(a3)) * (
                                     -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 + sin(a2) * sin(a3) * sin(
                             b2) / a3 + (1 - cos(a2)) * sin(b2) / a2) * sin(b1) * cos(b1) + (
                                 -(1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - (1 - cos(a3)) * sin(a2) * cos(
                             b2) * cos(b3) / a3 + sin(a3) * cos(a2) / a3 + sin(a2) / a2) * sin(a1) * cos(b1) + (
                                 1 - cos(a1)) * cos(b1) / a1],
                    [(-lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(
                        b3) / a3 - lm1 * (1 - cos(a3)) * sin(a2) * cos(b2) * cos(b3) / a3 + lm1 * sin(a3) * cos(
                        a2) / a3 + lm1 * sin(a2) / a2) * sin(b1) * cos(a1) - (
                                                                            -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * cos(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(a3) * cos(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * cos(
                                                                        b2) / a2) * sin(a1) * sin(b1) * cos(b1) - (
                                                                            -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * sin(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(a3) * sin(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * sin(
                                                                        b2) / a2) * sin(a1) * sin(b1) ** 2 + lm1 * sin(
                        a1) * sin(b1) / a1 - lm1 * (1 - cos(a1)) * sin(b1) / a1 ** 2, -2 * (1 - cos(a1)) * (
                                                                            -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * sin(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(a3) * sin(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * sin(
                                                                        b2) / a2) * sin(b1) * cos(b1) - (
                                                                            cos(a1) - 1) * (
                                                                            -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * cos(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(a3) * cos(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * cos(
                                                                        b2) / a2) * sin(b1) ** 2 + (cos(a1) - 1) * (
                                                                            -lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * cos(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(a3) * cos(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * cos(
                                                                        b2) / a2) * cos(b1) ** 2 + (
                                                                            -lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * sin(b3) / a3 - lm1 * (1 - cos(a3)) * sin(
                                                                        a2) * cos(b2) * cos(b3) / a3 + lm1 * sin(
                                                                        a3) * cos(a2) / a3 + lm1 * sin(a2) / a2) * sin(
                        a1) * cos(b1) + lm1 * (1 - cos(a1)) * cos(b1) / a1, (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                                                            -lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) ** 2 * sin(b3) / a3 - lm1 * (
                                                                                        1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * cos(b2) * cos(b3) / a3 + lm1 * sin(
                                                                        a3) * sin(b2) * cos(a2) / a3 + lm1 * sin(
                                                                        a2) * sin(b2) / a2 - lm1 * (1 - cos(a2)) * sin(
                                                                        b2) / a2 ** 2) + (cos(a1) - 1) * (
                                                                            -lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 - lm1 * (
                                                                                        1 - cos(a3)) * sin(a2) * cos(
                                                                        b2) ** 2 * cos(b3) / a3 + lm1 * sin(a3) * cos(
                                                                        a2) * cos(b2) / a3 + lm1 * sin(a2) * cos(
                                                                        b2) / a2 - lm1 * (1 - cos(a2)) * cos(
                                                                        b2) / a2 ** 2) * sin(b1) * cos(b1) + (
                                                                            -lm1 * (1 - cos(a3)) * sin(b2) * sin(
                                                                        b3) * cos(a2) / a3 - lm1 * (1 - cos(a3)) * cos(
                                                                        a2) * cos(b2) * cos(b3) / a3 - lm1 * sin(
                                                                        a2) * sin(a3) / a3 + lm1 * cos(
                                                                        a2) / a2 - lm1 * sin(a2) / a2 ** 2) * sin(
                        a1) * sin(b1), (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                        b2) ** 2 * cos(b3) / a3 - 2 * lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(
                        b2) / a3 - lm1 * (1 - cos(a2)) * (1 - cos(a3)) * cos(b2) ** 2 * cos(b3) / a3 + lm1 * sin(
                        a2) * sin(a3) * cos(b2) / a3 + lm1 * (1 - cos(a2)) * cos(b2) / a2) + (
                                                                            lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * cos(b3) / a3 - lm1 * (1 - cos(a3)) * sin(
                                                                        a2) * sin(b3) * cos(b2) / a3) * sin(a1) * sin(
                        b1) + (cos(a1) - 1) * (lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) ** 2 * sin(
                        b3) / a3 + 2 * lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * (
                                                           1 - cos(a2)) * (1 - cos(a3)) * sin(b3) * cos(
                        b2) ** 2 / a3 - lm1 * sin(a2) * sin(a3) * sin(b2) / a3 - lm1 * (1 - cos(a2)) * sin(
                        b2) / a2) * sin(b1) * cos(b1), (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                                                            -lm1 * (1 - cos(a2)) * sin(a3) * sin(
                                                                        b2) * cos(b2) * cos(b3) / a3 + lm1 * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * sin(a3) * sin(
                                                                        b3) / a3 + lm1 * sin(a2) * sin(b2) * cos(
                                                                        a3) / a3 + lm1 * (1 - cos(a2)) * (
                                                                                        1 - cos(a3)) * sin(b2) * cos(
                                                                        b2) * cos(b3) / a3 ** 2 - lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * sin(
                                                                        b3) / a3 ** 2 - lm1 * sin(a2) * sin(a3) * sin(
                                                                        b2) / a3 ** 2) + (cos(a1) - 1) * (
                                                                            -lm1 * (1 - cos(a2)) * sin(a3) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * sin(a3) * cos(
                                                                        b3) / a3 + lm1 * sin(a2) * cos(a3) * cos(
                                                                        b2) / a3 + lm1 * (1 - cos(a2)) * (
                                                                                        1 - cos(a3)) * sin(b2) * sin(
                                                                        b3) * cos(b2) / a3 ** 2 - lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * cos(
                                                                        b3) / a3 ** 2 - lm1 * sin(a2) * sin(a3) * cos(
                                                                        b2) / a3 ** 2) * sin(b1) * cos(b1) + (
                                                                            -lm1 * sin(a2) * sin(a3) * sin(b2) * sin(
                                                                        b3) / a3 - lm1 * sin(a2) * sin(a3) * cos(
                                                                        b2) * cos(b3) / a3 + lm1 * cos(a2) * cos(
                                                                        a3) / a3 + lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * sin(b3) / a3 ** 2 + lm1 * (
                                                                                        1 - cos(a3)) * sin(a2) * cos(
                                                                        b2) * cos(b3) / a3 ** 2 - lm1 * sin(a3) * cos(
                                                                        a2) / a3 ** 2) * sin(a1) * sin(b1),
                                                                (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                                                            lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                                        1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * cos(b3) / a3) + (
                                                                            -lm1 * (1 - cos(a3)) * sin(a2) * sin(
                                                                        b2) * cos(b3) / a3 + lm1 * (1 - cos(a3)) * sin(
                                                                        a2) * sin(b3) * cos(b2) / a3) * sin(a1) * sin(
                                                                    b1) + (-lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                    b2) * cos(b2) * cos(b3) / a3 - lm1 * (
                                                                                       1 - cos(a3)) * (
                                                                                       -(1 - cos(a2)) * cos(
                                                                                   b2) ** 2 + 1) * sin(b3) / a3) * (
                                                                            cos(a1) - 1) * sin(b1) * cos(b1),
                                                                (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                                                                            -(1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * cos(b2) * cos(b3) / a3 + (1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * sin(
                                                                                    b2) ** 2 + 1) * sin(b3) / a3 + sin(
                                                                        a2) * sin(a3) * sin(b2) / a3 + (
                                                                                        1 - cos(a2)) * sin(b2) / a2) + (
                                                                            cos(a1) - 1) * (
                                                                            -(1 - cos(a2)) * (1 - cos(a3)) * sin(
                                                                        b2) * sin(b3) * cos(b2) / a3 + (1 - cos(a3)) * (
                                                                                        -(1 - cos(a2)) * cos(
                                                                                    b2) ** 2 + 1) * cos(b3) / a3 + sin(
                                                                        a2) * sin(a3) * cos(b2) / a3 + (
                                                                                        1 - cos(a2)) * cos(
                                                                        b2) / a2) * sin(b1) * cos(b1) + (
                                                                            -(1 - cos(a3)) * sin(a2) * sin(b2) * sin(
                                                                        b3) / a3 - (1 - cos(a3)) * sin(a2) * cos(
                                                                        b2) * cos(b3) / a3 + sin(a3) * cos(
                                                                        a2) / a3 + sin(a2) / a2) * sin(a1) * sin(b1) + (
                                                                            1 - cos(a1)) * sin(b1) / a1],
                    [-(
                                -lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - lm1 * (1 - cos(a3)) * sin(
                            a2) * cos(b2) * cos(b3) / a3 + lm1 * sin(a3) * cos(a2) / a3 + lm1 * sin(a2) / a2) * sin(
                        a1) + (lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 - lm1 * (
                                1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 - lm1 * sin(a2) * sin(
                        a3) * cos(b2) / a3 - lm1 * (1 - cos(a2)) * cos(b2) / a2) * cos(a1) * cos(b1) + (lm1 * (
                                1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * (1 - cos(
                        a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 - lm1 * sin(a2) * sin(a3) * sin(
                        b2) / a3 - lm1 * (1 - cos(a2)) * sin(b2) / a2) * sin(b1) * cos(a1) + lm1 * cos(
                        a1) / a1 - lm1 * sin(a1) / a1 ** 2, -(
                                lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 - lm1 * (
                                    1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * cos(b3) / a3 - lm1 * sin(
                            a2) * sin(a3) * cos(b2) / a3 - lm1 * (1 - cos(a2)) * cos(b2) / a2) * sin(a1) * sin(b1) + (
                                                                                                                       lm1 * (
                                                                                                                           1 - cos(
                                                                                                                       a2)) * (
                                                                                                                                   1 - cos(
                                                                                                                               a3)) * sin(
                                                                                                                   b2) * cos(
                                                                                                                   b2) * cos(
                                                                                                                   b3) / a3 - lm1 * (
                                                                                                                                   1 - cos(
                                                                                                                               a3)) * (
                                                                                                                                   -(
                                                                                                                                               1 - cos(
                                                                                                                                           a2)) * sin(
                                                                                                                               b2) ** 2 + 1) * sin(
                                                                                                                   b3) / a3 - lm1 * sin(
                                                                                                                   a2) * sin(
                                                                                                                   a3) * sin(
                                                                                                                   b2) / a3 - lm1 * (
                                                                                                                                   1 - cos(
                                                                                                                               a2)) * sin(
                                                                                                                   b2) / a2) * sin(
                        a1) * cos(b1), (lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) ** 2 * sin(b3) / a3 + lm1 * (
                                1 - cos(a3)) * sin(a2) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * sin(a3) * sin(
                        b2) * cos(a2) / a3 - lm1 * sin(a2) * sin(b2) / a2 + lm1 * (1 - cos(a2)) * sin(
                        b2) / a2 ** 2) * sin(a1) * sin(b1) + (-lm1 * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(
                        a2) / a3 - lm1 * (1 - cos(a3)) * cos(a2) * cos(b2) * cos(b3) / a3 - lm1 * sin(a2) * sin(
                        a3) / a3 + lm1 * cos(a2) / a2 - lm1 * sin(a2) / a2 ** 2) * cos(a1) + (lm1 * (1 - cos(a3)) * sin(
                        a2) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (1 - cos(a3)) * sin(a2) * cos(b2) ** 2 * cos(
                        b3) / a3 - lm1 * sin(a3) * cos(a2) * cos(b2) / a3 - lm1 * sin(a2) * cos(b2) / a2 + lm1 * (
                                                                                                          1 - cos(
                                                                                                      a2)) * cos(
                        b2) / a2 ** 2) * sin(a1) * cos(b1), (lm1 * (1 - cos(a3)) * sin(a2) * sin(b2) * cos(
                        b3) / a3 - lm1 * (1 - cos(a3)) * sin(a2) * sin(b3) * cos(b2) / a3) * cos(a1) + (-lm1 * (
                                1 - cos(a2)) * (1 - cos(a3)) * sin(b2) ** 2 * sin(b3) / a3 - 2 * lm1 * (1 - cos(a2)) * (
                                                                                                                    1 - cos(
                                                                                                                a3)) * sin(
                        b2) * cos(b2) * cos(b3) / a3 + lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b3) * cos(
                        b2) ** 2 / a3 + lm1 * sin(a2) * sin(a3) * sin(b2) / a3 + lm1 * (1 - cos(a2)) * sin(
                        b2) / a2) * sin(a1) * cos(b1) + (-lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) ** 2 * cos(
                        b3) / a3 + 2 * lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 + lm1 * (
                                                                     1 - cos(a2)) * (1 - cos(a3)) * cos(b2) ** 2 * cos(
                        b3) / a3 - lm1 * sin(a2) * sin(a3) * cos(b2) / a3 - lm1 * (1 - cos(a2)) * cos(b2) / a2) * sin(
                        a1) * sin(b1), (-lm1 * sin(a2) * sin(a3) * sin(b2) * sin(b3) / a3 - lm1 * sin(a2) * sin(
                        a3) * cos(b2) * cos(b3) / a3 + lm1 * cos(a2) * cos(a3) / a3 + lm1 * (1 - cos(a3)) * sin(
                        a2) * sin(b2) * sin(b3) / a3 ** 2 + lm1 * (1 - cos(a3)) * sin(a2) * cos(b2) * cos(
                        b3) / a3 ** 2 - lm1 * sin(a3) * cos(a2) / a3 ** 2) * cos(a1) + (lm1 * (1 - cos(a2)) * sin(
                        a3) * sin(b2) * sin(b3) * cos(b2) / a3 - lm1 * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                        a3) * cos(b3) / a3 - lm1 * sin(a2) * cos(a3) * cos(b2) / a3 - lm1 * (1 - cos(a2)) * (
                                                                                                    1 - cos(a3)) * sin(
                        b2) * sin(b3) * cos(b2) / a3 ** 2 + lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * cos(
                        b2) ** 2 + 1) * cos(b3) / a3 ** 2 + lm1 * sin(a2) * sin(a3) * cos(b2) / a3 ** 2) * sin(
                        a1) * cos(b1) + (lm1 * (1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) / a3 - lm1 * (
                                -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) / a3 - lm1 * sin(a2) * sin(
                        b2) * cos(a3) / a3 - lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(
                        b3) / a3 ** 2 + lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                        b3) / a3 ** 2 + lm1 * sin(a2) * sin(a3) * sin(b2) / a3 ** 2) * sin(a1) * sin(b1), (-lm1 * (
                                1 - cos(a3)) * sin(a2) * sin(b2) * cos(b3) / a3 + lm1 * (1 - cos(a3)) * sin(a2) * sin(
                        b3) * cos(b2) / a3) * cos(a1) + (-lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(
                        b2) / a3 - lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * cos(b3) / a3) * sin(
                        a1) * sin(b1) + (lm1 * (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(
                        b3) / a3 + lm1 * (1 - cos(a3)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b3) / a3) * sin(
                        a1) * cos(b1), (-(1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) / a3 - (1 - cos(a3)) * sin(
                        a2) * cos(b2) * cos(b3) / a3 + sin(a3) * cos(a2) / a3 + sin(a2) / a2) * cos(a1) + ((1 - cos(
                        a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) / a3 - (1 - cos(a3)) * (-(1 - cos(a2)) * cos(
                        b2) ** 2 + 1) * cos(b3) / a3 - sin(a2) * sin(a3) * cos(b2) / a3 - (1 - cos(a2)) * cos(
                        b2) / a2) * sin(a1) * cos(b1) + ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * cos(b2) * cos(
                        b3) / a3 - (1 - cos(a3)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) / a3 - sin(a2) * sin(
                        a3) * sin(b2) / a3 - (1 - cos(a2)) * sin(b2) / a2) * sin(a1) * sin(b1) + sin(a1) / a1],
                    [result0[0][0], result0[0][1], result1[0][0], result1[0][1], result2[0][0], result2[0][1], 0],
                    [result0[1][0], result0[1][1], result1[1][0], result1[1][1], result2[1][0], result2[1][1], 0],
                    [result0[2][0], result0[2][1], result1[2][0], result1[2][1], result2[2][0], result2[2][1], 0]
                ]).astype('float64')

                # rank = np.linalg.matrix_rank(J)
                w = np.array([1]*7)
                qmin = [0, -2*pi,
                        0, -2*pi,
                        0, -2*pi, 0.3]
                qmax = [pi, 2*pi,
                        pi, 2*pi,
                        pi, 2*pi, 5]
                act_up = [pi * 17 / 18, 2*pi * 17 / 18,
                          pi * 17 / 18, 2*pi * 17 / 18,
                          pi * 17 / 18, 2*pi * 17 / 18, 4.7]
                act_down = [pi * 1 / 18, -2*pi * 17 / 18,
                            pi * 1 / 18, -2*pi * 17 / 18,
                            pi * 1 / 18, -2*pi * 17 / 18, 0.5]
                for i in range(7):
                    if q[i] >= qmax[i] or q[i] <= qmin[i]:
                        w[i] = 0
                    elif act_up[i] < q[i] < qmax[i] or \
                            qmin[i] < q[i] < act_down[i]:
                        if q[i] > act_up[i]:
                            d = float((q[i] - float(act_up[i])) / (float(qmax[i]) - float(act_up[i])))
                        elif q[i] < act_down[i]:
                            d = float(float((act_down[i]) - float(q[i])) / (float(act_down[i]) - float(qmin[i])))
                        else:
                            d = 1
                        w[i] = -d * d + 1
                    else:
                        w[i] = 1
                Hw = np.diag(w ** 2)
                u, s, vh = np.linalg.svd(J.dot(Hw).dot(J.transpose()))
                minimal = s[0]
                w_total = 0
                for i in range(len(w)):
                    w_total += 1-w[i]
                lmax = 0.1 + 0.1*(w_total)
                e = np.sqrt(3 * lmax)
                '''e = 0.4
                lmax =  1.2'''
                for n in s:
                    if minimal > n:
                        minimal = n
                if minimal <= e:
                    l = np.array([(1 - (minimal / e) ** 2) * lmax] * 6)
                else:
                    l = np.array([.0]*6)
                L = np.diag(l)

                I = np.diag([1]*7)
                K = np.diag([1]*7)
                inv = np.linalg.inv(J.dot(Hw).dot(J.transpose()) + L)
                projection = K.dot(I - J.transpose().dot(np.linalg.inv(J.dot(J.transpose()))).dot(J))
                criterion = np.array([0,0,0,0,0,0,1])
                #print(-projection.dot(criterion))
                joint_deta = Hw.dot(J.transpose()).dot(inv).dot(desired) #-projection.dot(criterion)+
                joint_deta = np.array(joint_deta + q)
                return joint_deta[0]
            def jacobian(x0, pos_desired, ori_desired):
                new = x0
                '''new[2] = 2*new[2]/new[0]*sin(new[0]/2)
                new[5] = 2*new[5]/new[3]*sin(new[3]/2)
                new[8] = 2*new[8]/new[6]*sin(new[6]/2)'''
                result = 0
                error = 0.00001
                pos_error = 0
                ori_error = 0
                #print(pos(new))
                #print(forwarding_orientation(new))
                for i in range(6000):
                    now = pos(new)
                    ori_now = forwarding_orientation(new)
                    ori_t = ori_desired.dot(ori_now.transpose())
                    euler = Rotation.from_matrix(ori_t).as_rotvec()
                    ori_error = euler
                    pos_error = (pos_desired - now)

                    result = total_jacobian(new, pos_error, ori_error)
                    new = result
                    if np.sqrt(ori_error[0] ** 2 + ori_error[1] ** 2 + ori_error[2] ** 2) < 0.00001 and \
                            np.sqrt(pos_error[0] ** 2 + pos_error[1] ** 2 + pos_error[2] ** 2) < 0.00001:
                        break
                print(forwarding_orientation(new))
                print('pos_error:', pos_error)
                print('ori_error:', ori_error)
                res = new
                #result = [1]*9
                return new
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
            def tranformation_normal(res):
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
            now = time.time()
            x0_rosenbrock = np.array(x0).astype('float64')
            # normal type
            '''res = least_squares(test_3_7dofs, x0_rosenbrock,
                                bounds=([0, -2 * pi, 0, -2 * pi, 0, -2 * pi, 0.1],
                                        [pi, 2 * pi, pi, 2 * pi, pi, 2 * pi, 4,]))
            new = np.array([res.x[0], res.x[1],
                            res.x[2], res.x[3],
                            res.x[4], res.x[5], res.x[6]]).astype('float64')
            result = transformation_normal(new)
            '''
            # string type

            try:
                res = least_squares(string_type, x0_rosenbrock,
                                bounds=([-pi, -pi, -pi, -2*pi, -2*pi, -2*pi, 0.09, 0.09, 0.09],
                                        [pi, pi, pi, 2*pi, 2*pi, 2*pi, 0.24, 0.24, 0.24]), ftol = 1e-4, xtol= 1e-4)
                # bounds = [(-pi, pi), (-2*pi, 2*pi), (0.09, 0.24), (-pi, pi), (-2*pi, 2*pi), (0.09, 0.24),
                #           (-pi, pi), (-2*pi, 2*pi), (0.09, 0.24)]
                # res = dual_annealing(Global, bounds)
                new = np.array([res.x[0], res.x[3], res.x[6],
                                res.x[1], res.x[4], res.x[7],
                                res.x[2], res.x[5], res.x[8]
                                ]).astype('float64')  # a1 b1 l1 a2 b2 l2 a3 b3 l3
                self.result = tranformation_string(new)

            except Exception as e:
                print(e)


            # jacobian method test
            '''res = jacobian(x0_rosenbrock, dst, R)
            result = tranformation(res)'''
            # end
            return self.result

        if model:
            # a1 a2 a3 b1 b2 b3 l1 l2 l3
            if euler is not None: # using euler angle or not
                R = self.euler_transform(euler)
                n = [R[0][0],R[1][0],R[2][0]]
                a = [R[0][2],R[1][2],R[2][2]]
            else:
                self.dst_dir = n
            # normal type
            '''pos_now = [self.seg[0].alpha*3, self.seg[0].beta,
                       self.seg[3].alpha*3, self.seg[3].beta,
                       self.seg[6].alpha*3, self.seg[6].beta,
                       self.seg[6].length*3
                       ]'''
            # string type
            pos_now = [self.seg[0].alpha * 3,
                       self.seg[3].alpha * 3,
                       self.seg[6].alpha * 3,
                       self.seg[0].beta,
                       self.seg[3].beta,
                       self.seg[6].beta,
                       self.seg[0].length*3,
                       self.seg[3].length*3,
                       self.seg[4].length*3
                       ]
            self.dst_pos = pts
            desired = test_square(pts, pos_now, a, n, R)
        else:
            desired = input

        for i in range(3): #更新目标位置信
            alphaD = desired[3*i]/3
            betaD = desired[1+3*i]
            lengthD = desired[2+3*i]/3

            if self.seg[3*i].UpdateD(alphaD, betaD, lengthD)\
                and self.seg[3*i+1].UpdateD(alphaD, betaD, lengthD)\
                and self.seg[3*i+2].UpdateD(alphaD, betaD, lengthD):
                1
            else:
                print('desired posture is unachievable')

        self.desired = desired
        return self.desired

    def move(self):
        for i in range(self.num):
            p = self.seg[i].pressureD

    def path_tracking(self):
        self.incre = 20

        # 3X3 控制模式
        for i in range(3):
            self.incre_alpha[i] = (result[i] - self.alpha[3 * i] * 3) / self.incre / 3
            self.incre_beta[i] = (result[3 + i] - self.beta[3 * i]) / self.incre
            self.incre_length[i] = (result[6 + i] - self.lm[3 * i] * 3) / self.incre / 3

            if self.incre:
                for i in range(3):
                    for j in range(3):
                        self.soft[3*i+j].alphaD += self.incre_alpha[i]
                        self.soft[3*i+j].lengthD += self.incre_length[i]
                        self.soft[3*ij].betaD += self.incre_beta[i]

    def pressure_controller(self):
        1

    def desired_orientation_euler(self):
        # Z - Y - Z
        a1,a2,a3,b1,b2,b3 = [self.seg[0].alphaD*3, self.seg[3].alphaD*3, self.seg[6].alphaD*3,
                             self.seg[0].betaD, self.seg[3].betaD, self.seg[6].betaD]
        R1 = np.array(
            [[-(1 - cos(a1)) * (
                    -(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                    -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b2) * cos(
                b3)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                      (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(a2) * sin(
                  a3) * cos(b2) * cos(b3)) + ((1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                    (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                                                      -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(b2)) * sin(
                a1) * cos(b1),
              - (1 - cos(a1)) * ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) - sin(a2) * sin(
                  a3) * sin(
                  b2) * sin(b3)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                      -(1 - cos(a2)) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b3) * cos(
                  b2)) + ((1 - cos(a3)) * sin(a2) * sin(b3) * cos(b2) * cos(b3) - (
                      (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * sin(b3) - (
                                  -(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(a2) * sin(b2)) * sin(a1) * cos(b1),
              - (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                                         (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(b2)) * sin(
                  b1) * cos(b1) + (-(1 - cos(a1)) * cos(b1) ** 2 + 1) * (
                      -(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                              (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(b2)) + (
                      ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                      (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                  b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) * sin(a1) * cos(b1)],
             [-(1 - cos(a1)) * ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(a2) * sin(
                 a3) * cos(b2) * cos(b3)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                      -(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b2) * cos(
                  b3)) + ((1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                     (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                                  -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(b2)) * sin(a1) * sin(b1),
              - (1 - cos(a1)) * (
                      -(1 - cos(a2)) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b3) * cos(
                  b2)) * sin(b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                      (1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) - sin(a2) * sin(
                  a3) * sin(b2) * sin(b3)) + ((1 - cos(a3)) * sin(a2) * sin(b3) * cos(b2) * cos(b3) - (
                      (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * sin(b3) - (
                                                      -(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(a2) * sin(b2)) * sin(
                  a1) * sin(b1),
              - (1 - cos(a1)) * (-(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                                         (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(b2)) * sin(
                  b1) * cos(b1) + (-(1 - cos(a1)) * sin(b1) ** 2 + 1) * (
                      -(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                              (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(b2)) + (
                      ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                      (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                  b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) * sin(a1) * sin(b1)],
             [((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                     (1 - cos(a3)) * sin(a2) * sin(b2) * sin(b3) * cos(b3) - (
                     (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * cos(b3) - (
                             -(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(a2) * cos(b2)) - (
                      -(1 - cos(a2)) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b2) * cos(
                  b3)) * sin(a1) * sin(b1) - ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                     -(1 - cos(a2)) * cos(b2) ** 2 + 1) * (-(1 - cos(a3)) * cos(b3) ** 2 + 1) - sin(a2) * sin(
                 a3) * cos(b2) * cos(b3)) * sin(a1) * cos(b1),
              ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                      (1 - cos(a3)) * sin(a2) * sin(b3) * cos(b2) * cos(b3) - (
                      (1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * sin(a3) * sin(b3) - (
                              -(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(a2) * sin(b2)) - (
                      -(1 - cos(a2)) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * sin(b2) * cos(b2) - (1 - cos(a3)) * (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b3) * cos(b3) - sin(a2) * sin(a3) * sin(b3) * cos(
                  b2)) * sin(a1) * cos(b1) - ((1 - cos(a2)) * (1 - cos(a3)) * sin(b2) * sin(b3) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * (-(1 - cos(a3)) * sin(b3) ** 2 + 1) - sin(a2) * sin(
                  a3) * sin(b2) * sin(b3)) * sin(a1) * sin(b1),
              ((1 - cos(a1)) * (-sin(b1) ** 2 - cos(b1) ** 2) + 1) * (
                      ((1 - cos(a2)) * (-sin(b2) ** 2 - cos(b2) ** 2) + 1) * (
                      (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) - sin(a2) * sin(a3) * sin(b2) * sin(
                  b3) - sin(a2) * sin(a3) * cos(b2) * cos(b3)) - (
                      -(1 - cos(a2)) * sin(a3) * sin(b2) * sin(b3) * cos(b2) + (
                      -(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(a3) * cos(b3) + (
                              (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * cos(b2)) * sin(
                  a1) * cos(b1) - (-(1 - cos(a2)) * sin(a3) * sin(b2) * cos(b2) * cos(b3) + (
                      -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a3) * sin(b3) + (
                                           (1 - cos(a3)) * (-sin(b3) ** 2 - cos(b3) ** 2) + 1) * sin(a2) * sin(
                  b2)) * sin(a1) * sin(b1)]]
        ) # end effector transformation
        euler = Rotation.from_matrix(R1).as_rotvec()
        return euler

    def euler_transform(self, euler): #alpha beta gamma
        R1 = Rotation.from_euler('zyz', euler).as_matrix()
        return R1

if __name__ == '__main__':
    x = [pi / 18, pi/18, pi/18, pi/4, pi/4, pi/4,0.2,0.2,0.2]
    soft1 = softArm(alpha=x[0], beta=x[3], length=x[6])
    soft2 = softArm(alpha=x[1], beta=x[3], length=x[7])
    soft3 = softArm(alpha=x[2], beta=x[3], length=x[8])
    soft4 = softArm(alpha=x[0], beta=x[4], length=x[6])
    soft5 = softArm(alpha=x[1], beta=x[4], length=x[7])
    soft6 = softArm(alpha=x[2], beta=x[4], length=x[8])
    soft7 = softArm(alpha=x[0], beta=x[5], length=x[6])
    soft8 = softArm(alpha=x[1], beta=x[5], length=x[7])
    soft9 = softArm(alpha=x[2], beta=x[5], length=x[8])

    softArms = SoftObject(soft1, soft2, soft3, soft4, soft5, soft6, soft7, soft8, soft9)
    softArms.inverse_kinematic()
    #softArms.move(
    #softArms.desired_orientation_euler()

