# -*- coding: utf-8 -*-
"""
Created on Sun Oct 14 21:47:55 2018

@author: 402072495
"""
import os
import numpy as np
# import inputs
# import serial
import threading
import time

class myJoyStick():
    def __init__(self, parent=None):
        # self.pads = inputs.devices.gamepads
        self.dataType = np.dtype([('ABS_X', int),
                                  ('ABS_Y', int),
                                  ('ABS_Z', int),
                                  ('ABS_RX', int),
                                  ('ABS_RY', int),
                                  ('ABS_RZ', int),
                                  ('ABS_HAT0X', int),
                                  ('ABS_HAT0Y', int),
                                  ('BTN_SOUTH', int),
                                  ('BTN_NORTH', int),
                                  ('BTN_WEST', int),
                                  ('BTN_EAST', int),
                                  ('BTN_TR', int),
                                  ('BTN_TL', int),
                                  ('BTN_SELECT', int),
                                  ('BTN_START', int)])
        self.data = np.zeros(1, self.dataType)
        # self.serialPort = serial.Serial()
        # self.serialPort.port = ""
        # self.serialPort.baudrate = 921600
        self.start_joyStick_thread = 0
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.RX = 0
        self.RY = 0
        self.TZ = 0
        self.flag  = 0

    def loopJoyStick(self):
        while self.start_joyStick_thread:
            events = inputs.get_gamepad() #阻塞]
            for event in events:
                #if event.code is not 'SYN_REPORT':
                    #print(event.code, event.state)
                if event.code == 'ABS_X':
                    if event.state > 10000:
                        self.X = 0.05
                    elif event.state < -10000:
                        self.X = -0.05
                    else:
                        self.X = .0

                if event.code == 'ABS_Y':
                    if event.state > 10000:
                        self.Y = 0.05
                    elif event.state < -10000:
                        self.Y = -0.05
                    else:
                        self.Y = .0

                if event.code == 'ABS_RX':
                    if event.state > 10000:
                        self.RX = 0.05
                    elif event.state < -10000:
                        self.RX = -0.05
                    else:
                        self.RX = .0

                if event.code == 'ABS_RY':
                    if event.state > 10000:
                        self.RY = 0.05
                    elif event.state < -10000:
                        self.RY = -0.05
                    else:
                        self.RY = .0

                if event.code == 'ABS_Z':
                    if event.state > 25:
                        self.Z = 0.05
                    else:
                        self.Z = .0

                if event.code == 'ABS_RZ':
                    if event.state > 25:
                        self.Z = -0.05
                    else:
                        self.Z = .0

                if event.code == 'BTN_TL':
                    if event.state:
                        self.TZ = 0.05
                    else:
                        self.TZ = 0

                if event.code == 'BTN_TR':
                    if event.state:
                        self.TZ = -0.05
                    else:
                        self.TZ = 0

                if event.code == 'BTN_SELECT':
                    if event.state:
                        self.flag = not self.flag

    def getData(self):
        return self.data[0]

    def start(self):
        if self.start_joyStick_thread == 0:
            1
            # self.start_joyStick_thread = 1
            # self.joyStick_thread = threading.Thread(target=self.loopJoyStick, name='JoyStickThread')
            # self.joyStick_thread.daemon = True
            # #self.joyStick_thread.start()
            # print("start joyStick,  getData() to get data")
        else:
            print("already open")

    def stop(self):
        self.start_joyStick_thread = 0
        print("joyStick stopped")


if __name__ == '__main__':
    a = myJoyStick()
    a.start()
    print(a)
    while 1:
        a