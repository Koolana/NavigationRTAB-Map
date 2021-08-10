#! /usr/bin/env python3.6

import serial
import time

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject

class Communicate(QObject):
    sendTrajPoint = pyqtSignal(list)
    finalPosition = pyqtSignal(list)

class MoveController(QtCore.QObject):
    listTargets = []
    currentPos = []

    maxSpeed = 0.2

    err = 0.1

    finish = False

    def __init__(self, parent):
        super().__init__(parent=parent)
        self.c = Communicate()

        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
        print(serial.VERSION)

        time.sleep(1)

        self.odomTimer = QtCore.QTimer()
        self.odomTimer.timeout.connect(self.odom_callback)
        self.odomTimer.start(100)

    def odom_callback(self):
        data = self.arduino.readline()
        data = data.decode("utf-8").split("; ")

        if len(data) != 6:
            return

        print("Odom data:", data)

        x = float(data[3])
        y = float(data[4])
        th = float(data[2])

        vx = float(data[0])
        vy = 0
        vth = float(data[1])

        self.c.sendTrajPoint.emit([x, y])

        if len(self.listTargets) == 0:
            if abs(vx) < 0.01 and self.finish == True:
                self.c.finalPosition.emit([x, y])
                self.finish = False

            return

        print(abs(x - self.listTargets[0][0]), abs(y - self.listTargets[0][1]))
        if abs(x - self.listTargets[0][0]) < self.err and abs(y - self.listTargets[0][1]) < self.err:
            self.arduino.write(bytes('p', 'utf-8'))
            self.listTargets.pop(0)

            if len(self.listTargets) == 0:
                self.finish = True


    def move_callback(self):
        move_msg = "v" + "{0:.2f}".format(self.maxSpeed) + \
                      " " + "{0:.2f}".format(0)

        self.arduino.write(bytes(move_msg, 'utf-8'))
        print("Move msg:", move_msg)
        self.arduino.flush()

    def startMoving(self):
        self.move_callback()

    def receiveNewTargets(self, targets):
        self.listTargets = targets
