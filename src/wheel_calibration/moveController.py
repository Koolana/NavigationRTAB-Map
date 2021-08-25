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
    changeIter = pyqtSignal(int)

class MoveController(QtCore.QObject):
    listTargets = []
    currentPos = []

    maxSpeed = 0.1
    maxRotate = 0.3

    prevSpeedLinear = 0
    prevSpeedRotate = 0

    err = 0.05

    finish = False
    # isRotate = False
    #
    # currTarget = 0
    # targetRotate = 0

    isClockwise = True

    a = 1
    testType = 0
    state = 0
    numIter = 0
    maxIter = 1

    currL = 0
    prevX = 0
    prevY = 0

    def __init__(self, parent):
        super().__init__(parent=parent)
        self.c = Communicate()

        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
        print(serial.VERSION)

        time.sleep(1)

        self.arduino.flushInput()
        self.arduino.flushOutput()

        time.sleep(1)

        self.odomTimer = QtCore.QTimer()
        self.odomTimer.timeout.connect(self.odom_callback)
        self.odomTimer.start(100)

    def odom_callback(self):
        self.arduino.flushInput()
        self.arduino.flushOutput()

        data = self.arduino.readline()
        data = data.decode("utf-8").split("; ")

        data = list(filter(None, data))

        if len(data) != 6:
            return

        # print("Odom data:", data)
        x = float(data[3])
        y = float(data[4])
        th = float(data[2])

        vx = float(data[0])
        vy = 0
        vth = float(data[1])

        self.currL += math.sqrt(((self.prevX - x) ** 2) + ((self.prevY - y) ** 2))

        self.c.sendTrajPoint.emit([x, y, th, vx, vth])

        if self.testType == 0:
            if self.state == 1:
                self.state = 2
                self.move_callback(self.maxSpeed, 0)

            if self.state == 2 and abs(self.currL - self.a) < self.err:
                self.finish = True
                self.state = 0
                self.currL = 0
                self.stopMoving()

            if abs(vx) < 0.01 and self.finish:
                self.c.finalPosition.emit([x, y, th])
                self.numIter += 1
                self.c.changeIter.emit(self.numIter)
                self.finish = False

        if self.testType == 1:
            if self.state == 1:
                self.state = 2
                self.move_callback(self.maxSpeed, 0)

            if self.state == 2 and self.currL + vx / 10 > self.a:
                self.state = 3
                self.stopMoving()
                self.move_callback(0, (-1 if self.isClockwise else 1) * self.maxRotate)

            if self.state == 3 and abs(th + vth / 10) > math.pi / 2 + 2 * math.pi * self.numIter:
                self.state = 4
                self.stopMoving()
                self.currL = 0
                self.move_callback(self.maxSpeed, 0)

            if self.state == 4 and self.currL + vx / 10 > self.a:
                self.state = 5
                self.stopMoving()
                self.move_callback(0, (-1 if self.isClockwise else 1) * self.maxRotate)

            if self.state == 5 and abs(th + vth / 10) > math.pi + 2 * math.pi * self.numIter:
                self.state = 6
                self.stopMoving()
                self.currL = 0
                self.move_callback(self.maxSpeed, 0)

            if self.state == 6 and self.currL + vx / 10 > self.a:
                self.state = 7
                self.stopMoving()
                self.move_callback(0, (-1 if self.isClockwise else 1) * self.maxRotate)

            if self.state == 7 and abs(th + vth / 10) > 3 * math.pi / 2 + 2 * math.pi * self.numIter:
                self.state = 8
                self.stopMoving()
                self.currL = 0
                self.move_callback(self.maxSpeed, 0)

            if self.state == 8 and self.currL + vx / 10 > self.a:
                self.state = 9
                self.stopMoving()
                self.move_callback(0, (-1 if self.isClockwise else 1) * self.maxRotate)

            if self.state == 9 and abs(th + vth / 10) > 2 * math.pi + 2 * math.pi * self.numIter:
                self.currL = 0
                self.numIter += 1
                self.c.changeIter.emit(self.numIter)

                if  self.numIter == self.maxIter:
                    self.state = 0
                    self.finish = True
                else:
                    self.state = 1

                self.stopMoving()

            if abs(vx) < 0.01 and self.finish:
                self.c.finalPosition.emit([x, y, th])
                self.finish = False

        if self.testType == 2:
            if self.state == 1:
                self.state = 2

                if self.isClockwise:
                    self.move_callback(self.maxSpeed, -self.maxSpeed / self.a)
                else:
                    self.move_callback(self.maxSpeed, self.maxSpeed / self.a)

            if self.state == 2 and abs(th) > 2 * math.pi * (1 + self.numIter):
                self.numIter += 1
                self.c.changeIter.emit(self.numIter)

                if  self.numIter == self.maxIter:
                    self.finish = True
                    self.state = 0
                    self.stopMoving()

            if abs(vx) < 0.01 and abs(vth) < 0.01 and self.finish:
                self.c.finalPosition.emit([x, y, th])
                self.finish = False

        self.prevX = x
        self.prevY = y

        #
        #
        # print(len(self.listTargets), self.currTarget)
        #
        # if len(self.listTargets) == self.currTarget or len(self.listTargets) == 0:
        #     if abs(vx) < 0.01 and self.finish == True:
        #         self.c.finalPosition.emit([x, y])
        #         self.finish = False
        #
        #     return
        #
        # if abs(x - self.listTargets[self.currTarget][0]) < self.err and abs(y - self.listTargets[self.currTarget][1]) < self.err and not self.isRotate:
        #     self.arduino.write(bytes('p', 'utf-8'))
        #
        #     time.sleep(1)
        #
        #     self.currTarget += 1
        #     self.isRotate = True
        #
        #     if len(self.listTargets) == self.currTarget:
        #         self.finish = True
        #     else:
        #         if not self.isClockwise:
        #             if abs(x - self.listTargets[self.currTarget][0]) > abs(y - self.listTargets[self.currTarget][1]):
        #                 if y > self.listTargets[self.currTarget][1]:
        #                     self.targetRotate += math.pi / 2 - math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
        #                 else:
        #                     self.targetRotate += math.pi / 2 + math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
        #             else:
        #                 if x > self.listTargets[self.currTarget][0]:
        #                     self.targetRotate += math.pi / 2 - math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
        #                 else:
        #                     self.targetRotate += math.pi / 2 + math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
        #
        #             self.move_callback(True, self.maxRotate)
        #             print("Left")
        #
        #         if self.isClockwise:
        #             if abs(x - self.listTargets[self.currTarget][0]) > abs(y - self.listTargets[self.currTarget][1]):
        #                 if y > self.listTargets[self.currTarget][1]:
        #                     self.targetRotate -= math.pi / 2 - math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
        #                 else:
        #                     self.targetRotate -= math.pi / 2 + math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
        #             else:
        #                 if x > self.listTargets[self.currTarget][0]:
        #                     self.targetRotate -= math.pi / 2 - math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
        #                 else:
        #                     self.targetRotate -= math.pi / 2 + math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
        #
        #             self.move_callback(True, -self.maxRotate)
        #             print("Right")
        #
        # if self.isRotate and not self.finish:
        #     if abs(th) > abs(self.targetRotate):
        #         self.arduino.write(bytes('p', 'utf-8'))
        #
        #         time.sleep(1)
        #
        #         self.move_callback(False, self.maxSpeed)
        #         self.isRotate = False

    def move_callback(self, speedLinear, speedRotate):
        move_msg = "v" + "{0:.2f}".format(speedLinear) + \
                       " " + "{0:.2f}".format(speedRotate)

        self.prevSpeedLinear = speedLinear
        self.prevSpeedRotate = speedRotate

        self.arduino.write(bytes(move_msg, 'utf-8'))
        print("Move msg:", move_msg)
        self.arduino.flush()

    def startMoving(self):
        # self.state = 0
        #
        # if self.testType == 0 or self.testType == 1:
        #     self.move_callback(self.maxSpeed, 0)
        #
        # if self.testType == 2:
        #     if self.isClockwise:
        #         self.move_callback(self.maxSpeed, -self.maxSpeed / self.a)
        #     else:
        #         self.move_callback(self.maxSpeed, self.maxSpeed / self.a)
        if self.state == 0:
            self.state = 1
        else:
            self.move_callback(self.prevSpeedLinear, self.prevSpeedRotate)

    def stopMoving(self):
        self.arduino.write(bytes('p', 'utf-8'))
        time.sleep(0.5)

    def resetMoving(self):
        self.stopMoving()
        self.arduino.write(bytes('n', 'utf-8'))

        self.finish = False
        self.state = 0
        self.numIter = 0

        self.currL = 0
        self.prevX = 0
        self.prevY = 0

    def setMaxIter(self, num):
        self.maxIter = num

    def changeRotateDir(self):
        self.isClockwise = not self.isClockwise

    def changeData(self, type, a):
        self.testType = type
        self.a = a
        print("Changed:", type, a)

    def receiveSpeedData(self, data):
        self.maxSpeed = data[1]
        self.maxRotate = data[2]

    def receiveNewTargets(self, targets):
        self.listTargets = targets
