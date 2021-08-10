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
    maxRotate = 0.3

    err = 0.1

    finish = False
    isRotate = False

    currTarget = 0
    targetRotate = 0

    isClockwise = True

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

        print(len(self.listTargets), self.currTarget)

        if len(self.listTargets) == self.currTarget or len(self.listTargets) == 0:
            if abs(vx) < 0.01 and self.finish == True:
                self.c.finalPosition.emit([x, y])
                self.finish = False

            return

        # print(abs(x - self.listTargets[self.currTarget][0]), abs(y - self.listTargets[self.currTarget][1]))
        if abs(x - self.listTargets[self.currTarget][0]) < self.err and abs(y - self.listTargets[self.currTarget][1]) < self.err and not self.isRotate:
            self.arduino.write(bytes('p', 'utf-8'))

            time.sleep(1)

            self.currTarget += 1
            self.isRotate = True

            if len(self.listTargets) == self.currTarget:
                self.finish = True
            else:
                # (y < self.listTargets[self.currTarget][1] or x > self.listTargets[self.currTarget][0]) and
                if not self.isClockwise:
                    if abs(x - self.listTargets[self.currTarget][0]) > abs(y - self.listTargets[self.currTarget][1]):
                        if y > self.listTargets[self.currTarget][1]:
                            self.targetRotate += math.pi / 2 - math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
                        else:
                            self.targetRotate += math.pi / 2 + math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
                    else:
                        if x > self.listTargets[self.currTarget][0]:
                            self.targetRotate += math.pi / 2 - math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
                        else:
                            self.targetRotate += math.pi / 2 + math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))

                    self.move_callback(True, self.maxRotate)
                    print("Left")

                # (y > self.listTargets[self.currTarget][1] or x < self.listTargets[self.currTarget][0]) and
                if self.isClockwise:
                    if abs(x - self.listTargets[self.currTarget][0]) > abs(y - self.listTargets[self.currTarget][1]):
                        if y > self.listTargets[self.currTarget][1]:
                            self.targetRotate -= math.pi / 2 - math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
                        else:
                            self.targetRotate -= math.pi / 2 + math.tan(abs(y - self.listTargets[self.currTarget][1]) / abs(x - self.listTargets[self.currTarget][0]))
                    else:
                        if x > self.listTargets[self.currTarget][0]:
                            self.targetRotate -= math.pi / 2 - math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))
                        else:
                            self.targetRotate -= math.pi / 2 + math.tan(abs(x - self.listTargets[self.currTarget][0]) / abs(y - self.listTargets[self.currTarget][1]))

                    self.move_callback(True, -self.maxRotate)
                    print("Right")

        # print(self.finish, self.targetRotate, th)
        if self.isRotate and not self.finish:
            if abs(th) > abs(self.targetRotate):
                self.arduino.write(bytes('p', 'utf-8'))

                time.sleep(1)

                self.move_callback(False, self.maxSpeed)
                self.isRotate = False

    def move_callback(self, isRotate, speed):
        if isRotate:
            move_msg = "v" + "{0:.2f}".format(0) + \
                       " " + "{0:.2f}".format(speed)
        else:
            move_msg = "v" + "{0:.2f}".format(speed) + \
                       " " + "{0:.2f}".format(0)

        self.arduino.write(bytes(move_msg, 'utf-8'))
        print("Move msg:", move_msg)
        self.arduino.flush()

    def startMoving(self):
        self.move_callback(False, self.maxSpeed)

    def stopMoving(self):
        self.arduino.write(bytes('p', 'utf-8'))
        time.sleep(0.5)

    def resetMoving(self):
        self.stopMoving()
        self.arduino.write(bytes('n', 'utf-8'))
        self.currTarget = 0
        self.targetRotate = 0

        self.finish = False
        self.isRotate = False

    def changeRotateDir(self):
        self.isClockwise = not self.isClockwise

    def receiveNewTargets(self, targets):
        self.listTargets = targets
