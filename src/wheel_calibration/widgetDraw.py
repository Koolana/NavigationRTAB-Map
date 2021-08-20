from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPen, QColor, QFont
from PyQt5.QtCore import QPoint, Qt

import time
import random
import math

class WidgetDraw(QtWidgets.QLabel):
    scaleDiv = 0.5  # деление шкалы в метрах
    numVerticalLine = 10
    numHorizontalLine = 10

    robotLenX = 0.25
    robotLenY = 0.25

    robotPos = [0, 0, 0]
    robotVel = [0, 0]
    robotDrawShift = QPoint(0, 0)

    testType = 0
    param = 1

    numIter = 0

    isClockwise = True

    currentIterColor = QColor(0, 0, 255, 255)
    trajectoryPoints = [[[0, 0, 0], currentIterColor]]

    def __init__(self, parent):
        super().__init__(parent=parent)
        self.setStyleSheet('QFrame {background-color:white;}')
        self.resize(300, 300)
        self.begin = QtCore.QPoint()
        self.end = QtCore.QPoint()

        self.drawRobotWidth = self.robotLenX / self.scaleDiv * self.size().width() / self.numVerticalLine
        self.drawRobotHeight = self.robotLenY / self.scaleDiv * self.size().height() / self.numHorizontalLine

        self.robotDrawShift.setX(self.size().width() / 2)
        self.robotDrawShift.setY(self.size().height() / 2)

        self.listTarget = []

    def paintEvent(self, event):
        super().paintEvent(event)
        qp = QtGui.QPainter(self)

        self.drawScale(qp)
        self.drawRobot(qp)
        self.drawTargets(qp)
        self.drawTarget(qp)
        self.drawTrajectory(qp)
        self.drawCurrentRobotPos(qp)

    def drawScale(self, qp):
        qp.save()

        pen = QPen()
        pen.setWidth(1)
        qp.setPen(pen)

        for vl in range(1, self.numVerticalLine):
            if vl == int(self.numVerticalLine / 2):
                pen.setWidth(2)
                qp.setPen(pen)
            else:
                pen.setWidth(1)
                qp.setPen(pen)

            qp.rotate(-90)
            qp.drawText(-self.size().height() + 10, self.size().width() / self.numVerticalLine * vl - 2, str(-(self.numVerticalLine / 2 - vl) * self.scaleDiv) + " m")
            qp.rotate(90)
            qp.drawLine(self.size().width() / self.numVerticalLine * vl, 0, self.size().width() / self.numVerticalLine * vl, self.size().height())

        for hl in range(1, self.numHorizontalLine):
            if hl == int(self.numHorizontalLine / 2):
                pen.setWidth(2)
                qp.setPen(pen)
            else:
                pen.setWidth(1)
                qp.setPen(pen)

            qp.drawText(10, self.size().height() / self.numHorizontalLine * hl - 2, str((self.numHorizontalLine / 2 - hl) * self.scaleDiv) + " m")
            qp.drawLine(0, self.size().height() / self.numHorizontalLine * hl, self.size().width(), self.size().height() / self.numHorizontalLine * hl)

        qp.restore()

    def drawRobot(self, qp):
        qp.save()
        qp.setBrush(QColor(0, 0, 0, 100))

        qp.translate(self.robotPos[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2,
                    -self.robotPos[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2)
        qp.rotate(-self.robotPos[2] * 180 / math.pi)

        qp.drawRect(-self.drawRobotWidth / 2, -self.drawRobotHeight / 2,
                    self.drawRobotWidth, self.drawRobotHeight)

        pen = QPen(QColor(255, 20, 20, 255), 2, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawLine(0, 0, 100, 0)
        qp.drawLine(100, 0, 90, -4)
        qp.drawLine(100, 0, 90, 4)

        qp.restore()

    def drawTargets(self, qp):
        qp.save()
        qp.setBrush(QColor(0, 255, 0, 100))

        for target in self.listTarget:
            qp.drawRect(target[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2 - 10, \
            -target[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2 - 10, \
            20, 20)

        qp.restore()

    def drawTarget(self, qp):
        qp.save()

        pen = QPen(QColor(0, 150, 0, 255), 2, Qt.DotLine)
        qp.setPen(pen)

        if self.testType == 0:
            qp.drawLine(0 + self.size().width() / 2,
                        0 + self.size().height() / 2,
                        self.param / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2,
                        0 + self.size().height() / 2)

        if self.testType == 1:
            qp.drawRect(0 + self.size().width() / 2,
                        (0 if self.isClockwise else -self.param / self.scaleDiv * self.size().height() / self.numHorizontalLine) + self.size().height() / 2,
                        self.param / self.scaleDiv * self.size().width() / self.numVerticalLine,
                        self.param / self.scaleDiv * self.size().height() / self.numHorizontalLine)

        if self.testType == 2:
            qp.drawEllipse(-2 * self.param / self.scaleDiv * self.size().width() / self.numVerticalLine / 2 + self.size().width() / 2,
                           (0 if self.isClockwise else -2 * self.param / self.scaleDiv * self.size().height() / self.numHorizontalLine) + self.size().height() / 2,
                           2 * self.param / self.scaleDiv * self.size().width() / self.numVerticalLine,
                           2 * self.param / self.scaleDiv * self.size().height() / self.numHorizontalLine)

        qp.restore()

    def drawTrajectory(self, qp):
        qp.save()

        pen = QPen(self.currentIterColor, 2, Qt.SolidLine)
        qp.setPen(pen)

        prevPoint = []
        for point in self.trajectoryPoints:
            if len(prevPoint) == 0:
                prevPoint = point
                continue

            pen.setColor(point[1])
            qp.setPen(pen)

            qp.drawLine(prevPoint[0][0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2, \
            -prevPoint[0][1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2, \
            point[0][0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2, \
            -point[0][1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2)

            prevPoint = point
        qp.restore()

    def drawCurrentRobotPos(self, qp):
        qp.save()

        qp.setBrush(QColor(255, 255, 255, 255))
        qp.setPen(QPen(QColor(0, 0, 0, 0)))

        shiftFrame = 150

        qp.drawRect(self.size().width() - shiftFrame, 0, shiftFrame, 5*20 + 8)

        qp.setPen(QPen(QColor(0, 0, 0, 255)))
        qp.setFont(QFont("Helvetica [Cronyx]", 12))

        qp.drawText(self.size().width() - shiftFrame + 10, 1 * 20, "x:")
        qp.drawText(self.size().width() - shiftFrame + 40, 1 * 20, "%5.2f" % (self.robotPos[0]) + " m")

        qp.drawText(self.size().width() - shiftFrame + 10, 2 * 20, "y:")
        qp.drawText(self.size().width() - shiftFrame + 40, 2 * 20, "%5.2f" % (self.robotPos[1]) + " m")

        qp.drawText(self.size().width() - shiftFrame + 10, 3 * 20, "th:")
        qp.drawText(self.size().width() - shiftFrame + 40, 3 * 20, "%5.2f" % (self.robotPos[2]) + " rad")

        qp.drawText(self.size().width() - shiftFrame + 10, 4 * 20, "vx:")
        qp.drawText(self.size().width() - shiftFrame + 40, 4 * 20, "%5.2f" % (self.robotVel[0]) + " m/sec")

        qp.drawText(self.size().width() - shiftFrame + 10, 5 * 20, "vy:")
        qp.drawText(self.size().width() - shiftFrame + 40, 5 * 20, "%5.2f" % (self.robotVel[1]) + " rad/sec")

        qp.restore()

    def swapRotateDir(self):
        self.isClockwise = not self.isClockwise
        self.update()

    def changeTestData(self, type, param):
        self.param = param
        self.testType = type
        print(self.param, self.testType)
        self.update()

    def newTestIteration(self, numIter):
        self.numIter = numIter

        self.currentIterColor = QColor(random.randint(0, 255),
                                       random.randint(0, 255),
                                       random.randint(0, 255), 255)

    def resizeEvent(self, event):
        if self.size().width() > self.size().height():
            self.numHorizontalLine = 10

            self.numVerticalLine = int(10 * self.size().width() / self.size().height())
            self.numVerticalLine += 0 if self.numVerticalLine % 2 == 0 else 1
        else:
            self.numHorizontalLine = int(10 * self.size().height() / self.size().width())
            self.numHorizontalLine += 0 if self.numHorizontalLine % 2 == 0 else 1

            self.numVerticalLine = 10

        self.drawRobotWidth = self.robotLenX / self.scaleDiv * self.size().width() / self.numVerticalLine
        self.drawRobotHeight = self.robotLenY / self.scaleDiv * self.size().height() / self.numHorizontalLine

        self.robotDrawShift.setX(self.size().width() / 2)
        self.robotDrawShift.setY(self.size().height() / 2)

        self.update()

    def receiveNewTargets(self, targets):
        self.listTarget = targets
        # print(self.listTarget)
        self.update()
        # self.listTarget.append(target)

    def reset(self):
        print("!!!!!!!!!!!!!!!!!!!!!!")
        self.currentIterColor = QColor(0, 0, 255, 255)

        self.trajectoryPoints = [[[0, 0, 0], self.currentIterColor]]
        self.robotPos = [0, 0, 0]

        self.update()

    def addTrajectoryPoint(self, point):
        self.robotPos = [point[0], point[1], point[2]]
        self.robotVel = [point[3], point[4]]

        if abs(self.trajectoryPoints[-1][0][0] - point[0]) > 0.03 or abs(self.trajectoryPoints[-1][0][1] - point[1]) > 0.03 or abs(self.trajectoryPoints[-1][0][2] - point[2]) > 0.03:
            print(self.robotPos)
            self.trajectoryPoints.append([point[:3], self.currentIterColor])
            
        self.update()
