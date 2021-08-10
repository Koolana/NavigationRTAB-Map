from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPen, QColor
from PyQt5.QtCore import QPoint, Qt

import time

class WidgetDraw(QtWidgets.QLabel):
    scaleDiv = 0.5  # деление шкалы в метрах
    numVerticalLine = 10
    numHorizontalLine = 10

    robotLenX = 0.25
    robotLenY = 0.25

    robotPos = [0, 0]
    robotDrawShift = QPoint(0, 0)

    trajectoryPoints = [[0, 0]]

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
        br = QtGui.QBrush(QtGui.QColor(100, 10, 10, 40))
        pen = QPen()
        pen.setWidth(1)
        qp.setPen(pen)
        qp.setBrush(br)

        for vl in range(1, self.numVerticalLine):
            if vl == int(self.numVerticalLine / 2):
                pen.setWidth(3)
                qp.setPen(pen)
            else:
                pen.setWidth(1)
                qp.setPen(pen)

            qp.drawText(10, self.size().height() / 10 * vl - 2, str((self.numVerticalLine / 2 - vl) * self.scaleDiv) + " m")
            qp.drawLine(self.size().width() / 10 * vl, 0, self.size().width() / 10 * vl, self.size().height())

        for hl in range(1, self.numHorizontalLine):
            if hl == int(self.numHorizontalLine / 2):
                pen.setWidth(3)
                qp.setPen(pen)
            else:
                pen.setWidth(1)
                qp.setPen(pen)

            qp.rotate(-90)
            qp.drawText(-self.size().height() + 10, self.size().width() / 10 * hl - 2, str(-(self.numVerticalLine / 2 - hl) * self.scaleDiv) + " m")
            qp.rotate(90)
            qp.drawLine(0, self.size().height() / 10 * hl, self.size().width(), self.size().height() / 10 * hl)

        self.drawRobot(qp)
        self.drawTargets(qp)
        self.drawTrajectory(qp)
        # qp.drawRect(QtCore.QRect(self.begin, self.end))

    # def mousePressEvent(self, event):
    #     self.begin = event.pos()
    #     self.end = event.pos()
    #     self.update()
    #     print("beegin = ", self.begin)
    #     print("end 1 = ", self.end)
    #
    # def mouseMoveEvent(self, event):
    #     self.end = event.pos()
    #     self.update()

    def drawRobot(self, qp):
        qp.save()
        qp.setBrush(QColor(0, 0, 0, 100))

        qp.drawRect(self.robotPos[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2 - self.drawRobotWidth / 2, \
                    -self.robotPos[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2 - self.drawRobotHeight / 2, \
                    self.drawRobotWidth, self.drawRobotHeight)
        qp.restore()

    def drawTargets(self, qp):
        qp.save()
        qp.setBrush(QColor(0, 255, 0, 100))

        for target in self.listTarget:
            qp.drawRect(target[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2 - 10, \
            -target[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2 - 10, \
            20, 20)

        qp.restore()

    def drawTrajectory(self, qp):
        qp.save()

        pen = QPen(Qt.blue, 2, Qt.SolidLine)
        pen.setStyle(Qt.DashDotLine)
        qp.setPen(pen)

        prevPoint = []
        for point in self.trajectoryPoints:
            if len(prevPoint) == 0:
                prevPoint = point
                continue

            qp.drawLine(prevPoint[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2, \
            -prevPoint[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2, \
            point[0] / self.scaleDiv * self.size().width() / self.numVerticalLine + self.size().width() / 2, \
            -point[1] / self.scaleDiv * self.size().height() / self.numHorizontalLine + self.size().height() / 2)
            prevPoint = point

        qp.restore()

    def resizeEvent(self, event):
        self.drawRobotWidth = self.robotLenX / self.scaleDiv * self.size().width() / self.numVerticalLine
        self.drawRobotHeight = self.robotLenY / self.scaleDiv * self.size().height() / self.numHorizontalLine

        self.robotDrawShift.setX(self.size().width() / 2)
        self.robotDrawShift.setY(self.size().height() / 2)

    def receiveNewTargets(self, targets):
        self.listTarget = targets
        print(self.listTarget)
        self.update()
        # self.listTarget.append(target)

    def reset(self):
        print("!!!!!!!!!!!!!!!!!!!!!!")
        self.trajectoryPoints = [[0, 0]]
        self.robotPos = [0, 0]

        self.update()

    def addTrajectoryPoint(self, point):
        if abs(self.trajectoryPoints[-1][0] - point[0]) > 0.1 or abs(self.trajectoryPoints[-1][1] - point[1]) > 0.1:
            self.robotPos = [point[0], point[1]]
            print(self.robotPos)
            self.trajectoryPoints.append(point)
            self.update()

    #
    # def drawRectangles(self, qp):
    #     qp.setBrush(QColor(255, 0, 0, 100))
    #     qp.save() # save the QPainter config
    #     qp.drawRect(10, 15, 20, 20)
    #     qp.setBrush(QColor(0, 0, 255, 100))
    #     qp.drawRect(50, 15, 20, 20)
    #     qp.restore() # restore the QPainter config
    #     qp.drawRect(100, 15, 20, 20)
