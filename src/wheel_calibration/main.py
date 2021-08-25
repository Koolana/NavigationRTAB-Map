import sys
from widgetDraw import WidgetDraw
from widgetTargetList import WidgetTargetList
from moveController import MoveController
from widgetCalculation import WidgetCalculation
from widgetRobot import WidgetRobot
from PyQt5 import QtCore
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

if __name__ == '__main__':
    winH = 1080
    winW = 1400

    app = QApplication(sys.argv)

    w = QWidget()
    w.resize(winW, winH)

    globalLayout = QHBoxLayout(w)

    # image = QImage(w.size().height(), w.size().width(), QImage.Format_RGB32)
    # lbl = QLabel()
    # lbl.setPixmap(QPixmap(image))
    drawLayout = QVBoxLayout()
    wd = WidgetDraw(w)
    drawLayout.addWidget(wd)

    controlButtonsLayout = QHBoxLayout()
    btnStart = QPushButton('Start')
    btnStart.setToolTip('This is a <b>QPushButton</b> widget')
    btnStart.resize(btnStart.sizeHint())
    controlButtonsLayout.addWidget(btnStart)

    btnStop = QPushButton('Stop moving')
    controlButtonsLayout.addWidget(btnStop)

    btnNextItr = QPushButton('Next iteration')
    controlButtonsLayout.addWidget(btnNextItr)

    # btnNewTest = QPushButton('New test')
    # controlButtonsLayout.addWidget(btnNewTest)

    drawLayout.addLayout(controlButtonsLayout)

    globalLayout.addLayout(drawLayout, 5)

    verticalLayout = QVBoxLayout()

    wr = WidgetRobot()
    wr.c.changeRotateDir.connect(wd.swapRotateDir)
    verticalLayout.addWidget(wr)

    wtl = WidgetTargetList(w)
    wtl.c.sendTargets.connect(wd.receiveNewTargets)
    wtl.c.sendData.connect(wd.changeTestData)
    verticalLayout.addWidget(wtl, 3)

    wc = WidgetCalculation(w)
    wtl.c.sendData.connect(wc.changeTestType)
    wr.c.sendData.connect(wc.receiveData)
    wr.c.changeRotateDir.connect(wc.changeRotateDir)
    verticalLayout.addWidget(wc, 8)

    globalLayout.addLayout(verticalLayout, 1)

    mc = MoveController(w)
    wtl.c.sendTargets.connect(mc.receiveNewTargets)
    wtl.c.sendMaxIter.connect(mc.setMaxIter)
    btnStart.clicked.connect(mc.startMoving)
    btnStop.clicked.connect(mc.stopMoving)
    btnNextItr.clicked.connect(mc.resetMoving)
    wr.c.changeRotateDir.connect(mc.changeRotateDir)
    wr.c.sendData.connect(mc.receiveSpeedData)

    wtl.c.sendData.connect(mc.changeData)

    mc.c.sendTrajPoint.connect(wd.addTrajectoryPoint)
    mc.c.finalPosition.connect(wc.receiveFinalPoint)
    mc.c.changeIter.connect(wd.newTestIteration)

    btnNextItr.clicked.connect(wd.reset)

    w.setWindowTitle('Wheel calibration')
    w.show()

    sys.exit(app.exec_())
