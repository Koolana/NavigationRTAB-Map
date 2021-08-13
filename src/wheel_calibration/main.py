import sys
from widgetDraw import WidgetDraw
from widgetTargetList import WidgetTargetList
from moveController import MoveController
from widgetCalculation import WidgetCalculation
from PyQt5 import QtCore
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

isRightCircleArrow = True

rightCircleArrow = None
leftCircleArrow = None

def swapCircleArrow():
    global isRightCircleArrow
    global leftCircleArrow
    global rightCircleArrow

    if isRightCircleArrow:
        isRightCircleArrow = False
        btnChangeRDir.setIcon(leftCircleArrow)
    else:
        isRightCircleArrow = True
        btnChangeRDir.setIcon(rightCircleArrow)

if __name__ == '__main__':
    winH = 1080
    winW = 1400

    app = QApplication(sys.argv)

    rightCircleArrow = QIcon('img/Uhrzeigersinn.png')
    leftCircleArrow = QIcon('img/Gegenuhrzeigersinn.png')

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

    btnNewTest = QPushButton('New test')
    controlButtonsLayout.addWidget(btnNewTest)

    drawLayout.addLayout(controlButtonsLayout)

    globalLayout.addLayout(drawLayout, 5)

    verticalLayout = QVBoxLayout()

    btnChangeRDir = QPushButton()
    btnChangeRDir.setIcon(rightCircleArrow)
    btnChangeRDir.setFixedSize(90, 90)
    btnChangeRDir.setIconSize(QSize(80, 80))
    btnChangeRDir.clicked.connect(swapCircleArrow)
    btnChangeRDir.clicked.connect(wd.swapRotateDir)
    verticalLayout.addWidget(btnChangeRDir, 1, QtCore.Qt.AlignCenter | QtCore.Qt.AlignBottom)


    wtl = WidgetTargetList(w)
    wtl.c.sendTargets.connect(wd.receiveNewTargets)
    wtl.c.sendData.connect(wd.changeTestData)
    verticalLayout.addWidget(wtl, 3)

    wc = WidgetCalculation(w)
    wtl.c.sendData.connect(wc.changeTestType)
    verticalLayout.addWidget(wc, 8)

    globalLayout.addLayout(verticalLayout, 1)

    mc = MoveController(w)
    wtl.c.sendTargets.connect(mc.receiveNewTargets)
    btnStart.clicked.connect(mc.startMoving)
    btnStop.clicked.connect(mc.stopMoving)
    btnNextItr.clicked.connect(mc.resetMoving)
    btnChangeRDir.clicked.connect(mc.changeRotateDir)

    wtl.c.sendData.connect(mc.changeData)

    mc.c.sendTrajPoint.connect(wd.addTrajectoryPoint)
    mc.c.finalPosition.connect(wc.receiveFinalPoint)
    mc.c.changeIter.connect(wd.newTestIteration)

    btnNextItr.clicked.connect(wd.reset)

    w.setWindowTitle('Wheel calibration')
    w.show()

    sys.exit(app.exec_())
