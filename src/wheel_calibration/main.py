import sys
from widgetDraw import WidgetDraw
from widgetTargetList import WidgetTargetList
from moveController import MoveController
from widgetCalculation import WidgetCalculation
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

if __name__ == '__main__':
    winH = 800
    winW = 1000

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

    btnStart = QPushButton('Start')
    btnStart.setToolTip('This is a <b>QPushButton</b> widget')
    btnStart.resize(btnStart.sizeHint())
    drawLayout.addWidget(btnStart)

    globalLayout.addLayout(drawLayout, 5)

    verticalLayout = QVBoxLayout()

    wtl = WidgetTargetList(w)
    wtl.c.sendTargets.connect(wd.receiveNewTargets)
    verticalLayout.addWidget(wtl, 3)

    wc = WidgetCalculation(w)
    verticalLayout.addWidget(wc, 2)

    globalLayout.addLayout(verticalLayout, 1)

    mc = MoveController(w)
    wtl.c.sendTargets.connect(mc.receiveNewTargets)
    btnStart.clicked.connect(mc.startMoving)

    mc.c.sendTrajPoint.connect(wd.addTrajectoryPoint)
    mc.c.finalPosition.connect(wc.receiveFinalPoint)

    w.setWindowTitle('Wheel calibration')
    w.show()

    sys.exit(app.exec_())
