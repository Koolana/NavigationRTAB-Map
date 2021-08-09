import sys
from widgetDraw import WidgetDraw
from widgetTargetList import WidgetTargetList
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

if __name__ == '__main__':
    winH = 800
    winW = 700

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

    wtl = WidgetTargetList(w)
    globalLayout.addWidget(wtl, 1)

    wtl.c.sendTargets.connect(wd.receiveNewTargets)

    w.setWindowTitle('Wheel calibration')
    w.show()

    sys.exit(app.exec_())
