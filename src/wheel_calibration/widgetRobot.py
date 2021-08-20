from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, \
QTextEdit, QGridLayout, QListWidget, QListWidgetItem, QLineEdit

from widgetItemData import ItemData

import math

class Communicate(QObject):
    changeRotateDir = pyqtSignal()
    sendData = pyqtSignal(list)

class WidgetRobot(QtWidgets.QWidget):
    isRightCircleArrow = True

    rightCircleArrow = None
    leftCircleArrow = None

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.c = Communicate()

        self.rightCircleArrow = QIcon('img/Uhrzeigersinn.png')
        self.leftCircleArrow = QIcon('img/Gegenuhrzeigersinn.png')

        globalLayout = QGridLayout()

        titleRobot = QLabel("Robot parameters")
        globalLayout.addWidget(titleRobot, 0, 0, 1, 4)

        self.btnChangeRDir = QPushButton()
        self.btnChangeRDir.setIcon(self.rightCircleArrow)
        self.btnChangeRDir.setFixedSize(90, 90)
        self.btnChangeRDir.setIconSize(QSize(80, 80))
        self.btnChangeRDir.clicked.connect(self.swapCircleArrow)
        globalLayout.addWidget(self.btnChangeRDir, 1, 0, 4, 4, QtCore.Qt.AlignCenter | QtCore.Qt.AlignBottom)

        lWidth = QLabel("Width:")
        globalLayout.addWidget(lWidth, 5, 0, 1, 1)

        self.leWidth = QLineEdit(str(0.275))
        globalLayout.addWidget(self.leWidth, 6, 0, 1, 4)

        lLinSpeed = QLabel("Linear speed:")
        globalLayout.addWidget(lLinSpeed, 7, 0, 1, 1)

        self.leLinSpeed = QLineEdit(str(0.1))
        globalLayout.addWidget(self.leLinSpeed, 8, 0, 1, 4)

        lAngSpeed = QLabel("Angular speed:")
        globalLayout.addWidget(lAngSpeed, 9, 0, 1, 1)

        self.leAngSpeed = QLineEdit(str(0.3))
        globalLayout.addWidget(self.leAngSpeed, 10, 0, 1, 4)

        self.btnSave = QPushButton("Save")
        self.btnSave.clicked.connect(self.sendData)
        globalLayout.addWidget(self.btnSave, 11, 0, 1, 4)

        self.setLayout(globalLayout)

        self.c.sendData.emit([float(self.leWidth.text()),
                              float(self.leLinSpeed.text()),
                              float(self.leAngSpeed.text())])

    def sendData(self):
        self.c.sendData.emit([float(self.leWidth.text()),
                              float(self.leLinSpeed.text()),
                              float(self.leAngSpeed.text())])

    def swapCircleArrow(self):
        if self.isRightCircleArrow:
            self.isRightCircleArrow = False
            self.btnChangeRDir.setIcon(self.leftCircleArrow)
        else:
            self.isRightCircleArrow = True
            self.btnChangeRDir.setIcon(self.rightCircleArrow)

        self.c.changeRotateDir.emit()