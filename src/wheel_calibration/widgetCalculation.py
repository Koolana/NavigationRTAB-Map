from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

class Communicate(QObject):
    receiveFinalPoint = pyqtSignal(list)

class WidgetCalculation(QtWidgets.QLabel):
    def __init__(self, parent):
        super().__init__(parent=parent)
        self.c = Communicate()

        globalLayout = QVBoxLayout()

        textFieldsLayout = QHBoxLayout()
        self.te1 = QTextEdit()
        textFieldsLayout.addWidget(self.te1)

        self.te2 = QTextEdit()
        textFieldsLayout.addWidget(self.te2)

        globalLayout.addLayout(textFieldsLayout, 5)

        self.btnSave = QPushButton('Calculate!')
        self.btnSave.setToolTip('This is a <b>QPushButton</b> widget')
        self.btnSave.resize(self.btnSave.sizeHint())
        self.btnSave.clicked.connect(self.calc)

        globalLayout.addWidget(self.btnSave, 1)

        self.finalKoefsLabel = QLabel()
        globalLayout.addWidget(self.finalKoefsLabel, 3)

        self.setLayout(globalLayout)

    def calc(self):
        print("Calc:", self.te.toPlainText().split("\n"))

    def receiveFinalPoint(self, point):
        print("WWWWWWWWWWWWW", point)
        self.te1.append(str(point[0]) + " " + str(point[1]) + "\n")
        self.update()
