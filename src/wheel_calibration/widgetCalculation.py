from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

import math

class Communicate(QObject):
    receiveFinalPoint = pyqtSignal(list)

class WidgetCalculation(QtWidgets.QLabel):
    finalPointsList = []
    experimentPointsList = []

    type = 0
    a = 0
    b = 0.275

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

        self.btnClear = QPushButton('Clear')
        self.btnClear.clicked.connect(self.clear)

        globalLayout.addWidget(self.btnClear, 1)

        self.finalKoefsLabel = QLabel()
        globalLayout.addWidget(self.finalKoefsLabel, 3)

        self.setLayout(globalLayout)

    def clear(self):
        self.te1.setText("")
        self.te2.setText("")

        self.update()

    def calc(self):
        self.finalPointsList = []
        for str in self.te1.toPlainText().split("\n"):
            data = str.split(" ")
            if len(data) == 2:
                self.finalPointsList.append([float(data[0]), float(data[1])])

        self.experimentPointsList = []
        for str in self.te2.toPlainText().split("\n"):
            data = str.split(" ")
            if len(data) == 2:
                self.experimentPointsList.append([float(data[0]), float(data[1])])

        print("Calc:", self.experimentPointsList, self.finalPointsList)

        if self.type == 0:
            delta = [[exp[0] - odom[0], exp[1] - odom[1]] for exp, odom in zip(self.experimentPointsList, self.finalPointsList)]
            sum_delta_x = sum([i[0] for i in delta])
            sum_delta_y = sum([i[1] for i in delta])
            print(sum_delta_x, sum_delta_y)

            half_betta = math.atan(sum_delta_y / sum_delta_x)

            r = 1000 / math.sin(half_betta)
            print(r)

            Ed = (r+12.3/2)/(r-12.3/2)

            Cl = 0 if (Ed+1) == 0 else (2 / (Ed+1))

            Cr = 0 if ((1/Ed)+1) == 0 else (2/((1/Ed)+1))

            self.finalKoefsLabel.setText("Cl = " + "%.4s" % (Cl) + "\n" + \
                                         "Cr = " + "%.4s" % (Cr))
            self.update()
        else:
            delta = [[exp[0] - odom[0], exp[1] - odom[1]] for exp, odom in zip(self.experimentPointsList, self.finalPointsList)]
            sum_delta_x_right = sum([i[0] for i in delta[:5]])
            sum_delta_y_right = sum([i[1] for i in delta[:5]])

            sum_delta_x_left = sum([i[0] for i in delta[5:]])
            sum_delta_y_left = sum([i[1] for i in delta[5:]])
            print(sum_delta_x_right, sum_delta_y_right)

            betta = (sum_delta_x_right - sum_delta_x_left)/((-4)*self.a)

            R = 0 if betta == 0 else (self.a / 2) / math.sin(betta / 2)

            Ed = (R + self.b / 2) / (R - self.b / 2)

            alpha = (sum_delta_x_right + sum_delta_x_left) / ((-4) * self.a)
            b_new = (1.5708 * self.b) / (1.5708 - alpha)

            Eb = 1.5708 / (1.5708-alpha)

            Cl = 0 if (Ed + 1) == 0 else (2 / (Ed + 1))
            Cr = 0 if ((1 / Ed) + 1) == 0 else (2/((1/Ed)+1))

            self.finalKoefsLabel.setText("b_new = " + "%.4s" % (b_new) + "\n" \
                                         "Cl = " + "%.4s" % (Cl) + "\n" + \
                                         "Cr = " + "%.4s" % (Cr))
            self.update()

    def changeTestType(self, type, a):
        self.type = type
        self.a = a

    def receiveFinalPoint(self, point):
        print("WWWWWWWWWWWWW", point)
        self.te1.append(str(point[0]) + " " + str(point[1]))
        self.update()
