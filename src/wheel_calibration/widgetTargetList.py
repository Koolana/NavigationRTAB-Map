from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QRadioButton

class Communicate(QObject):
    sendTargets = pyqtSignal(list)
    sendData = pyqtSignal(int, float)

class WidgetTargetList(QtWidgets.QLabel):
    type = 0

    def __init__(self, parent):
        super().__init__(parent=parent)
        self.c = Communicate()

        pointLayout = QVBoxLayout()
        # self.te = QTextEdit()
        # pointLayout.addWidget(self.te)
        #
        # self.btnSave = QPushButton('Save points')
        # self.btnSave.setToolTip('This is a <b>QPushButton</b> widget')
        # self.btnSave.resize(self.btnSave.sizeHint())
        # self.btnSave.clicked.connect(self.sendTargets)
        #
        # pointLayout.addWidget(self.btnSave)

        self.le = QLabel("Parametr:")
        self.le.setAlignment(QtCore.Qt.AlignBottom)
        pointLayout.addWidget(self.le, 1)

        self.le = QLineEdit("1")
        pointLayout.addWidget(self.le, 1)

        radiobutton = QRadioButton("Line")
        radiobutton.setChecked(True)
        radiobutton.typeTest = 0
        radiobutton.toggled.connect(self.onClicked)
        pointLayout.addWidget(radiobutton, 1)

        radiobutton = QRadioButton("Square")
        radiobutton.typeTest = 1
        radiobutton.toggled.connect(self.onClicked)
        pointLayout.addWidget(radiobutton, 1)

        radiobutton = QRadioButton("Сircle")
        radiobutton.typeTest = 2
        radiobutton.toggled.connect(self.onClicked)
        pointLayout.addWidget(radiobutton, 1)

        self.btnSave = QPushButton('Save')
        self.btnSave.clicked.connect(self.sendTestData)
        pointLayout.addWidget(self.btnSave, 1)

        self.setLayout(pointLayout)

    def onClicked(self):
        radioButton = self.sender()
        if radioButton.isChecked():
            self.type = radioButton.typeTest

    def sendTestData(self):
        self.c.sendData.emit(self.type, float(self.le.text()))

    def sendTargets(self):
        tragetList = []

        for str in self.te.toPlainText().split("\n"):
            data = str.split(" ")
            if len(data) == 2:
                tragetList.append([float(data[0]), float(data[1])])

        self.c.sendTargets.emit(tragetList)
