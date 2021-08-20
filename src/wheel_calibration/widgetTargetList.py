from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QRadioButton

class Communicate(QObject):
    sendTargets = pyqtSignal(list)
    sendData = pyqtSignal(int, float)

class WidgetTargetList(QtWidgets.QLabel):
    currTypeTest = 0

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

        self.labelParam1 = QLabel("Line length:")
        self.labelParam1.setAlignment(QtCore.Qt.AlignBottom)
        pointLayout.addWidget(self.labelParam1, 1)

        self.le = QLineEdit("1")
        self.le.editingFinished.connect(self.testChoosed)
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

        self.setLayout(pointLayout)

    def testChoosed(self):
        self.c.sendData.emit(self.currTypeTest, float(self.le.text()))

    def onClicked(self):
        radioButton = self.sender()
        if radioButton.isChecked():
            if radioButton.typeTest == 0:
                self.currTypeTest = 0
                self.labelParam1.setText("Length:")
            if radioButton.typeTest == 1:
                self.currTypeTest = 1
                self.labelParam1.setText("Side length:")
            if radioButton.typeTest == 2:
                self.currTypeTest = 2
                self.labelParam1.setText("Radius:")

            self.testChoosed()
            self.update()

    def sendTargets(self):
        tragetList = []

        for str in self.te.toPlainText().split("\n"):
            data = str.split(" ")
            if len(data) == 2:
                tragetList.append([float(data[0]), float(data[1])])

        self.c.sendTargets.emit(tragetList)
