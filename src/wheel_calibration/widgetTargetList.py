from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QTextEdit

class Communicate(QObject):
    sendTargets = pyqtSignal(list)

class WidgetTargetList(QtWidgets.QLabel):
    def __init__(self, parent):
        super().__init__(parent=parent)
        self.c = Communicate()

        pointLayout = QVBoxLayout()
        self.te = QTextEdit()
        pointLayout.addWidget(self.te)

        self.btnSave = QPushButton('Save points')
        self.btnSave.setToolTip('This is a <b>QPushButton</b> widget')
        self.btnSave.resize(self.btnSave.sizeHint())
        self.btnSave.clicked.connect(self.sendTargets)

        pointLayout.addWidget(self.btnSave)
        self.setLayout(pointLayout)

    def sendTargets(self):
        tragetList = []

        for str in self.te.toPlainText().split("\n"):
            data = str.split(" ")
            if len(data) == 2:
                tragetList.append([float(data[0]), float(data[1])])

        self.c.sendTargets.emit(tragetList)
