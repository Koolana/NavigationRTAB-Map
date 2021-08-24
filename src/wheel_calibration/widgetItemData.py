from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QTextEdit, QGridLayout, QListWidget, QLineEdit


class ItemData(QtWidgets.QWidget):
    odomPoint = []

    def __init__(self, point, isClockwise=True, number=0, parent=None):
        super().__init__(parent=parent)

        self.odomPoint = point
        self.isClockwise = isClockwise

        globalLayout = QGridLayout()
        globalLayout.setSpacing(0)
        globalLayout.setContentsMargins(5, 5, 0, 5)

        labelNumber = QLabel(str(number))
        globalLayout.addWidget(labelNumber, 0, 0, 3, 0)

        labelRotateDir = QLabel("CW" if isClockwise else "CCW")
        labelRotateDir.setFont(QFont("Helvetica [Cronyx]", 5))
        labelRotateDir.setAlignment(QtCore.Qt.AlignBottom)
        globalLayout.addWidget(labelRotateDir, 2, 0, 1, 0)

        titleX = QLabel("x")
        titleX.setAlignment(QtCore.Qt.AlignCenter)
        globalLayout.addWidget(titleX, 0, 1, 1, 1)

        titleY = QLabel("y")
        titleY.setAlignment(QtCore.Qt.AlignCenter)
        globalLayout.addWidget(titleY, 2, 1, 1, 1)

        self.lOdomDataX = QLabel("%.2f" % (point[0]))
        self.lOdomDataX.setAlignment(QtCore.Qt.AlignCenter)
        globalLayout.addWidget(self.lOdomDataX, 0, 2, 1, 4)

        self.lOdomDataY = QLabel("%.2f" % (point[1]))
        self.lOdomDataY.setAlignment(QtCore.Qt.AlignCenter)
        globalLayout.addWidget(self.lOdomDataY, 2, 2, 1, 4)

        self.lExperDataX = QLineEdit()
        globalLayout.addWidget(self.lExperDataX, 0, 6, 1, 4)

        self.lExperDataY = QLineEdit()
        globalLayout.addWidget(self.lExperDataY, 2, 6, 1, 4)

        l = QLabel()
        l.setFixedHeight(8)
        globalLayout.addWidget(l, 1, 0, 1, 9)

        self.setLayout(globalLayout)

    def getData(self):
        if not self.isfloat(self.lExperDataX.text()) or not self.isfloat(self.lExperDataY.text()):
            return [self.odomPoint, None, self.isClockwise]
        else:
            return [self.odomPoint, [float(self.lExperDataX.text()), float(self.lExperDataY.text())], self.isClockwise]

    def isfloat(self, x):
        try:
            a = float(x)
        except (TypeError, ValueError):
            return False
        else:
            return True

    def paintEvent(self, event):
        qp = QtGui.QPainter(self)
        qp.drawLine(self.width() / 9 - 1, 0, self.width() / 9 - 1, self.height())
        qp.drawLine(self.width() / 9 + 1, 0, self.width() / 9 + 1, self.height())

        qp.drawLine(2 * self.width() / 9 - 1, 0, 2 * self.width() / 9 - 1, self.height())
        qp.drawLine(2 * self.width() / 9 + 1, 0, 2 * self.width() / 9 + 1, self.height())

        qp.drawLine(5 * self.width() / 9 + 1, 0, 5 * self.width() / 9 + 1, self.height())
        qp.drawLine(5 * self.width() / 9 - 1, 0, 5 * self.width() / 9 - 1, self.height())

        qp.drawLine(self.width() / 9, self.height() / 2, self.width(), self.height() / 2)

        qp.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
        qp.drawLine(0, self.height() - 2, self.width(), self.height() - 2)
        # qp.drawRoundedRect(5, 5, self.width() - 10, self.height() - 7, 3, 3);

    def resizeEvent(self, event):
        self.lExperDataX.setFixedWidth(3 * self.width() / 9)
        self.lExperDataY.setFixedWidth(3 * self.width() / 9)

        self.update()
