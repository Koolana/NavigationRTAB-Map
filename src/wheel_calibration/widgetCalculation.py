from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from widgetItemData import ItemData
from os.path import expanduser

import math

class Communicate(QObject):
    receiveFinalPoint = pyqtSignal(list)

class WidgetCalculation(QtWidgets.QGroupBox):
    finalPointsList = []
    experimentPointsList = []
    isClockwisePointsList = []

    numIter = 0
    isClockwise = True

    type = 0
    a = 0
    b = 0.275

    def __init__(self, parent):
        super().__init__(parent=parent)

        self.c = Communicate()

        self.dialog = QFileDialog(self)
        self.dialog.setFileMode(QFileDialog.AnyFile)
        self.dialog.setViewMode(QFileDialog.Detail)

        self.setTitle("Experiment information")

        globalLayout = QVBoxLayout()

        textFieldsLayout = QGridLayout()

        label = QLabel("Odometry\ndata")
        label.setAlignment(QtCore.Qt.AlignCenter)
        textFieldsLayout.addWidget(label, 0, 0)

        label = QLabel("Real\ndata")
        label.setAlignment(QtCore.Qt.AlignCenter)
        textFieldsLayout.addWidget(label, 0, 1)

        self.wDataList = QListWidget()
        textFieldsLayout.addWidget(self.wDataList, 1, 0, 1, 2)

        globalLayout.addLayout(textFieldsLayout, 5)

        impExpLayout = QHBoxLayout()

        self.btnImport = QPushButton('Import')
        self.btnImport.clicked.connect(self.importFile)
        impExpLayout.addWidget(self.btnImport, 1)

        self.btnExport = QPushButton('Export')
        self.btnExport.clicked.connect(self.exportToFile)
        impExpLayout.addWidget(self.btnExport, 1)

        globalLayout.addLayout(impExpLayout, 1)

        self.btnCalc = QPushButton('Calculate!')
        self.btnCalc.setToolTip('This is a <b>QPushButton</b> widget')
        self.btnCalc.resize(self.btnCalc.sizeHint())
        self.btnCalc.clicked.connect(self.calc)
        globalLayout.addWidget(self.btnCalc, 1)

        self.btnClear = QPushButton('Clear')
        self.btnClear.clicked.connect(self.clear)
        globalLayout.addWidget(self.btnClear, 1)

        self.finalKoefsLabel = QLabel()
        globalLayout.addWidget(self.finalKoefsLabel, 3)

        self.setLayout(globalLayout)

    def importFile(self):
        filename, filetype = self.dialog.getOpenFileName(self, "Choose a data-file",
                                                        expanduser("~"), "Data files (*.csv)",
                                                        options=QFileDialog.DontUseNativeDialog)

        if filename == '':
            return
            
        self.clear()

        file = open(filename, 'r')

        for line in file.readlines():
            data = line.split(';')
            self.numIter += 1

            itemData = ItemData([float(data[1]), float(data[2])], 'True' == data[0], self.numIter)
            itemData.setExperimentPoint([float(data[3]), float(data[4])])
            self.addDataItemToList(itemData)

        file.close()
        self.update()

    def exportToFile(self):
        self.updateDatabase()

        if len(self.finalPointsList) == 0:
            return

        if len(self.experimentPointsList) == 0:
            return

        if len(self.finalPointsList) == 0:
            return

        dirname = self.dialog.getExistingDirectory(self, 'Select a folder:',
                                                   expanduser("~"), options=QFileDialog.DontUseNativeDialog | QFileDialog.ShowDirsOnly)

        file = open(dirname + '/exported.csv', 'w')

        for isClockwise, experimentPoint, odomPoint in zip(self.isClockwisePointsList, self.experimentPointsList, self.finalPointsList):
            file.write(str(isClockwise) + ";" +
                       str(odomPoint[0]) + ";" + str(odomPoint[1]) + ";" +
                       str(experimentPoint[0]) + ";" + str(experimentPoint[1]) + ";\n")

        file.close()

    def updateDatabase(self):
        self.finalPointsList = []
        self.experimentPointsList = []
        self.isClockwisePointsList = []

        for item in self.getAllListItems(self.wDataList):
            dataFromItem = self.wDataList.itemWidget(item).getData()
            if dataFromItem[0] is not None:
                self.finalPointsList.append(dataFromItem[0])

            if dataFromItem[1] is not None:
                self.experimentPointsList.append(dataFromItem[1])

            if dataFromItem[2] is not None:
                self.isClockwisePointsList.append(dataFromItem[2])

    def calc(self):
        self.updateDatabase()

        if len(self.finalPointsList) == 0:
            self.finalKoefsLabel.setText("Empty odometry data!")
            return

        if len(self.experimentPointsList) == 0:
            self.finalKoefsLabel.setText("Empty experimental data!")
            return

        if len(self.finalPointsList) != len(self.experimentPointsList):
            self.finalKoefsLabel.setText("Data size\ndoes not match!")
            return

        if self.type == 0:
            delta = [[exp[0] - odom[0], exp[1] - odom[1]] for exp, odom in zip(self.experimentPointsList, self.finalPointsList)]
            sum_delta_x = sum([i[0] for i in delta])
            sum_delta_y = sum([i[1] for i in delta])
            # print(sum_delta_x, sum_delta_y)
            if sum_delta_x == 0:
                self.finalKoefsLabel.setText("Division by zero!")
                return

            half_betta = math.atan(sum_delta_y / sum_delta_x)

            r = 1000 / math.sin(half_betta)
            # print(r)

            Ed = (r+12.3/2)/(r-12.3/2)

            Cl = 0 if (Ed+1) == 0 else (2 / (Ed+1))

            Cr = 0 if ((1/Ed)+1) == 0 else (2 / ((1 / Ed) + 1))

            self.finalKoefsLabel.setText("Results:\nCl = " + "%.4f" % (Cl) + "\n" + \
                                         "Cr = " + "%.4f" % (Cr))
        if self.type == 1:
            deltas = [[exp[0] - odom[0], exp[1] - odom[1]] for exp, odom in zip(self.experimentPointsList, self.finalPointsList)]

            sum_delta_x_right = 0
            sum_delta_y_right = 0
            sum_delta_x_left = 0
            sum_delta_y_left = 0

            for delta, isClockwise in zip(deltas, self.isClockwisePointsList):
                if isClockwise:
                    sum_delta_x_right += delta[0]
                    sum_delta_y_right += delta[1]
                else:
                    sum_delta_x_left += delta[0]
                    sum_delta_y_left += delta[1]

            print(self.isClockwisePointsList)

            betta = (sum_delta_x_right - sum_delta_x_left)/((-4)*self.a)

            R = 0 if betta == 0 else (self.a / 2) / math.sin(betta / 2)

            Ed = (R + self.b / 2) / (R - self.b / 2)

            alpha = (sum_delta_x_right + sum_delta_x_left) / ((-4) * self.a)

            Eb = (math.pi / 2) / ((math.pi / 2)-alpha)

            b_actual = Eb * self.b

            Cl = 0 if (Ed + 1) == 0 else (2 / (Ed + 1))
            Cr = 0 if ((1 / Ed) + 1) == 0 else (2 / ((1 / Ed) + 1))

            self.finalKoefsLabel.setText("Results:\nb_actual = " + "%.4f" % (b_actual) + "\n" \
                                         "Cl = " + "%.4f" % (Cl) + "\n" + \
                                         "Cr = " + "%.4f" % (Cr))

        if self.type == 2:
            self.finalKoefsLabel.setText("Calculation algorithm\nnot found!")

        self.update()

    def clear(self):
        self.wDataList.clear()

        self.numIter = 0

        self.update()

    def changeTestType(self, type, a):
        self.type = type
        self.a = a

    def changeRotateDir(self):
        self.isClockwise = not self.isClockwise

    def addDataItemToList(self, item):
        lwi = QListWidgetItem(self.wDataList)
        lwi.setFlags(QtCore.Qt.NoItemFlags)
        lwi.setSizeHint(item.sizeHint())

        self.wDataList.addItem(lwi)
        self.wDataList.setItemWidget(lwi, item)
        print("New odom point added")

    def receiveFinalPoint(self, point):
        self.numIter += 1

        itemData = ItemData(point, self.isClockwise, self.numIter)
        self.addDataItemToList(itemData)

        self.update()

    def receiveData(self, data):
        self.b = data[0]

    def getAllListItems(self, widgetList):
        for i in range(widgetList.count()):
            yield widgetList.item(i)
