# Form implementation generated from reading ui file 'gui_gcs.ui'
#
# Created by: PyQt6 UI code generator 6.4.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1427, 937)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.mainTabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.mainTabWidget.setObjectName("mainTabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.tab)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.groupBox_9 = QtWidgets.QGroupBox(self.tab)
        self.groupBox_9.setMinimumSize(QtCore.QSize(0, 100))
        self.groupBox_9.setObjectName("groupBox_9")
        self.gridLayout_22 = QtWidgets.QGridLayout(self.groupBox_9)
        self.gridLayout_22.setObjectName("gridLayout_22")
        self.enableLogging_CB = QtWidgets.QCheckBox(self.groupBox_9)
        self.enableLogging_CB.setObjectName("enableLogging_CB")
        self.gridLayout_22.addWidget(self.enableLogging_CB, 2, 0, 1, 1)
        self.dataTelemLog_TW = QtWidgets.QTableWidget(self.groupBox_9)
        self.dataTelemLog_TW.setColumnCount(11)
        self.dataTelemLog_TW.setObjectName("dataTelemLog_TW")
        self.dataTelemLog_TW.setRowCount(0)
        self.gridLayout_22.addWidget(self.dataTelemLog_TW, 1, 0, 1, 1)
        self.saveLogToCSV_PB = QtWidgets.QPushButton(self.groupBox_9)
        self.saveLogToCSV_PB.setObjectName("saveLogToCSV_PB")
        self.gridLayout_22.addWidget(self.saveLogToCSV_PB, 3, 0, 1, 1)
        self.gridLayout_3.addWidget(self.groupBox_9, 1, 1, 1, 1)
        self.stackedWidget = QtWidgets.QStackedWidget(self.tab)
        self.stackedWidget.setObjectName("stackedWidget")
        self.page = QtWidgets.QWidget()
        self.page.setObjectName("page")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.page)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.groupBox = QtWidgets.QGroupBox(self.page)
        self.groupBox.setMinimumSize(QtCore.QSize(500, 0))
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.gridLayout_plotA = QtWidgets.QGridLayout()
        self.gridLayout_plotA.setObjectName("gridLayout_plotA")
        self.gridLayout_8.addLayout(self.gridLayout_plotA, 0, 0, 1, 1)
        self.gridLayout_9.addWidget(self.groupBox, 0, 1, 1, 1)
        self.gridLayout_9.setColumnStretch(1, 5)
        self.stackedWidget.addWidget(self.page)
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.gridLayout_14 = QtWidgets.QGridLayout(self.page_3)
        self.gridLayout_14.setObjectName("gridLayout_14")
        self.gridLayout_13 = QtWidgets.QGridLayout()
        self.gridLayout_13.setObjectName("gridLayout_13")
        self.gridLayout_14.addLayout(self.gridLayout_13, 0, 0, 1, 1)
        self.stackedWidget.addWidget(self.page_3)
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.gridLayout_12 = QtWidgets.QGridLayout(self.page_2)
        self.gridLayout_12.setObjectName("gridLayout_12")
        self.webEngineView = QtWebEngineWidgets.QWebEngineView(self.page_2)
        self.webEngineView.setUrl(QtCore.QUrl("about:blank"))
        self.webEngineView.setObjectName("webEngineView")
        self.gridLayout_12.addWidget(self.webEngineView, 0, 0, 1, 1)
        self.stackedWidget.addWidget(self.page_2)
        self.gridLayout_3.addWidget(self.stackedWidget, 1, 0, 1, 1)
        self.gridLayout_3.setColumnStretch(0, 1)
        self.mainTabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.gridLayout_16 = QtWidgets.QGridLayout(self.tab_2)
        self.gridLayout_16.setObjectName("gridLayout_16")
        self.stackedWidget_2 = QtWidgets.QStackedWidget(self.tab_2)
        self.stackedWidget_2.setObjectName("stackedWidget_2")
        self.page_4 = QtWidgets.QWidget()
        self.page_4.setObjectName("page_4")
        self.gridLayout_17 = QtWidgets.QGridLayout(self.page_4)
        self.gridLayout_17.setObjectName("gridLayout_17")
        self.groupBox_7 = QtWidgets.QGroupBox(self.page_4)
        self.groupBox_7.setMinimumSize(QtCore.QSize(500, 0))
        self.groupBox_7.setObjectName("groupBox_7")
        self.gridLayout_15 = QtWidgets.QGridLayout(self.groupBox_7)
        self.gridLayout_15.setObjectName("gridLayout_15")
        self.gridLayout_plotB = QtWidgets.QGridLayout()
        self.gridLayout_plotB.setObjectName("gridLayout_plotB")
        self.gridLayout_15.addLayout(self.gridLayout_plotB, 0, 0, 1, 1)
        self.groupBox_8 = QtWidgets.QGroupBox(self.groupBox_7)
        self.groupBox_8.setMaximumSize(QtCore.QSize(16777215, 150))
        self.groupBox_8.setObjectName("groupBox_8")
        self.gridLayout_20 = QtWidgets.QGridLayout(self.groupBox_8)
        self.gridLayout_20.setObjectName("gridLayout_20")
        self.gridLayout_18 = QtWidgets.QGridLayout()
        self.gridLayout_18.setObjectName("gridLayout_18")
        self.GPSEval_ProgressBar = QtWidgets.QProgressBar(self.groupBox_8)
        self.GPSEval_ProgressBar.setMaximum(101)
        self.GPSEval_ProgressBar.setProperty("value", 0)
        self.GPSEval_ProgressBar.setTextVisible(False)
        self.GPSEval_ProgressBar.setObjectName("GPSEval_ProgressBar")
        self.gridLayout_18.addWidget(self.GPSEval_ProgressBar, 1, 0, 1, 1)
        self.beginGPSAccuracy_PB = QtWidgets.QPushButton(self.groupBox_8)
        self.beginGPSAccuracy_PB.setObjectName("beginGPSAccuracy_PB")
        self.gridLayout_18.addWidget(self.beginGPSAccuracy_PB, 0, 0, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.gridLayout_18.addItem(spacerItem, 2, 0, 1, 1)
        self.gridLayout_20.addLayout(self.gridLayout_18, 0, 0, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_20.addItem(spacerItem1, 0, 3, 1, 1)
        self.gridLayout_21 = QtWidgets.QGridLayout()
        self.gridLayout_21.setObjectName("gridLayout_21")
        self.label_14 = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.gridLayout_21.addWidget(self.label_14, 0, 0, 1, 1)
        self.GPSEvalCEP_SP = QtWidgets.QDoubleSpinBox(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.GPSEvalCEP_SP.setFont(font)
        self.GPSEvalCEP_SP.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.GPSEvalCEP_SP.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.GPSEvalCEP_SP.setObjectName("GPSEvalCEP_SP")
        self.gridLayout_21.addWidget(self.GPSEvalCEP_SP, 0, 1, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        self.label_15.setFont(font)
        self.label_15.setObjectName("label_15")
        self.gridLayout_21.addWidget(self.label_15, 1, 0, 1, 1)
        self.GPSEvalR95_SP = QtWidgets.QDoubleSpinBox(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.GPSEvalR95_SP.setFont(font)
        self.GPSEvalR95_SP.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.GPSEvalR95_SP.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.GPSEvalR95_SP.setObjectName("GPSEvalR95_SP")
        self.gridLayout_21.addWidget(self.GPSEvalR95_SP, 1, 1, 1, 1)
        self.gridLayout_20.addLayout(self.gridLayout_21, 0, 2, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_20.addItem(spacerItem2, 0, 1, 1, 1)
        self.gridLayout_20.setColumnStretch(3, 1)
        self.gridLayout_15.addWidget(self.groupBox_8, 1, 0, 1, 1)
        self.gridLayout_17.addWidget(self.groupBox_7, 0, 0, 1, 1)
        self.stackedWidget_2.addWidget(self.page_4)
        self.page_5 = QtWidgets.QWidget()
        self.page_5.setObjectName("page_5")
        self.stackedWidget_2.addWidget(self.page_5)
        self.gridLayout_16.addWidget(self.stackedWidget_2, 1, 0, 1, 1)
        self.mainTabWidget.addTab(self.tab_2, "")
        self.gridLayout.addWidget(self.mainTabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1427, 22))
        self.menubar.setObjectName("menubar")
        self.menuFiles = QtWidgets.QMenu(self.menubar)
        self.menuFiles.setObjectName("menuFiles")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        self.menuTools = QtWidgets.QMenu(self.menubar)
        self.menuTools.setObjectName("menuTools")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.dockWidget_4 = QtWidgets.QDockWidget(MainWindow)
        self.dockWidget_4.setObjectName("dockWidget_4")
        self.dockWidgetContents_4 = QtWidgets.QWidget()
        self.dockWidgetContents_4.setObjectName("dockWidgetContents_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.dockWidgetContents_4)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.groupBox_2 = QtWidgets.QGroupBox(self.dockWidgetContents_4)
        self.groupBox_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.groupBox_2.setFont(font)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label = QtWidgets.QLabel(self.groupBox_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)
        self.serialPort_CB = QtWidgets.QComboBox(self.groupBox_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        self.serialPort_CB.setFont(font)
        self.serialPort_CB.setObjectName("serialPort_CB")
        self.gridLayout_2.addWidget(self.serialPort_CB, 0, 1, 1, 1)
        self.connectSerialButton = QtWidgets.QPushButton(self.groupBox_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        self.connectSerialButton.setFont(font)
        self.connectSerialButton.setObjectName("connectSerialButton")
        self.gridLayout_2.addWidget(self.connectSerialButton, 0, 2, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_2, 0, 0, 1, 1)
        self.groupBox_6 = QtWidgets.QGroupBox(self.dockWidgetContents_4)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.groupBox_6.setFont(font)
        self.groupBox_6.setObjectName("groupBox_6")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox_6)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_5 = QtWidgets.QLabel(self.groupBox_6)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.gridLayout_6.addWidget(self.label_5, 0, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox_6)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.gridLayout_6.addWidget(self.label_7, 0, 3, 1, 1)
        self.ax_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.ax_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.ax_SB.setFont(font)
        self.ax_SB.setFrame(True)
        self.ax_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.ax_SB.setReadOnly(True)
        self.ax_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.ax_SB.setMinimum(-100.0)
        self.ax_SB.setObjectName("ax_SB")
        self.gridLayout_6.addWidget(self.ax_SB, 1, 1, 1, 1)
        self.gyroZ_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.gyroZ_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.gyroZ_SB.setFont(font)
        self.gyroZ_SB.setFrame(True)
        self.gyroZ_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gyroZ_SB.setReadOnly(True)
        self.gyroZ_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.gyroZ_SB.setDecimals(1)
        self.gyroZ_SB.setMinimum(-100.0)
        self.gyroZ_SB.setObjectName("gyroZ_SB")
        self.gridLayout_6.addWidget(self.gyroZ_SB, 2, 3, 1, 1)
        self.ay_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.ay_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.ay_SB.setFont(font)
        self.ay_SB.setFrame(True)
        self.ay_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.ay_SB.setReadOnly(True)
        self.ay_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.ay_SB.setMinimum(-100.0)
        self.ay_SB.setObjectName("ay_SB")
        self.gridLayout_6.addWidget(self.ay_SB, 1, 2, 1, 1)
        self.gyroY_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.gyroY_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.gyroY_SB.setFont(font)
        self.gyroY_SB.setFrame(True)
        self.gyroY_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gyroY_SB.setReadOnly(True)
        self.gyroY_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.gyroY_SB.setDecimals(1)
        self.gyroY_SB.setMinimum(-100.0)
        self.gyroY_SB.setObjectName("gyroY_SB")
        self.gridLayout_6.addWidget(self.gyroY_SB, 2, 2, 1, 1)
        self.gyroX_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.gyroX_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.gyroX_SB.setFont(font)
        self.gyroX_SB.setFrame(True)
        self.gyroX_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gyroX_SB.setReadOnly(True)
        self.gyroX_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.gyroX_SB.setDecimals(1)
        self.gyroX_SB.setMinimum(-100.0)
        self.gyroX_SB.setObjectName("gyroX_SB")
        self.gridLayout_6.addWidget(self.gyroX_SB, 2, 1, 1, 1)
        self.az_SB = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.az_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.az_SB.setFont(font)
        self.az_SB.setFrame(True)
        self.az_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.az_SB.setReadOnly(True)
        self.az_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.az_SB.setMinimum(-100.0)
        self.az_SB.setObjectName("az_SB")
        self.gridLayout_6.addWidget(self.az_SB, 1, 3, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.groupBox_6)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.gridLayout_6.addWidget(self.label_6, 0, 2, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.groupBox_6)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.gridLayout_6.addWidget(self.label_8, 1, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.groupBox_6)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.gridLayout_6.addWidget(self.label_9, 2, 0, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_6, 4, 0, 1, 1)
        self.groupBox_3 = QtWidgets.QGroupBox(self.dockWidgetContents_4)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.groupBox_3.setFont(font)
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.groupBox_3)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.temperature_SB = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.temperature_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.temperature_SB.setFont(font)
        self.temperature_SB.setFrame(True)
        self.temperature_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.temperature_SB.setReadOnly(True)
        self.temperature_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.temperature_SB.setDecimals(1)
        self.temperature_SB.setMinimum(-50.0)
        self.temperature_SB.setObjectName("temperature_SB")
        self.gridLayout_5.addWidget(self.temperature_SB, 1, 2, 1, 1)
        self.altitude_SB = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.altitude_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.altitude_SB.setFont(font)
        self.altitude_SB.setFrame(True)
        self.altitude_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.altitude_SB.setReadOnly(True)
        self.altitude_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.altitude_SB.setDecimals(1)
        self.altitude_SB.setMinimum(-100.0)
        self.altitude_SB.setMaximum(1000.0)
        self.altitude_SB.setObjectName("altitude_SB")
        self.gridLayout_5.addWidget(self.altitude_SB, 1, 0, 1, 1)
        self.pressure_SB = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.pressure_SB.setMinimumSize(QtCore.QSize(120, 0))
        self.pressure_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.pressure_SB.setFont(font)
        self.pressure_SB.setFrame(True)
        self.pressure_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.pressure_SB.setReadOnly(True)
        self.pressure_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.pressure_SB.setDecimals(1)
        self.pressure_SB.setMaximum(1000000.0)
        self.pressure_SB.setObjectName("pressure_SB")
        self.gridLayout_5.addWidget(self.pressure_SB, 1, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_3)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.gridLayout_5.addWidget(self.label_2, 0, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.groupBox_3)
        self.label_3.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridLayout_5.addWidget(self.label_3, 0, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.groupBox_3)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.gridLayout_5.addWidget(self.label_4, 0, 2, 1, 1)
        self.gridLayout_5.setRowStretch(1, 1)
        self.gridLayout_4.addWidget(self.groupBox_3, 1, 0, 1, 1)
        self.groupBox_4 = QtWidgets.QGroupBox(self.dockWidgetContents_4)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.groupBox_4.setFont(font)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.altitudeGPS_SB = QtWidgets.QDoubleSpinBox(self.groupBox_4)
        self.altitudeGPS_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.altitudeGPS_SB.setFont(font)
        self.altitudeGPS_SB.setFrame(True)
        self.altitudeGPS_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.altitudeGPS_SB.setReadOnly(True)
        self.altitudeGPS_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.altitudeGPS_SB.setMinimum(-100.0)
        self.altitudeGPS_SB.setMaximum(1000.0)
        self.altitudeGPS_SB.setObjectName("altitudeGPS_SB")
        self.gridLayout_7.addWidget(self.altitudeGPS_SB, 2, 2, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.label_12 = QtWidgets.QLabel(self.groupBox_4)
        self.label_12.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.gridLayout_7.addWidget(self.label_12, 1, 2, 1, 1)
        self.gridLayout_11 = QtWidgets.QGridLayout()
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.longitude_SB = QtWidgets.QDoubleSpinBox(self.groupBox_4)
        self.longitude_SB.setMinimumSize(QtCore.QSize(150, 0))
        self.longitude_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.longitude_SB.setFont(font)
        self.longitude_SB.setFrame(True)
        self.longitude_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.longitude_SB.setReadOnly(True)
        self.longitude_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.longitude_SB.setDecimals(6)
        self.longitude_SB.setMinimum(-100.0)
        self.longitude_SB.setMaximum(100.0)
        self.longitude_SB.setObjectName("longitude_SB")
        self.gridLayout_11.addWidget(self.longitude_SB, 1, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.groupBox_4)
        self.label_11.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_11.setFont(font)
        self.label_11.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.gridLayout_11.addWidget(self.label_11, 0, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.groupBox_4)
        self.label_10.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_10.setFont(font)
        self.label_10.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.gridLayout_11.addWidget(self.label_10, 0, 0, 1, 1)
        self.latitude_SB = QtWidgets.QDoubleSpinBox(self.groupBox_4)
        self.latitude_SB.setMinimumSize(QtCore.QSize(150, 0))
        self.latitude_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.latitude_SB.setFont(font)
        self.latitude_SB.setFrame(True)
        self.latitude_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.latitude_SB.setReadOnly(True)
        self.latitude_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.latitude_SB.setDecimals(6)
        self.latitude_SB.setMinimum(-100.0)
        self.latitude_SB.setMaximum(100.0)
        self.latitude_SB.setObjectName("latitude_SB")
        self.gridLayout_11.addWidget(self.latitude_SB, 1, 0, 1, 1)
        self.copycoordinates_TB = QtWidgets.QToolButton(self.groupBox_4)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("copy_icon.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.copycoordinates_TB.setIcon(icon)
        self.copycoordinates_TB.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonIconOnly)
        self.copycoordinates_TB.setObjectName("copycoordinates_TB")
        self.gridLayout_11.addWidget(self.copycoordinates_TB, 1, 2, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_11, 0, 2, 1, 1)
        self.gridLayout_7.setRowStretch(0, 1)
        self.gridLayout_7.setRowStretch(1, 1)
        self.gridLayout_4.addWidget(self.groupBox_4, 2, 0, 1, 1)
        self.groupBox_5 = QtWidgets.QGroupBox(self.dockWidgetContents_4)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        self.groupBox_5.setFont(font)
        self.groupBox_5.setObjectName("groupBox_5")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.groupBox_5)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.heading_SB = QtWidgets.QDoubleSpinBox(self.groupBox_5)
        self.heading_SB.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(False)
        self.heading_SB.setFont(font)
        self.heading_SB.setFrame(True)
        self.heading_SB.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.heading_SB.setReadOnly(True)
        self.heading_SB.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.heading_SB.setDecimals(1)
        self.heading_SB.setMaximum(360.0)
        self.heading_SB.setObjectName("heading_SB")
        self.gridLayout_10.addWidget(self.heading_SB, 2, 0, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.groupBox_5)
        self.label_13.setMaximumSize(QtCore.QSize(16777215, 25))
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.gridLayout_10.addWidget(self.label_13, 1, 0, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_5, 3, 0, 1, 1)
        self.dockWidget_4.setWidget(self.dockWidgetContents_4)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(1), self.dockWidget_4)
        self.actionMRO = QtGui.QAction(MainWindow)
        self.actionMRO.setObjectName("actionMRO")
        self.actionSave = QtGui.QAction(MainWindow)
        self.actionSave.setObjectName("actionSave")
        self.actionOpenConfig = QtGui.QAction(MainWindow)
        self.actionOpenConfig.setObjectName("actionOpenConfig")
        self.actionInstructions = QtGui.QAction(MainWindow)
        self.actionInstructions.setObjectName("actionInstructions")
        self.actionImport_Preliminary_Config = QtGui.QAction(MainWindow)
        self.actionImport_Preliminary_Config.setObjectName("actionImport_Preliminary_Config")
        self.actionExportPreliminaryConfig = QtGui.QAction(MainWindow)
        self.actionExportPreliminaryConfig.setObjectName("actionExportPreliminaryConfig")
        self.actionSaveAs = QtGui.QAction(MainWindow)
        self.actionSaveAs.setObjectName("actionSaveAs")
        self.actionCruise_thrust_calculator = QtGui.QAction(MainWindow)
        self.actionCruise_thrust_calculator.setObjectName("actionCruise_thrust_calculator")
        self.menuFiles.addAction(self.actionOpenConfig)
        self.menuFiles.addAction(self.actionSave)
        self.menuFiles.addAction(self.actionSaveAs)
        self.menuFiles.addSeparator()
        self.menuFiles.addAction(self.actionExportPreliminaryConfig)
        self.menuHelp.addAction(self.actionInstructions)
        self.menubar.addAction(self.menuFiles.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuTools.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(MainWindow)
        self.mainTabWidget.setCurrentIndex(0)
        self.stackedWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_9.setTitle(_translate("MainWindow", "Logging"))
        self.enableLogging_CB.setText(_translate("MainWindow", "Enable Logging"))
        self.saveLogToCSV_PB.setText(_translate("MainWindow", "Save To CSV"))
        self.groupBox.setTitle(_translate("MainWindow", "Plotting Area"))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.tab), _translate("MainWindow", "Data and Telemetry"))
        self.groupBox_7.setTitle(_translate("MainWindow", "Plotting Area"))
        self.groupBox_8.setTitle(_translate("MainWindow", "GPS Accuracy"))
        self.beginGPSAccuracy_PB.setText(_translate("MainWindow", "Begin"))
        self.label_14.setText(_translate("MainWindow", "CEP"))
        self.GPSEvalCEP_SP.setSuffix(_translate("MainWindow", " m"))
        self.label_15.setText(_translate("MainWindow", "R95"))
        self.GPSEvalR95_SP.setSuffix(_translate("MainWindow", " m"))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.tab_2), _translate("MainWindow", "Calibration"))
        self.menuFiles.setTitle(_translate("MainWindow", "Files"))
        self.menuEdit.setTitle(_translate("MainWindow", "Edit"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.menuTools.setTitle(_translate("MainWindow", "Tools"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Serial"))
        self.label.setText(_translate("MainWindow", "COM Port"))
        self.connectSerialButton.setText(_translate("MainWindow", "Connect"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Inertial Measurement Unit"))
        self.label_5.setText(_translate("MainWindow", "X"))
        self.label_7.setText(_translate("MainWindow", "Z"))
        self.ax_SB.setSuffix(_translate("MainWindow", " g"))
        self.gyroZ_SB.setSuffix(_translate("MainWindow", " dps"))
        self.ay_SB.setSuffix(_translate("MainWindow", " g"))
        self.gyroY_SB.setSuffix(_translate("MainWindow", " dps"))
        self.gyroX_SB.setSuffix(_translate("MainWindow", " dps"))
        self.az_SB.setSuffix(_translate("MainWindow", " g"))
        self.label_6.setText(_translate("MainWindow", "Y"))
        self.label_8.setText(_translate("MainWindow", "Acceleration"))
        self.label_9.setText(_translate("MainWindow", "Gyro"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Barometer"))
        self.temperature_SB.setSuffix(_translate("MainWindow", " C"))
        self.altitude_SB.setSuffix(_translate("MainWindow", " m"))
        self.pressure_SB.setSuffix(_translate("MainWindow", " Pa"))
        self.label_2.setText(_translate("MainWindow", "Altitude"))
        self.label_3.setText(_translate("MainWindow", "Pressure"))
        self.label_4.setText(_translate("MainWindow", "Temperature"))
        self.groupBox_4.setTitle(_translate("MainWindow", "GPS & Compass"))
        self.altitudeGPS_SB.setSuffix(_translate("MainWindow", " m"))
        self.label_12.setText(_translate("MainWindow", "GPS Altitude"))
        self.longitude_SB.setSuffix(_translate("MainWindow", " °E"))
        self.label_11.setText(_translate("MainWindow", "Longitude"))
        self.label_10.setText(_translate("MainWindow", "Latitude"))
        self.latitude_SB.setSuffix(_translate("MainWindow", " °N"))
        self.copycoordinates_TB.setText(_translate("MainWindow", "..."))
        self.groupBox_5.setTitle(_translate("MainWindow", "Compass "))
        self.heading_SB.setSuffix(_translate("MainWindow", " °N"))
        self.label_13.setText(_translate("MainWindow", "Heading"))
        self.actionMRO.setText(_translate("MainWindow", "MRO"))
        self.actionSave.setText(_translate("MainWindow", "Save"))
        self.actionOpenConfig.setText(_translate("MainWindow", "Open"))
        self.actionInstructions.setText(_translate("MainWindow", "Instructions"))
        self.actionImport_Preliminary_Config.setText(_translate("MainWindow", "Import Config."))
        self.actionExportPreliminaryConfig.setText(_translate("MainWindow", "Export Config."))
        self.actionSaveAs.setText(_translate("MainWindow", "Save As"))
        self.actionCruise_thrust_calculator.setText(_translate("MainWindow", "Cruise thrust calculator"))
from PyQt6 import QtWebEngineWidgets
