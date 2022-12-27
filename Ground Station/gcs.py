import sys, os, time
import glob, serial
import json
import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import (QApplication, QDialog, QMainWindow, QInputDialog, QWidget, QLabel,QMessageBox)
from PyQt6.QtGui import QPen, QColor, QFont, QIntValidator, QDoubleValidator
from PyQt6.QtCore import Qt, QSettings, QDir, QThread, QObject, pyqtSignal as Signal, pyqtSlot as Slot

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from gui.gui_gcs import Ui_MainWindow

class SerialReaderObj(QObject):
    serialBroadcast = Signal(dict)

    def __init__(self, port):
        super().__init__(None)
        self.serialPort = port
        self.thread = None
        self.run = True
        self.data = {}


    @Slot()
    def readSerial(self):
        data = {}
        while self.run:
            message = self.serialPort.readline().decode().strip()
            print(message)
            if message != 'EOF' and len(message) > 0:
                message = message.split(':')
                if message[0] != 'pos':
                    data[message[0].strip()] = float(message[1].strip())
                elif message[0] == 'Acc' or message[0] == 'Gyr':
                    data[message[0] + 'X'] = float(message[1].split(',')[0].strip())
                    data[message[0] + 'Y'] = float(message[1].split(',')[1].strip())
                    data[message[0] + 'Z'] = float(message[1].split(',')[2].strip())
                else:
                    data['latitude'] = float(message[1].split(',')[0])
                    data['longitude'] = float(message[1].split(',')[1])

            elif len(message) > 0:
                self.serialBroadcast.emit(data)
                data = {}

            time.sleep(0.05)

class UI_MW(QMainWindow, Ui_MainWindow):
    serialStartRequested = Signal()

    def __init__(self, app_name='GUIPreliminaryTool', parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.settings = QSettings("__settings.ini",QSettings.Format.IniFormat)
        self.workingDirectory = QDir.current().currentPath()

        # Initialize some values
        self.init_def_values()
        self.serialReaderObj = None
        self.serialReaderThread = None
        self.serialPort = None

        # Other random initialisations
        self.stackedWidget.setCurrentIndex(0)

        # Initialize plotting area
        self.PLOT_FIGURES = {}
        self.dispose = 0
        self.setNewFigure('plotA',self.gridLayout_plotA,True)

        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())
        self.serialPort_CB.setCurrentIndex(0)
        self.setSignals()



    def init_def_values(self):
        """
        Initialize some default values for the GUI.

        :return: None
        """

        pass

    def serial_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def setNewFigure(self, name, layout, toolbar=False):
        '''

        :return:
        '''

        if name in self.PLOT_FIGURES:
            pass

        # Store Figure in Memory
        fig_dict = {}
        fig_dict["fig"] = plt.figure()
        fig_dict["tb"] = 0
        fig_dict["canvas"] = 0
        self.PLOT_FIGURES[name] = fig_dict

        # Set Figure Layout
        canvas = FigureCanvas(self.PLOT_FIGURES[name]["fig"])

        if toolbar:
            tb = NavigationToolbar(canvas, self)
            layout.addWidget(tb)
            self.PLOT_FIGURES["tb"] = tb

        layout.addWidget(canvas)
        self.PLOT_FIGURES["canvas"] = canvas
        # refresh canvas
        canvas.draw()

    def connectToSerial(self):
        try:
            port = self.serialPort_CB.currentText()
            arduino = serial.Serial(port=port, baudrate=115200, timeout=.1)
            self.serialPort = arduino

            # Start a thread that runs in the backgrounds continuously to update the gui
            if self.serialReaderObj != None:
                # Stop the previous thread and reset
                self.serialReaderThread.exit()
                self.serialReaderThread.wait()
                self.serialReaderObj = None
                self.serialReaderThread = None
                pass

            self.serialReaderObj = SerialReaderObj(arduino)
            self.serialReaderThread = QThread()

            self.serialReaderObj.serialBroadcast.connect(self.updateGuiData)

            self.serialStartRequested.connect(self.serialReaderObj.readSerial)
            self.serialReaderObj.moveToThread(self.serialReaderThread)
            self.serialReaderThread.start()
            self.serialStartRequested.emit()

            message = "Connection successful."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Information)
            msgBox.setText(message)
            msgBox.setWindowTitle('Success')
            msgBox.exec()
            return True
        except Exception as e:
            message = "Connection Failed:\n"+str(e)
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()
            return False

    def updateGuiData(self, data):
        self.altitude_SB.setValue(data['altitude'])
        self.pressure_SB.setValue(data['pressure'])
        self.temperature_SB.setValue(data['temperature'])

        self.latitude_SB.setValue(data['latitude'])
        self.longitude_SB.setValue(data['longitude'])
        self.altitudeGPS_SB.setValue(0)

    def setSignals(self):
        self.connectSerialButton.clicked.connect(lambda: self.connectToSerial())




if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = UI_MW()
    win.show()
    sys.exit(app.exec())
