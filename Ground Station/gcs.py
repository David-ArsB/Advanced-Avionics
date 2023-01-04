import sys, time
import glob, serial
import folium, io
from Modules.GPSFuncs import distCoordsComponentsSigned
import numpy as np
from math import atan2, pi
from PyQt6.QtWidgets import (QApplication, QMainWindow, QTableWidgetItem, QMessageBox)
from PyQt6.QtGui import QClipboard
from PyQt6.QtCore import Qt, QSettings, QDir, QThread, QObject, pyqtSignal as Signal, pyqtSlot as Slot

import matplotlib.pyplot as plt
from matplotlib.ticker import (FormatStrFormatter, LinearLocator)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from gui.gui_gcs import Ui_MainWindow

import serial.tools.list_ports





class GPSEvaluatorWorker(QObject):
    finished = Signal()
    progress = Signal(int)
    data = Signal(dict)

    def __init__(self, main):
        super().__init__(None)
        self.main = main



    def run(self):
        """Long-running task."""
        n = 0
        oldDataTag = None

        fig = self.main.PLOT_FIGURES['GPS_Acc']['fig']
        ax = fig.gca()
        ax.cla()
        ax.grid()
        ax.plot([], [], 'ro', markersize=3)
        ax.axis('equal')
        ax.plot([0], [0], 'x', color='gold', markersize=10)
        self.coords = {}
        self.coords['lat'] = []
        self.coords['long'] = []

        self.main.GPSEvalCEP_SP.setValue(0)
        self.main.GPSEvalR95_SP.setValue(0)
        self.main.GPSEval_ProgressBar.setValue(0)

        while n < 100:
            try:
                data = self.main.serialReaderObj.data
                dataTag = data['tag']

                if dataTag == oldDataTag:
                    time.sleep(0.25)
                    continue
                else:
                    try:
                        lat = data['latitude']
                        long = data['longitude']
                        self.coords['lat'].append(lat)
                        self.coords['long'].append(long)
                        oldDataTag = dataTag
                    except:
                        continue

                n += 1
                self.data.emit(data)
                if n != 100:
                    self.main.GPSEval_ProgressBar.setValue(n)
                else:
                    self.main.GPSEval_ProgressBar.setValue(self.main.GPSEval_ProgressBar.maximum())

                print(n)

            except Exception as e:
                print('[GPS Eval] Exception has occured: ' + str(e))

            self.finished.emit()

class SerialReaderObj(QObject):
    serialBroadcast = Signal(dict)

    def __init__(self, port):
        super().__init__(None)
        self.serialPort = port
        self.thread = None
        self.run = True
        self.data = {}
        self.tx_buf = None

    def readSerialOutline(self):
        message = self.serialPort.readline().decode().strip()
        if message[0] == 'pos':
            return float(message[1].split(',')[0]), float(message[1].split(',')[1])

        return None

    def writeToSerial(self):
        if self.tx_buf != None:
            tx_buf = self.tx_buf
        else:
            tx_buf = '\0'

        self.serialPort.write(tx_buf.encode())
        self.tx_buf = None

    @Slot()
    def readSerial(self):
        data = {}
        data['tag'] = 1
        messages = []
        while self.run:
            message = self.serialPort.readline().decode().strip()
            if message == 'BOF':
                while message != 'EOF':
                    message = self.serialPort.readline().decode().strip()
                    messages.append(message)
                    message = message.split(':')
                    if message[0].find('$b') != -1:
                        pass
                    elif message[0] == 'pos':
                        data['latitude'] = float(message[1].split(',')[0])
                        data['longitude'] = float(message[1].split(',')[1])
                    elif message[0] == 'Acc' or message[0] == 'Gyr' or message[0] == 'Mag':
                        data[message[0] + 'X'] = float(message[1].split(',')[0].strip())
                        data[message[0] + 'Y'] = float(message[1].split(',')[1].strip())
                        data[message[0] + 'Z'] = float(message[1].split(',')[2].strip())
                    elif message[0] == 'EOF':
                        tag = data['tag']
                        self.serialBroadcast.emit(data)
                        self.data = data
                        data = {}
                        data['tag'] = tag + 1
                        break
                    else:
                        try:
                            data[message[0].strip()] = float(message[1].strip())
                        except:
                            pass

                    time.sleep(0.05)
                self.writeToSerial()

            # if message != messageOld:
            #
            #     #print(message)
            #     if message != 'EOF' and message != 'BOF' and len(message) > 0:
            #         message = message.split(':')
            #         if message[0].find('$b') != -1:
            #             pass
            #         elif message[0] == 'pos':
            #             data['latitude'] = float(message[1].split(',')[0])
            #             data['longitude'] = float(message[1].split(',')[1])
            #         elif message[0] == 'Acc' or message[0] == 'Gyr' or message[0] == 'Mag':
            #             data[message[0] + 'X'] = float(message[1].split(',')[0].strip())
            #             data[message[0] + 'Y'] = float(message[1].split(',')[1].strip())
            #             data[message[0] + 'Z'] = float(message[1].split(',')[2].strip())
            #         else:
            #             try:
            #                 data[message[0].strip()] = float(message[1].strip())
            #             except:
            #                 pass
            #                 #print('Missed Frame: (%s: %s)' % (message[0], message[1]))



                # elif len(message) > 0 and message != 'BOF':
                #     tag = data['tag']
                #     self.serialBroadcast.emit(data)
                #     self.data = data
                #     data = {}
                #     data['tag'] = tag+1
                #
                # messageOld = message







class UI_MW(QMainWindow, Ui_MainWindow):
    serialStartRequested = Signal()
    gpsEvalRequested = Signal()

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
        self.dataTelemLog_TW.setHorizontalHeaderLabels(['Time', 'Altitude', 'Latitude', 'Longitude', 'Heading'])
        self.dataTelemLog_TW.resizeColumnsToContents()




        # Initialize plotting area
        self.PLOT_FIGURES = {}
        self.dispose = 0
        self.setNewFigure('plotA', self.gridLayout_plotA, True)
        self.setNewFigure('GPS_Acc', self.gridLayout_plotB, True)

        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())
        self.serialPort_CB.setCurrentIndex(1)
        self.setSignals()

        c = (45.517449, -73.784236)
        m = folium.Map(
            title='GPS Coordinates',
            zoom_start=13,
            location=c
        )
        data = io.BytesIO()
        m.save(data,close_file=False)
        self.webEngineView.setHtml(data.getvalue().decode())
        self.map = m

        lon1 = -73.78810
        lon2 = -73.77935
        lat1 = 45.51600
        lat2 = 45.52080
        fig = self.PLOT_FIGURES['plotA']['fig']
        ax = fig.gca()
        ax.set_xlim(round(lon1,5), round(lon2,5))
        ax.set_ylim(round(lat1,5), round(lat2,5))
        ax.xaxis.set_major_locator(LinearLocator(numticks = 7))
        ax.xaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
        ax.yaxis.set_major_locator(LinearLocator(numticks = 7))
        ax.yaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
        #ax.set_xticks(np.arange(lon1, lon2, (lon2-lon1)/4))
        #ax.set_yticks(np.arange(lat1, lat2, (lat2-lat1)/4))
        ax.plot([],[],'o')
        ax.imshow(plt.imread('map.png'), extent=(lon1, lon2, lat1, lat2), aspect=(1756/2252)**-1)  # Load the image to matplotlib plot.
        ax.grid()
        self.PLOT_FIGURES['plotA']['canvas'].draw()

        fig = self.PLOT_FIGURES['GPS_Acc']['fig']
        ax = fig.gca()
        ax.plot([], [], 'ro')
        ax.grid()
        self.PLOT_FIGURES['GPS_Acc']['canvas'].draw()


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
        ports = serial.tools.list_ports.comports()
        res = []
        for port, desc, hwid in sorted(ports):
            res.append("{}: {}".format(port, desc))

        return res

        # if sys.platform.startswith('win'):
        #     ports = ['COM%s' % (i + 1) for i in range(9)]
        # elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        #     # this excludes your current terminal "/dev/tty"
        #     ports = glob.glob('/dev/tty[A-Za-z]*')
        # elif sys.platform.startswith('darwin'):
        #     ports = glob.glob('/dev/tty.*')
        # else:
        #     raise EnvironmentError('Unsupported platform')
        #
        # result = []
        # for port in ports:
        #     try:
        #         s = serial.Serial(port)
        #         s.close()
        #         result.append(port)
        #     except (OSError, serial.SerialException) as e:
        #         pass
        # return result

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
            self.PLOT_FIGURES[name]["tb"] = tb

        layout.addWidget(canvas)
        self.PLOT_FIGURES[name]["canvas"] = canvas
        # refresh canvas
        canvas.draw()

    def connectToSerial(self):
        try:
            port = self.serialPort_CB.currentText().split(':')[0].strip()
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
        if len(data) == 1:
            return
        try:
            self.altitude_SB.setValue(data['altitude'])
            self.pressure_SB.setValue(data['pressure'])
            self.temperature_SB.setValue(data['temperature'])

            self.latitude_SB.setValue(data['latitude'])
            self.longitude_SB.setValue(data['longitude'])
            self.altitudeGPS_SB.setValue(data['altGPS'])

            self.ax_SB.setValue(data['AccX'])
            self.ay_SB.setValue(data['AccY'])
            self.az_SB.setValue(data['AccZ'])

            self.gyroX_SB.setValue(data['GyrX'])
            self.gyroY_SB.setValue(data['GyrY'])
            self.gyroZ_SB.setValue(data['GyrZ'])

            magX = data['MagX']
            magY = data['MagY']
            heading = atan2(magY, magX) * 180 / pi
            if heading < 0:
                heading += 360

            self.heading_SB.setValue(heading)
            if self.enableLogging_CB.isChecked():
                currentRow = self.dataTelemLog_TW.rowCount()

                self.dataTelemLog_TW.setRowCount(currentRow + 1)
                # Column 0: Time
                newitem = QTableWidgetItem(time.strftime("%H:%M:%S", time.localtime()))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 0, newitem)
                # Column 1: Altitude
                newitem = QTableWidgetItem(str(data['altitude']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 1, newitem)
                # Column 2: GPS Latitude
                newitem = QTableWidgetItem(str(data['latitude']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 2, newitem)
                # Column 3: GPS Longitude
                newitem = QTableWidgetItem(str(data['longitude']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 3, newitem)
                # Column 4: Heading
                newitem = QTableWidgetItem(str(round(heading,1)))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 4, newitem)
                self.dataTelemLog_TW.resizeColumnsToContents()

                self.dataTelemLog_TW.show()

                #currentColumn = self.dataTelemLog_TW.columnCount() + 1

            fig = self.PLOT_FIGURES['plotA']['fig']
            ax = fig.gca()
            xdata = ax.lines[0].get_xdata()
            ydata = ax.lines[0].get_ydata()

            ax.lines[0].set_xdata(np.append(xdata, data['longitude']))
            ax.lines[0].set_ydata(np.append(ydata, data['latitude']))
            self.PLOT_FIGURES['plotA']['canvas'].draw()
        except Exception as e:
            print('[GUI UPDATE & LOGGING] An exception has occured: '+str(e))
            pass

    def copyGPSToClipboard(self):
        cb = QApplication.clipboard()
        cb.clear(mode=QClipboard.Mode.Clipboard)
        text = str(self.latitude_SB.value()) + ',' + str(self.longitude_SB.value())
        cb.setText(text, mode=QClipboard.Mode.Clipboard)


    def startGPSAccEval(self ):
        try:
            self.GPSEvaluatorThread = QThread()
            # Step 3: Create a worker object
            self.GPSEvaluatorWorker = GPSEvaluatorWorker(self)
            # Step 4: Move worker to the thread
            self.GPSEvaluatorWorker.moveToThread(self.GPSEvaluatorThread)
            # Step 5: Connect signals and slots
            self.GPSEvaluatorThread.started.connect(self.GPSEvaluatorWorker.run)
            self.GPSEvaluatorWorker.finished.connect(self.GPSEvaluatorThread.quit)
            self.GPSEvaluatorWorker.finished.connect(self.GPSEvaluatorWorker.deleteLater)
            self.GPSEvaluatorThread.finished.connect(self.GPSEvaluatorThread.deleteLater)
            self.GPSEvaluatorWorker.data.connect(self.doGPSAccEval)
            #self.GPSEvaluatorWorker.progress.connect(self.progressBar)
            # Step 6: Start the thread
            self.GPSEvaluatorThread.start()

            # Final resets
            self.beginGPSAccuracy_PB.setEnabled(False)
            self.GPSEvaluatorThread.finished.connect(
                lambda: self.beginGPSAccuracy_PB.setEnabled(True)
            )
            #self.gpsEvalRequested.emit()
        except Exception as e:
            print('[GPS Eval] Exception has occured: ' + str(e))
    def doGPSAccEval(self, data):
        try:
            xdata = self.GPSEvaluatorWorker.coords['long']
            ydata = self.GPSEvaluatorWorker.coords['lat']

            if len(xdata) > 2:
                lat = data['latitude']
                long = data['longitude']

                fig = self.PLOT_FIGURES['GPS_Acc']['fig']
                ax = fig.gca()

                long_mean = np.mean(xdata)
                lat_mean = np.mean(ydata)

                long_dists = np.array([])
                lat_dists = np.array([])
                for i in range(len(xdata)):
                    dists = distCoordsComponentsSigned([lat_mean, long_mean], [ydata[i], xdata[i]], data['altGPS'])
                    long_dists = np.append(long_dists, dists[0])
                    lat_dists = np.append(lat_dists, dists[1])


                dists_std = np.sqrt(np.std(lat_dists)**2 + np.std(long_dists)**2)
                #F = 50
                #CEP = dists_std*np.sqrt(-2*np.log(1-F/100))/np.sqrt(2)
                CEP = 0.59*(np.std(lat_dists)+ np.std(long_dists))
                CEP_Circle = plt.Circle((0, 0), CEP, color='g', fill=False)

                r95 = 2*dists_std
                r95 = 2.08 * CEP
                r95_Circle = plt.Circle((0, 0), r95, color='b', fill=False)

                ax.lines[0].set_xdata(long_dists)
                ax.lines[0].set_ydata(lat_dists)
                [p.remove() for p in reversed(ax.patches)]
                ax.add_patch(CEP_Circle)
                ax.add_patch(r95_Circle)

                self.GPSEvalCEP_SP.setValue(CEP)
                self.GPSEvalR95_SP.setValue(r95)

                if len(xdata)>1 and len(ydata)>1:
                    ax.set_xlim( -r95 - 0.25, r95 + 0.25)
                    ax.set_ylim( -r95 - 0.25, r95 + 0.25)

                ax.xaxis.set_major_locator(LinearLocator(numticks=7))
                ax.xaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
                ax.yaxis.set_major_locator(LinearLocator(numticks=7))
                ax.yaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
                self.PLOT_FIGURES['GPS_Acc']['canvas'].draw()

        except Exception as e:
            print('[GPS Eval] Exception has occured: ' + str(e))

    def transmitTest(self):

        self.serialReaderObj.tx_buf = 'testtesttest\0'

    def refreshComPorts(self):
        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())
    def setSignals(self):
        self.connectSerialButton.clicked.connect(lambda: self.connectToSerial())
        self.copycoordinates_TB.clicked.connect(lambda: self.copyGPSToClipboard())
        self.beginGPSAccuracy_PB.clicked.connect(lambda: self.startGPSAccEval())
        self.refresh_COM_TB.clicked.connect(lambda: self.refreshComPorts())
        self.pushButton.clicked.connect(lambda: self.transmitTest())




if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = UI_MW()
    win.show()
    sys.exit(app.exec())
