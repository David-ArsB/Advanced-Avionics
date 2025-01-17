import sys, time
import serial
from copy import deepcopy
from Modules.GPSFuncs import distCoordsComponentsSigned
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QTableWidgetItem, QMessageBox, QWidget, QColorDialog)
from PyQt6.QtGui import QClipboard
from PyQt6.QtCore import Qt, QSettings, QDir, QThread, QObject, pyqtSignal as Signal, pyqtSlot as Slot


import matplotlib.pyplot as plt
from matplotlib.ticker import (FormatStrFormatter, LinearLocator)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from gui.gui_gcs import Ui_MainWindow
from gui.Windows.AltitudeDisplay import Ui_AltitudeDisplay

import serial.tools.list_ports
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

# pip install PyQt6-WebEngine required

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


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
                    time.sleep(0.10)
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

                # print(n)

            except Exception as e:
                print('[GPS Eval] Exception has occured: ' + str(e))

        self.finished.emit()

class SerialReaderObj(QObject):
    finished = Signal()
    serialBroadcast = Signal(dict)

    def __init__(self, port):
        super().__init__(None)
        self.serialPort = port
        self.thread = None
        self.run = True
        self.data = {}
        self.tx_buf = []

    def writeToSerial(self):
        if len(self.tx_buf) != 0:
            tx_buf = self.tx_buf
        else:
            tx_buf = ['\0']

        print(tx_buf)
        for tx in tx_buf:
            self.serialPort.write(tx.encode())
        self.tx_buf = []

    @Slot()
    def readSerial(self):
        data = {}
        dataTagOld = 1
        data['tag'] = dataTagOld
        t1 = time.time()
        writes = 0

        while self.run:

            messages = []
            done = False
            # Wait for bytes to enter the serial port and register incoming messages
            while not done:
                if self.serialPort.in_waiting:
                    inLine = self.serialPort.readline()
                    try:
                        inLine = inLine.decode().strip()
                        messages.append(inLine)
                        if messages[-1].find("EOF") != -1:
                            done = True

                    except:
                        print('Failed to decode: ')
                        print(inLine)

            # Process each line into a data dictionary
            for line in messages:
                message = line.split(':')

                if message[0] == 'BOF':
                    tag = data['tag']
                    data = {}
                    data['tag'] = tag + 1
                    data['CONF'] = []
                    #self.writeToSerial()
                    #writes += 1

                elif message[0] == 'TEMP_BARO':
                    data['TEMP_BARO'] = float(message[1].strip())

                elif message[0] == 'PRESS_BARO':
                    data['PRESS_BARO'] = float(message[1].strip())

                elif message[0] == 'ALT_BARO':
                    data['ALT_BARO'] = float(message[1].strip())

                elif message[0] == 'GPS_LAT':
                    data['GPS_LAT'] = float(message[1].strip())

                elif message[0] == 'GPS_LONG':
                    data['GPS_LONG'] = float(message[1].strip())

                elif message[0] == 'GPS_ALT':
                    data['GPS_ALT'] = float(message[1].strip())

                elif message[0] == 'LOCPOS':
                    data['locN'] = float(message[1].split(',')[0])
                    data['locE'] = float(message[1].split(',')[1])
                    print([data['locN'],data['locE']])

                elif message[0] == 'Acc' or message[0] == 'Gyr' or message[0] == 'Mag':
                    data[message[0] + 'X'] = float(message[1].split(',')[0].strip())
                    data[message[0] + 'Y'] = float(message[1].split(',')[1].strip())
                    data[message[0] + 'Z'] = float(message[1].split(',')[2].strip())

                elif message[0].find('STATUS') != -1:
                    data['STATUS'] = message[1].strip()

                elif message[0].find('CONF') != -1:
                    data['CONF'].append(message[1].strip())

                elif message[0].find('RecvOk') != -1:
                    data['RecvOk'] = float(message[1].strip())

                elif message[0] == 'EOF':
                    # EOF is confirmed
                    self.serialBroadcast.emit(deepcopy(data))
                    print('Writing to Serial...\n')
                    self.writeToSerial()

                else:
                    try:
                        if is_number(message[1]):
                            data[message[0]] = float(message[1])
                        else:
                            data[message[0]] = message[1]
                    except:
                        data['Unknown'] = line

        self.finished.emit()

class AltitudeDisplay(QWidget, Ui_AltitudeDisplay):
    def __init__(self):
        super(AltitudeDisplay, self).__init__()
        self.setupUi(self)
        self.setWindowTitle("Intructions")

class UI_MW(QMainWindow, Ui_MainWindow):
    serialStartRequested = Signal()
    gpsEvalRequested = Signal()

    def __init__(self, app_name='GUIPreliminaryTool', parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.settings = QSettings("__settings.ini",QSettings.Format.IniFormat)
        self.workingDirectory = QDir.current().currentPath()
        self.success_rate = 0
        self.error_rate = 0
        self.msgs_wainting_for_conf = []

        #self.AltitudeDisplay = AltitudeDisplay()
        #self.AltitudeDisplay.show()

        # Initialize some values
        self.init_def_values()
        self.serialReaderObj = None
        self.serialReaderThread = None
        self.serialPort = None
        self.drone_pada = None

        # Other random initialisations
        self.stackedWidget.setCurrentIndex(0)
        self.dataTelemLog_TW.setHorizontalHeaderLabels(['Time', 'Altitude', 'Latitude', 'Longitude', 'Heading'])
        self.dataTelemLog_TW.resizeColumnsToContents()
        self.loggingChecked()
        self.missionChanged()

        # Initialize plotting area
        self.PLOT_FIGURES = {}
        self.dispose = 0
        self.setNewFigure('plotA', self.gridLayout_plotA, True)
        #self.setNewFigure('GPS_Acc', self.gridLayout_plotB, True)
        #self.setNewFigure('Altitude', self.gridLayout_plotC, True)
        #fig = self.PLOT_FIGURES['Altitude']['fig']
        #ax = fig.gca()
        #ax.plot([], [], '-o')
        #ax.grid()
        #self.PLOT_FIGURES['Altitude']['canvas'].draw()

        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())
        self.serialPort_CB.setCurrentIndex(1)
        self.serialPortSiK_CB.clear()
        self.serialPortSiK_CB.addItems(self.serial_ports())
        self.serialPortSiK_CB.setCurrentIndex(1)
        self.setSignals()

        #c = (45.517449, -73.784236)
        #m = folium.Map(
        #    title='GPS Coordinates',
        #    zoom_start=13,
        #    location=c
        #)
        #data = io.BytesIO()
        #m.save(data,close_file=False)
        #self.webEngineView.setHtml(data.getvalue().decode())
        #self.map = m

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

        #fig = self.PLOT_FIGURES['GPS_Acc']['fig']
        #ax = fig.gca()
        #ax.plot([], [], 'ro')
        #ax.grid()
        #self.PLOT_FIGURES['GPS_Acc']['canvas'].draw()


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

    def connectToSiK(self):
        serialToArduino = self.serialPort_CB.currentText()
        serialToSik = self.serialPortSiK_CB.currentText()

        if serialToArduino == serialToSik:
            message = "Connection error. SiK Radio COM Port cannot be equal to Arduino COM Port."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Information)
            msgBox.setText(message)
            msgBox.setWindowTitle('Success')
            msgBox.exec()
            return False
        newPort = serialToSik.split(':')[0].strip()
        try:
            # Setup Serial connection with the SiK Telemetry Radio
            if self.drone_pada is not None:
                self.drone_pada.close()
                self.drone_pada = None

            # Connect
            self.drone_pada = connect(newPort, wait_ready=False, baud=57600, heartbeat_timeout=120)
            self.drone_pada.wait_ready(True, timeout=120)

            # Print Some Parameters
            connectionKey = "Connection To PADA Succesfull.\n\nAutopilot Firmware version: %s\n" % self.drone_pada.version
            for key, value in self.drone_pada.parameters.iteritems():
                connectionKey += " - Key:%s Value:%s\n" % (key, value)

            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(connectionKey)
            msgBox.setWindowTitle('Success')
            msgBox.exec()

            cmds = self.drone_pada.commands
            # Call clear() on Vehicle.commands and upload the command to the vehicle.
            cmds.clear()
            cmds.upload()
            return True

        except Exception as e:
            message = "Connection Failed:\n" + str(e)
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()
            return False

    def connectToSerial(self):
        serialToArduino = self.serialPort_CB.currentText()
        serialToSik = self.serialPortSiK_CB.currentText()

        if serialToArduino == serialToSik:
            message = "Connection error. SiK Radio COM Port cannot be equal to Arduino COM Port."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Information)
            msgBox.setText(message)
            msgBox.setWindowTitle('Success')
            msgBox.exec()
            return False

        try:
            # Setup Serial connection with the Arduino
            newPort = self.serialPort_CB.currentText().split(':')[0].strip()
            if self.serialPort == None or self.serialPort.port != newPort:
                if self.serialPort != None:
                    self.serialPort.close()

                port = newPort
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
                self.serialReaderObj.moveToThread(self.serialReaderThread)

                self.serialReaderObj.finished.connect(self.serialReaderThread.quit)
                self.serialReaderObj.finished.connect(self.serialReaderObj.deleteLater)
                self.serialReaderThread.finished.connect(self.serialReaderThread.deleteLater)
                self.serialStartRequested.connect(self.serialReaderObj.readSerial)
                self.serialReaderThread.start()
                self.serialStartRequested.emit()
                self.connectSerialButton.setText('Disconnect')

                message = "Connection successful."
                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Information)
                msgBox.setText(message)
                msgBox.setWindowTitle('Success')
                msgBox.exec()
                return True

            elif self.serialPort.is_open:

                self.serialReaderObj.run = False
                # Stop the previous thread and reset
                self.serialReaderThread.exit()
                self.serialReaderThread.wait()
                self.serialReaderObj = None
                self.serialReaderThread = None
                self.serialPort.close()
                self.serialPort = None
                self.connectSerialButton.setText('Connect')

                message = "Disconnection successful."
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
        error = []

        try:
            if 'STATUS' in data: # PA status information
                if data['STATUS'].find('@ARMED') != -1:
                    if self.PAstat_LE.text() == '@STANDBY':
                        self.PAstat_LE.setText(data['STATUS'])
                        self.armPA_PB.setEnabled(False)
                        self.stdbPA_PB.setEnabled(True)

                elif data['STATUS'].find('@STANDBY') != -1:
                    if self.PAstat_LE.text() == '@ARMED':
                        self.PAstat_LE.setText(data['STATUS'])
                        self.armPA_PB.setEnabled(True)
                        self.stdbPA_PB.setEnabled(False)
                    return None

            # Check if any command confirmation data has been received from the PA
            # This confirms that the appropriate command has been received
            # If a message confirmation is missing, the message will be broadcast again until reception.
            for conf in data['CONF']:
                if conf.find('@ARMED') != -1:
                    if '$ARMED' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$ARMED')
                elif conf.find('@STANDBY') != -1:
                    if '$STANDBY' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$STANDBY')
                elif conf.find('@RELEASE') != -1:
                    if '$RELEASE' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$RELEASE')
                elif conf.find('@SET_TARGET_COLOR') != -1:
                    if '$SET_TARGET_COLOR' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$SET_TARGET_COLOR')
                elif conf.find('@SET_MISSION_TYPE') != -1:
                    if '$SET_MISSION_TYPE' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$SET_MISSION_TYPE')

            # Repeat unheard messages.
            for msg in self.msgs_wainting_for_conf:
                self.transmitCommand(msg)

            if 'RecvOk' in data: # PA message reception rate (one message expected per PA loop)
                self.PAReceptionRate_DSB.setValue(data['RecvOk'])

            if 'ALT_BARO' in data:
                self.altitude_SB.setValue(data['ALT_BARO'])
                self.altitude_SB_2.setValue(data['ALT_BARO'])
            else:
                error.append('ALT_BARO')

            if 'PRESS_BARO' in data:
                self.pressure_SB.setValue(data['PRESS_BARO'])
            else:
                error.append('PRESS_BARO')

            if 'TEMP_BARO' in data:
                self.temperature_SB.setValue(data['TEMP_BARO'])
            else:
                error.append('TEMP_BARO')

            if 'GPS_LAT' in data and 'GPS_LONG' in data:
                self.latitude_SB.setValue(data['GPS_LAT'])
                self.longitude_SB.setValue(data['GPS_LONG'])
            else:
                error.append('GPS_POS')

            if 'GPS_ALT' in data:
                self.altitudeGPS_SB.setValue(data['GPS_ALT'])
            else:
                error.append('GPS_ALT')

            if 'AccX' in data and 'AccY' in data and 'AccZ' in data:
                self.ax_SB.setValue(data['AccX'])
                self.ay_SB.setValue(data['AccY'])
                self.az_SB.setValue(data['AccZ'])
            else:
                error.append('Acc')

            if 'GyrX' in data and 'GyrY' in data and 'GyrZ' in data:
                self.gyroX_SB.setValue(data['GyrX'])
                self.gyroY_SB.setValue(data['GyrY'])
                self.gyroZ_SB.setValue(data['GyrZ'])
            else:
                error.append('Gyr')

            if 'Heading' in data :
                heading = data['Heading']
                self.heading_SB.setValue(heading)
                #print(heading)

            else:
                error.append('Heading')

            if self.enableLogging_CB.isChecked() and False:
                currentRow = self.dataTelemLog_TW.rowCount()

                self.dataTelemLog_TW.setRowCount(currentRow + 1)
                # Column 0: Time
                newitem = QTableWidgetItem(time.strftime("%H:%M:%S", time.localtime()))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 0, newitem)
                # Column 1: Altitude
                newitem = QTableWidgetItem(str(data['ALT_BARO']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 1, newitem)
                # Column 2: GPS Latitude
                newitem = QTableWidgetItem(str(data['GPS_LAT']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 2, newitem)
                # Column 3: GPS Longitude
                newitem = QTableWidgetItem(str(data['GPS_LONG']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 3, newitem)
                # Column 4: Heading
                newitem = QTableWidgetItem(str(round(heading, 1)))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 4, newitem)
                #self.dataTelemLog_TW.resizeColumnsToContents()

                #self.dataTelemLog_TW.show()

                #currentColumn = self.dataTelemLog_TW.columnCount() + 1

                fig = self.PLOT_FIGURES['plotA']['fig']
                ax = fig.gca()
                xdata = ax.lines[0].get_xdata()
                ydata = ax.lines[0].get_ydata()

                ax.lines[0].set_xdata(np.append(xdata, data['longitude']))
                ax.lines[0].set_ydata(np.append(ydata, data['latitude']))
                self.PLOT_FIGURES['plotA']['canvas'].draw()
                self.PLOT_FIGURES['plotA']['canvas'].flush_events()

                #fig = self.PLOT_FIGURES['Altitude']['fig']
                #ax = fig.gca()
                #xdata = ax.lines[0].get_xdata()
                #ydata = ax.lines[0].get_ydata()
                #if len(xdata) == 0:
                #    ax.lines[0].set_xdata([1])
                #else:
                #    ax.lines[0].set_xdata(np.append(xdata, xdata[-1]+1))
                #ax.lines[0].set_ydata(np.append(ydata, data['altitude']))
                #ax.relim()
                #ax.autoscale_view()
                #self.PLOT_FIGURES['Altitude']['canvas'].draw()
                #self.PLOT_FIGURES['Altitude']['canvas'].flush_events()

            #fig = self.PLOT_FIGURES['plotA']['fig']
            #ax = fig.gca()
            #xdata = ax.lines[0].get_xdata()
            #ydata = ax.lines[0].get_ydata()
            #ax.lines[0].set_xdata(np.append(xdata, data['GPS_LONG']))
            #ax.lines[0].set_ydata(np.append(ydata, data['GPS_ALT']))
            #self.PLOT_FIGURES['plotA']['canvas'].draw()
            #self.PLOT_FIGURES['plotA']['canvas'].flush_events()

            #self.success_rate += 1

            print('Success Rate: '+str((self.success_rate)/(self.success_rate+self.error_rate))+'\n')

        except Exception as e:
            print('[GUI UPDATE & LOGGING] An exception has occured: '+str(e))
            print('Values not accessible: ' + str(error))
            print(data)
            self.error_rate += 1
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

    def color_picker(self):
        color = QColorDialog.getColor()
        self.colorId_LE.setStyleSheet("QLineEdit { background-color: %s}" % color.name())
        self.TARGET_COLOR = color
        # Print color value in lineedit

    def transmitCommand(self, com):
        try:
            if (com + '\0') not in self.serialReaderObj.tx_buf:
                self.serialReaderObj.tx_buf.append(com)

                if com == '$ARM':
                    self.armPA_PB.setEnabled(False)
                    self.stdbPA_PB.setEnabled(True)
                    if '$ARM' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$ARM')

                        if self.missionType_CB.currentText() == 'Static Target':
                            self.transmitCommand('$SET_MISSION_TYPE:STATIC')

                        elif self.missionType_CB.currentText() == 'Random Target':
                            color = self.TARGET_COLOR
                            self.transmitCommand('$SET_MISSION_TYPE:RANDOM')
                            self.transmitCommand('$SET_TARGET_COLOR:0')

                elif com == '$STANDBY':
                    self.stdbPA_PB.setEnabled(False)
                    self.armPA_PB.setEnabled(True)
                    if '$STANDBY' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$STANDBY')

                elif com == '$RELEASE':
                    if '$RELEASE' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$RELEASE')

                elif com.find('$SET_MISSION_TYPE') != -1:
                    if '$SET_MISSION_TYPE' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$SET_MISSION_TYPE')

                elif com.find('$SET_TARGET_COLOR') != -1:
                    if '$SET_TARGET_COLOR' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$SET_TARGET_COLOR')

        except:
            message = "No serial port selected, please select a COM port."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()

    def refreshComPorts(self):
        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())

    def loggingChecked(self):
        if not self.enableLogging_CB.isChecked():
            self.dataTelemLog_TW.setRowCount(0)

            fig = self.PLOT_FIGURES['plotA']['fig']
            ax = fig.gca()
            ax.lines[0].set_xdata(np.array([]))
            ax.lines[0].set_ydata(np.array([]))
            self.PLOT_FIGURES['plotA']['canvas'].draw()

            #fig = self.PLOT_FIGURES['Altitude']['fig']
            #ax = fig.gca()
            #ax.lines[0].set_xdata(np.array([]))
            #ax.lines[0].set_ydata(np.array([]))
            #self.PLOT_FIGURES['Altitude']['canvas'].draw()

    def releasePADA(self):
        if self.drone_pada is not None and self.serialPort is not None and self.serialPort.is_open:
            # ===============#
            # SETUP MISSION #
            # ===============#
            # Pertinent links: https://ardupilot.org/plane/docs/arming-your-plane.html
            # Get commands object from Vehicle.
            cmds = self.drone_pada.commands
            cmds.upload()

            # ===============#
            # START MISSION #
            # ===============#
            # Set mode to AUTO to start mission
            self.drone_pada.mode = VehicleMode("AUTO")

            # Release the PADA from the PA
            self.transmitCommand('$RELEASE')

        else:
            message = "Error: one or more peripherals are missing. Please confirm that the SiK Telemetry Radio and the Arduino are both connected."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()

    def missionChanged(self):
        if self.missionType_CB.currentText() == 'Static Target':
            self.colorPicker_PB.setEnabled(False)
            self.colorId_LE.setEnabled(False)
            self.targetLatitude_SB.setReadOnly(False)
            self.targetLongitude_SB.setReadOnly(False)
        elif self.missionType_CB.currentText() == 'Random Target':
            self.colorPicker_PB.setEnabled(True)
            self.colorId_LE.setEnabled(True)
            self.targetLatitude_SB.setReadOnly(True)
            self.targetLongitude_SB.setReadOnly(True)
    def setSignals(self):
        self.connectSerialButton.clicked.connect(lambda: self.connectToSerial())
        self.connectToSiKButton.clicked.connect(lambda: self.connectToSiK())
        self.copycoordinates_TB.clicked.connect(lambda: self.copyGPSToClipboard())
        self.beginGPSAccuracy_PB.clicked.connect(lambda: self.startGPSAccEval())
        self.refresh_COM_TB.clicked.connect(lambda: self.refreshComPorts())
        self.enableLogging_CB.stateChanged.connect(lambda: self.loggingChecked())
        self.missionType_CB.currentIndexChanged.connect(lambda : self.missionChanged())
        self.colorPicker_PB.clicked.connect(lambda: self.color_picker())

        # Commands
        self.releasePADA_PB.clicked.connect(lambda: self.releasePADA())
        self.armPA_PB.clicked.connect(lambda: self.transmitCommand('$ARM'))
        self.stdbPA_PB.clicked.connect(lambda: self.transmitCommand('$STANDBY'))
        self.resetPA_PB.clicked.connect(lambda: self.transmitCommand('$RESET'))
        self.calibrateAltimeter_PB.clicked.connect(lambda: self.transmitCommand('$CAL_ALTIMETER'))
        self.calibrateGPS_PB.clicked.connect(lambda: self.transmitCommand('$CAL_GPS'))


def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)

if __name__ == "__main__":
    sys.excepthook = except_hook
    app = QApplication(sys.argv)
    win = UI_MW()
    win.show()
    sys.exit(app.exec())
