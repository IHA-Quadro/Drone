
import time

from PyQt4 import QtGui
from ui.subpanel.BasePanelController import BasePanelController
from ui.subpanel.commmonitor.CommMonitorPanel import Ui_CommMonitorPanel


class CommMonitorController(QtGui.QWidget, BasePanelController):
    
    def __init__(self, vehicle_event_dispatcher, protocol_handler):
        QtGui.QWidget.__init__(self)
        BasePanelController.__init__(self)
        self.ui = Ui_CommMonitorPanel()
        self.ui.setupUi(self)
        self.ui.sendButton.setEnabled(False)
        self.ui.clearButton.setEnabled(False)
                
        # Connect GUI slots and signals
        self.ui.lineEdit.returnPressed.connect(self.sendCommand)
        self.ui.sendButton.clicked.connect(self.sendCommand)
        self.ui.clearButton.clicked.connect(self.clearComm)
        
    def start(self):
        '''This method starts a timer used for any long running loops in a subpanel'''
#        if self._communicator.isConnected() == True:
#            self.timer = QtCore.QTimer()
#            self.timer.timeout.connect(self.readContinuousData)
#            self.timer.start(50)
#            self.startCommThread()
            
    def sendCommand(self):
        command = str(self.ui.lineEdit.text())
        self._communicator.write(command)
        self.ui.commLog.append(self.timeStamp() + " -> " + command)
        self.ui.lineEdit.clear()
        time.sleep(0.150)
    
    def readContinuousData(self):
        '''This method continually reads telemetry from the AeroQuad'''
        isConnected = self._communicator.isConnected()
        self.ui.sendButton.setEnabled(isConnected)
        self.ui.clearButton.setEnabled(isConnected)
        if isConnected and not self.commData.empty():         
            self.ui.commLog.append(self.timeStamp() + " <- " + self.commData.get())
            self.ui.commLog.ensureCursorVisible()
        
    def clearComm(self):
        self.ui.lineEdit.clear()
        self.ui.commLog.clear()

