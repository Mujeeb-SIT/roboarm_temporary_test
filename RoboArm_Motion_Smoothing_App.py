#TODO:
#GUI main window does not move/resize when camera is on (unless there is a big delay in the threads run method)

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import QtQuickWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QTableWidget,QTableWidgetItem, QDialog, QMessageBox, QMainWindow
from PyQt5.QtGui import QIcon,QPixmap 

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys
from pathlib import Path
import ntpath, os, glob
import cv2
import time
import datetime
import csv
import socket
from XOR_CheckSum import xor_checksum_string
import threading
import keyboard #pip install keyboard

from RoboArm_Motion_Smoothing_GUI import Ui_MainWindow
from PoseProcessThread import PoseProcessThread
from CamThread import CamQThread


#global configuration variables


g_inputMode = 0; #1=Mouse, 2-Camera
g_cameraOff = True
g_poseMode = -1 #1=Arm, 2-Wrist, 3=both....
g_wristEnabled = True
g_angleEnabled = True
g_poseProcessPaused = False
g_camera_fps = 15  #approximately
g_pose_fps = 10 #regardless of (depth)camera fps, poseProcess thread will use its onw delay
g_depthCamera_fps = 15 #not interfaced  yet
g_armMT = 0
g_wristMT = 0
g_QSize = 1
#g_robotIP = '192.168.250.4' #used with direct ethernet 
g_robotIP = '192.168.' #with Nokia 5G Sim, we used Netgear on robot side
							#the above is the IP address with port forwarding on the robot arm
#mec_server_ip = 192.168.254.11  #only used to do ping test the notkia 5G network
#robo arm port is 5890 ()

#Internal global variables
g_mouse_frame_w = -1
g_mouse_frame_h = -1
g_camera_frame_w = -1
g_camera_frame_h = -1
g_armAngle = -1
g_wristAngle = -1
g_AppObj = None
g_FormObj = None
g_robotConnectedStatus = False

#thread handlers
g_qPoseProcessThread = None
g_qcameraThread = None
g_keyboard_thread = None


#class MyForm(QDialog):
class MyForm(QMainWindow):
	def __init__(self):
		super().__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)

		#FCT-(extra code starts here)======================================
		self.initProgram()  #Create two functions initGUI, and initVars


	def initProgram(self):
		#global variable initialization
		global g_mouse_frame_w, g_mouse_frame_h, g_camera_frame_h, g_camera_frame_w
		global g_inputMode
		global g_wristEnabled, g_angleEnabled
		global g_camera_fps
		global g_qPoseProcessThread, g_qcameraThread, g_keyboard_thread
		global g_poseProcessPaused 
		global g_armMT, g_wristMT 
		global g_cameraOff
		global g_AppObj, g_FormObj
		global g_robotIP
		global g_QSize


		#motion threshold settings
		g_armMT = 10 #default value
		g_wristMT = 10 #default value
		g_QSize = 4

		#global variables
		g_inputMode = 1; #1=mouse, 2=camera
		g_wristEnabled = True
		g_angleEnabled = True
		g_cameraOff = True
		g_poseProcessPaused = False

		if g_inputMode == 1:
			self.ui.radioButton_Mouse.setChecked(True)
			self.ui.checkBox_CameraOff.setEnabled(False)
			self.ui.checkBox_CameraOff.setChecked(False)
			g_cameraOff = True
		else:
			self.ui.radioButton_Camera.setChecked(False)
			self.ui.checkBox_CameraOff.setEnabled(True)

		if g_angleEnabled:
			self.ui.checkBox_enableAngle.setChecked(True)
		else:
			self.ui.checkBox_enableAngle.setChecked(False)

		if 	g_wristEnabled:
			self.ui.checkBox_enableWrist.setChecked(True)
			self.ui.lineEdit_MT_Wrist.setEnabled(True)
		else:
			self.ui.checkBox_enableWrist.setChecked(False)
			self.ui.lineEdit_MT_Wrist.setEnabled(False)
		
		if g_cameraOff:
			self.ui.checkBox_CameraOff.setChecked(True)
		else:
			self.ui.checkBox_CameraOff.setChecked(False)

		if g_angleEnabled:
			self.ui.lineEdit_MT_Arm.setEnabled(True)
			self.ui.lineEdit_MT_Arm.setEnabled(True)
		else:
			self.ui.lineEdit_MT_Arm.setEnabled(False)
			self.ui.lineEdit_MT_Arm.setEnabled(False)
		
		self.ui.lineEdit_MT_Arm.setText(str(g_armMT))
		self.ui.lineEdit_MT_Wrist.setText(str(g_wristMT))
		self.ui.lineEdit_QSize.setText(str(g_QSize))
		
		if g_poseProcessPaused:
			self.ui.checkBox_Pause_PoseProcess.setChecked(True)
			self.ui.lineEdit_MT_Arm.setEnabled(True)
			self.ui.lineEdit_MT_Wrist.setEnabled(True)
		else:
			self.ui.checkBox_Pause_PoseProcess.setChecked(False)
			self.ui.lineEdit_MT_Arm.setEnabled(False)
			self.ui.lineEdit_MT_Wrist.setEnabled(False)

		g_mouse_frame_w = self.ui.frame.width()
		g_mouse_frame_h = self.ui.frame.height()
		g_camera_frame_w = self.ui.label_CameraFrame.width()
		g_camera_frame_h = self.ui.label_CameraFrame.height()
		self.ui.pushButton_Move2InitialPos.setEnabled(False) #enable only when robot is connected
		self.ui.pushButton_Move2FinalPos.setEnabled(False) #enable only when robot is connected

		self.ui.frame.setStyleSheet("QFrame {background-color: rgb(200, 255, 255);"
                                "border-width: 2;"
                                "border-radius: 1;"
                                "border-style: solid;"
                                "border-color: rgb(10, 10, 10)}"
                                )

		self.ui.label_RobotConnected.setStyleSheet("color: red") 

		self.ui.frame.mousePressEvent = self.mousePressMove
		self.ui.frame.mouseMoveEvent = self.mousePressMove
		self.ui.frame.mouseReleaseEvent = self.mouseReleaseEvent
		self.ui.frame.installEventFilter(self)
		
		self.ui.checkBox_enableAngle.stateChanged.connect(self.State_Angle_Enabled)
		self.ui.checkBox_enableWrist.stateChanged.connect(self.state_Wrist_Enabled)
		self.ui.checkBox_Pause_PoseProcess.stateChanged.connect(self.State_Pause_PoseProcess)
		self.ui.checkBox_CameraOff.stateChanged.connect(self.State_CameraOff)

		self.ui.lineEdit_MT_Arm.textChanged.connect(self.MTchanged)
		self.ui.lineEdit_MT_Wrist.textChanged.connect(self.MTchanged)
		self.ui.lineEdit_QSize.textChanged.connect(self.lineEdit_QSizeChanged)


		self.ui.radioButton_Mouse.clicked.connect(self.InputSelectRadio)
		self.ui.radioButton_Camera.clicked.connect(self.InputSelectRadio)
		self.ui.pushButton_Move2InitialPos.clicked.connect(self.MoveRoboArm2InitPos)
		self.ui.pushButton_Move2FinalPos.clicked.connect(self.pushButton_Move2FinalPos)
		self.ui.pushButton_MoveTest.clicked.connect(self.pushButton_MoveTest)

		
		#menu items
		self.ui.actionConnect.triggered.connect(self.ConnectRobot)  

		g_qPoseProcessThread = PoseProcessThread(g_pose_fps, g_robotIP)
		g_qPoseProcessThread.started.connect(self.PoseProcessStartedSlot)
		g_qPoseProcessThread.dataReceivedGuiUpdate.connect(self.DataRecvdGuiUpdateSlot)
		g_qPoseProcessThread.robotConnected.connect(self.RobotConnectedSlot)
		
		g_qcameraThread = CamQThread(g_camera_frame_w, g_camera_frame_h, g_camera_fps, cameraID=0)
		g_qcameraThread.started.connect(self.CameraStartedSlot)
		g_qcameraThread.frameShow.connect(self.CameraFrameShowSlot)

		self.ui.verticalSlider.valueChanged.connect(self.SliderValuechanged)

		
		g_qPoseProcessThread.start()
		g_qcameraThread.start()

		#initialization of parameters of threads (such as motion threshold, queue size etc)
		g_qPoseProcessThread.updateMT(g_armMT, g_wristMT)
		g_qPoseProcessThread.updateQSize(g_QSize)
		
		time.sleep(0.1) #give the threads time to start
		if (not g_poseProcessPaused) and (g_inputMode == 1):
			g_qPoseProcessThread.resume()
			g_qcameraThread.pause()


		#keyboard_thread = threading.Thread(target=keyboard_process,)
		'''
		g_keyboard_thread = threading.Thread(target=keyboard_process,args=(g_AppObj,g_FormObj))
		g_keyboard_thread.start()
		'''


	def SliderValuechanged(self):
		global g_angleEnabled, g_wristEnabled, g_poseProcessPaused,  g_armAngle, g_wristAngle
		global g_qPoseProcessThread

		g_armAngle = self.ui.verticalSlider.value()
		
		if (not g_angleEnabled):
			g_armAngle = -1

		if (not g_wristEnabled):
			g_wristAngle = -1

		#if (not g_poseProcessPaused): 
		print("slider cahnged...g_armAngle=",g_armAngle)
		g_qPoseProcessThread.dataUpdate(g_armAngle, g_wristAngle)
		

	def pushButton_MoveTest(self):
		global g_qPoseProcessThread
		
		startVal = self.ui.lineEdit_StartAngle.text()
		endVal = self.ui.lineEdit_EndAngle.text()
		stepVal = self.ui.lineEdit_AngleStepSize.text()

		g_qPoseProcessThread.moveTest(startVal,endVal,stepVal)


	def MoveRoboArm2InitPos(self):
		global g_robotConnectedStatus, g_qPoseProcessThread
		if g_robotConnectedStatus == True:
			i = g_qPoseProcessThread.SetRoboArm2EndPosition(endPos =0)
			if i != 0:
				print("Error: MoveRoboArm2InitPos() received error value:",i)

	def pushButton_Move2FinalPos(self):
		global g_robotConnectedStatus, g_qPoseProcessThread
		if g_robotConnectedStatus == True:
			i = g_qPoseProcessThread.SetRoboArm2EndPosition(endPos =1)
			if i != 0:
				print("Error: pushButton_Move2FinalPos() received error value:",i)



	def RobotConnectedSlot(self, result):
		global g_robotConnectedStatus
		print("RobotConnectedSlot() received")
		if (result == True):
			g_robotConnectedStatus = True
			self.ui.label_RobotConnected.setText("Robot arm conncted")
			self.ui.label_RobotConnected.setStyleSheet("color: green") 
			self.ui.actionConnect.setText("Disconnect")
			self.ui.pushButton_Move2InitialPos.setEnabled(True)
			self.ui.pushButton_Move2FinalPos.setEnabled(True)
		else:
			g_robotConnectedStatus = False
			self.ui.label_RobotConnected.setText("Connection Failed")
			self.ui.label_RobotConnected.setStyleSheet("color: red") 
			self.ui.actionConnect.setText("Connect")
			self.ui.pushButton_Move2InitialPos.setEnabled(False)
			self.ui.pushButton_Move2FinalPos.setEnabled(False)


	def ConnectRobot(self):
		global g_robotConnectedStatus, g_qPoseProcessThread

		if self.ui.actionConnect.text()=="Connect":
			g_qPoseProcessThread.ConnectRobot()
			self.ui.label_RobotConnected.setText("Connecting...Wait Please")
			#result will be recieved in the slot
		else:
			g_qPoseProcessThread.DisconnectRobot()
			g_robotConnectedStatus = False
			self.ui.label_RobotConnected.setText("Robot arm not conncted")
			self.ui.label_RobotConnected.setStyleSheet("color: red") 
			self.ui.actionConnect.setText("Connect")
			self.ui.pushButton_Move2InitialPos.setEnabled(False)
			self.ui.pushButton_Move2FinalPos.setEnabled(False)
			
	def CameraFrameShowSlot(self, CamImage):
		self.ui.label_CameraFrame.setPixmap(QPixmap.fromImage(CamImage))

	def InputSelectRadio(self):
		global g_inputMode
		global g_cameraOff
		
		if self.ui.radioButton_Mouse.isChecked():
			g_inputMode = 1 #1: mouse
			g_cameraOff = True
			g_qcameraThread.pause()
			self.ui.checkBox_CameraOff.setChecked(True)
			self.ui.checkBox_CameraOff.setEnabled(False)
		else:
			g_inputMode = 2 #2: Camera
			self.ui.checkBox_CameraOff.setEnabled(True)

		print("g_inputMode=",g_inputMode)


	def lineEdit_QSizeChanged(self):
		global g_qPoseProcessThread, g_poseProcessPaused
		global g_QSize

		if (not g_poseProcessPaused):
			print("Pause the Pose Process first before changing the QSize")
		else:
			try:
				qsize = int(self.ui.lineEdit_QSize.text())
				g_qPoseProcessThread.updateQSize(qsize)
				g_QSize = qsize
			except ValueError:
				self.ui.lineEdit_QSize.setText(str(g_QSize))
			

	def MTchanged(self):
		global g_armMT, g_wristMT
		
		print("MTchanged_wrist")
		try:
			armMT = int(self.ui.lineEdit_MT_Arm.text())
			wristMT = int(self.ui.lineEdit_MT_Wrist.text())
			
			g_armMT = armMT
			g_wristMT = wristMT
			g_qPoseProcessThread.updateMT(g_armMT, g_wristMT)
			
		except ValueError:
			self.ui.lineEdit_MT_Wrist.setText(str(g_wristMT))
			self.ui.lineEdit_MT_Arm.setText(str(g_armMT))


	def CameraStartedSlot(self):
		print("CameraStartedSlot ()")

	def PoseProcessStartedSlot(self):
		print("PoseProcessStartedSlot ()")

	def DataRecvdGuiUpdateSlot(self, armAngle, wristAngle):
		print("DataRecvdGuiUpdateSlot CALLED")
		self.ui.label_2.setText(str(armAngle))
		self.ui.label_12.setText(str(wristAngle))

	def State_CameraOff(self):
		global g_cameraOff, g_qcameraThread

		if self.ui.checkBox_CameraOff.isChecked():
			g_cameraOff = True #camera off
			g_qcameraThread.pause()

		else:
			g_cameraOff = False # camera on
			g_qcameraThread.resume()


	def State_Pause_PoseProcess(self):
		global g_qPoseProcessThread, g_poseProcessPaused
		
		if self.ui.checkBox_Pause_PoseProcess.isChecked():
			g_poseProcessPaused = True
			g_qPoseProcessThread.pause()
			self.ui.lineEdit_MT_Arm.setEnabled(True)
			self.ui.lineEdit_MT_Wrist.setEnabled(True)

		else:
			g_poseProcessPaused = False
			g_qPoseProcessThread.resume()
			self.ui.lineEdit_MT_Arm.setEnabled(False)
			self.ui.lineEdit_MT_Wrist.setEnabled(False)
			

	def State_Angle_Enabled(self):
		global g_angleEnabled
		if self.ui.checkBox_enableAngle.isChecked():
			g_angleEnabled = True
		else:
			g_angleEnabled = False
		print("g_angleEnabled and g_wristEnabled=",g_angleEnabled, g_wristEnabled)

	def state_Wrist_Enabled(self):
		global g_wristEnabled
		if self.ui.checkBox_enableWrist.isChecked():
			g_wristEnabled = True
		else:
			g_wristEnabled = False
		print("aaaag_angleEnabled and g_wristEnabled=",g_angleEnabled, g_wristEnabled)


	def closeEvent(self,event):
		
		self.quitAllThreads()
		sys.exit(0)	 #which one is better???
		QApplication.quit() 
		
	def eventFilter(self, obj, event):
		#if self.ui.window is obj:  #for main window events
		if self.ui.frame is obj:
			if event.type() == QtCore.QEvent.MouseButtonPress:
				self.mousePressMove(event)
			elif event.type() == QtCore.QEvent.MouseMove:
				self.mousePressMove(event)
			elif event.type() == QtCore.QEvent.MouseButtonRelease:
				self.mouseReleased(event)
		return super().eventFilter(obj, event)


	#def mouseMoveEvent(self, event):
	def mousePressMove(self, event):
		global g_mouse_frame_w, g_mouse_frame_h, g_armAngle, g_wristAngle
		global g_qPoseProcessThread, g_poseProcessPaused
		global g_wristEnabled, g_angleEnabled
		global g_inputMode

		if g_inputMode != 1: #if not mouse input, then ignore the mouse
			self.ui.frame.update()
			return

		#print("mouseMoveEvent pos=",event.pos())		
		#Find arm angle
		y = event.pos().y() #arm
		x = event.pos().x() #wrist

		if (y<=0):
			g_armAngle = 150
		elif y>g_mouse_frame_h:
			g_armAngle = 30
		else:
			range = 150-30
			g_armAngle = 30 + round ((y/g_mouse_frame_h)*range)
			g_armAngle = (-1)*g_armAngle + 180
					

		if (x<=0):
			g_wristAngle = 0
		elif x>g_mouse_frame_w:
			g_wristAngle = 90
		else:
			g_wristAngle1 = round ((x/g_mouse_frame_w)*90)
			g_wristAngle = g_wristAngle1
		
		'''
		self.ui.label_2.setText(str(g_armAngle))
		self.ui.label_12.setText(str(g_wristAngle))
		'''

		if (not g_angleEnabled):
			g_armAngle = -1
		if (not g_wristEnabled):
			g_wristAngle = -1
		if (not g_poseProcessPaused):
			g_qPoseProcessThread.dataUpdate(g_armAngle, g_wristAngle)
		self.ui.frame.update()


	#def mouseReleaseEvent(self, event):
	def mouseReleased(self, event):
		return
		print("mouseReleaseEvent pos=",event.pos())
		self.ui.frame.update()



	def quitAllThreads(self):
		global g_qPoseProcessThread, g_qcameraThread, g_keyboard_thread
		
		if (g_qcameraThread != None):
			g_qcameraThread.pause()
			time.sleep(0.1)
			g_qcameraThread.quit()
		if (g_qPoseProcessThread != None):
			g_qPoseProcessThread.pause()
			time.sleep(0.1)
			g_qPoseProcessThread.quit()
		time.sleep(0.1) #give the threads time finish 
	

if __name__=="__main__":
	#global g_AppObj, g_FormObj
	app1 = QtWidgets.QApplication(sys.argv)
	g_AppObj = app1
	w = MyForm()
	w.setWindowTitle("RoboArm Library Test Program") #
	w.setWindowFlags(QtCore.Qt.WindowCloseButtonHint | QtCore.Qt.WindowMinimizeButtonHint)
	g_FormObj = w
	w.show()
	app1.aboutToQuit.connect(w.closeEvent)  #it works here as well as in the MyForm Class...how?
	sys.exit(app1.exec_())

