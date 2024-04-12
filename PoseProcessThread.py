'''
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import QtQuickWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QTableWidget,QTableWidgetItem, QDialog, QMessageBox, QMainWindow
from PyQt5.QtGui import QIcon,QPixmap 
'''

#from PyQt5.QtCore import (QRunnable, QThread, QThreadPool, pyqtSignal, pyqtSlot)
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import cv2
import time
import os
import threading
#from collections import deque
from queue import Queue


from RoboArm_Lib import RoboArm
threadConn1 = None
threadConn2 = None
quitThreadsFlag = False


class PoseProcessThread(QThread):
	started = pyqtSignal()  #parameters()
	dataReceivedGuiUpdate = pyqtSignal(int, int)  #parameters()
	robotConnected = pyqtSignal(bool)  #parameters(True/False)
	
	#def OpenSocketConnection (RobotArmObj): #if created outside class
	def OpenSocketConnectionThreadFunction (self, RobotArmObj_NotInuse):
		print("Opening a connection to robot now...")
		result = self.RoboObj.connect(self.robotIP, self.robotPort)
		print("Robot.connection() result:", result)

		if result[0] < 0:
			self.robotConnected.emit(False) #some error
			self.RobotConnectedStatus = False
		else:
			self.robotConnected.emit(True) #OK//
			self.RobotConnectedStatus = True
		

	#def OpenSocketConnection (RobotArmObj): #if created outside class
	def ListenRobotResponseThread_Not_In_Use (self, RobotArmObj):
		global quitThreadsFlag
		print("ListenRobotResponseThread started...")
		
		count = 0
		while(True):
			if quitThreadsFlag:
				break
			if (True):
				time.sleep(0.5)
				continue

			if not self.RobotConnectedStatus:
				print("List thread: robot not connected...so skipping listening")
				time.sleep(1)
				continue

			if self.sentCount <= 0:
				print("List thread: no command sent...so skipping listening")
				time.sleep(0.1)
				continue

			#self.sentcount >=1, hence some command has  been sent
			try:		
				responseMsg = self.RoboObj.response() #TMSTA, queu number, true etc.
				print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
				print("responseMsg=",responseMsg, "type=",type(responseMsg))
				if "timed out" in str(responseMsg):
					self.resetState()
					continue

				if ('$TMSTA' in responseMsg):
					self.qLock.acquire() 
					print("Response Action acknowledged *********************************.")
					self.sentCount = self.sentCount - 1 #sentCount will be incremented when command is sent to robot
					self.AngleCmdQ.get() #remove item (item is put in dataReceived function only)
					print("Releasing lock....BBBBBBB")
					self.qLock.release()

			except Exception as e:
				print("Response Listen exception received:",e)
				self.resetState() #Reset Q and necessary variables
				
			time.sleep(0.02)

	
	#send related state....separated from connection realted
	#affected by pause, resume
	def resetState(self):
		self.armAngle = -1
		self.wristAngle = -1
		self.previousAngle = -1
		self.sentCount = 0
		self.sentPos = 0
		self.AngleCmdQ.queue.clear()
		try:
			self.qLock.release()
		except Exception as e:
			pass #ignore lock release exception if already released
		
	def __init__(self, poseProcess_fps, robotIP, robotPort=5890):
		QThread.__init__(self)
		self.frameRateSleepDuration = 1/poseProcess_fps
		#self.frameRateSleepDuration = 0.1 #maximum 10 commands in one second
		self.armMT = 0 #arm motion threshold
		self.wristMT = 0 #wrist motion threshold
		self.robotIP = robotIP
		self.robotPort = robotPort
		self.RoboObj = RoboArm()
		self.RobotConnectedStatus = False #commands to robot will be sent or not based on this status
		self.MaxQueueSize = 1 #can be  updated form the GUI
		self.AngleCmdQ = Queue(maxsize = self.MaxQueueSize)
		self.ThreadActive = False
		self.qLock = threading.Lock()
		self.resetState()
		

	def run(self):
		global quitThreadsFlag

		self.started.emit()
		
		while True:  #use some non-loop methods (which can be interrupted later
			if quitThreadsFlag:
				break

			if not self.RobotConnectedStatus:
				time.sleep(2) #2 second
				continue
			
			if (not self.ThreadActive): #thread is suspended
				time.sleep(self.frameRateSleepDuration) #to control the rate of pose detection
				continue

			self.qLock.acquire() 
			if self.AngleCmdQ.empty():
				#print("Q is empty...ignoring...")
				self.qLock.release()
				time.sleep(self.frameRateSleepDuration) 
				continue

			speed = 100
			acc = 250
			#angleArm = self.AngleCmdQ.queue[0] #item is not removed from the Q using get...it will be removed after response
			angleArm = self.AngleCmdQ.get() #read and remove the item from the queue
			self.RoboObj.move(a2=angleArm, speed=speed, accel=acc) 
			#self.sentCount = self.sentCount + 1 #sentCount will be decremented after response is received
			self.qLock.release()
			self.dataReceivedGuiUpdate.emit(angleArm, self.wristAngle)
			

	#move robot arm to start or final position (i.e. one of the end positions)
	def SetRoboArm2EndPosition(self, endPos = 0): #endPos: 0=initial, 1=final
		print("PoseProcessThread--SetRoboArm2InitPosition() called")
		print("RobotConnectedStatus=",self.RobotConnectedStatus)
		if self.RobotConnectedStatus == True:
			speed = 100
			#RoboObj.move(a2=AppObj.commandCurrent, speed=speed, accel=250)
			if endPos == 0: #initial position
				self.RoboObj.move(a2=30, speed=speed, accel=250) #default parameters move the robot arm to initial position
			else:
				self.RoboObj.move(a2=150, speed=speed, accel=250) #default parameters move the robot arm to initial position
			return 0
		else:
			return -1 #Error 
	

	def ConnectRobot(self):
		global threadConn1, threadConn2
		#It is necessary to create a thread here...otherwise if robot is unavailable and waiting takes long,
		#it crashes the current thead

		threadConn1 = threading.Thread(target=self.OpenSocketConnectionThreadFunction, name="optional", args=(self.RoboObj,))
		#no need to wait (i.e join()....)
		threadConn1.start()

		#threadConn2 = threading.Thread(target=self.ListenRobotResponseThread, args=(self.RoboObj,))
		#threadConn2.start()

	def DisconnectRobot(self):
		if self.RobotConnectedStatus == True:
			try:
				self.RoboObj.disconnect()
				self.RobotConnectedStatus = False
			except Exception as e:
				print("disconnect exception:", e)

	def updateQSize(self, qsize):
		if (self.ThreadActive == False):
			self.MaxQueueSize = qsize
			self.AngleCmdQ = Queue(maxsize = self.MaxQueueSize)
		print("PoseProcessThread-updateQSize: current Q Size= ", self.AngleCmdQ.qsize())


	def pause(self):
		print("PoseProcessThread pause called ")
		if (self.ThreadActive == True):
			self.ThreadActive = False
			self.resetState()
		else:
			pass #thread is already in pause mode...ignore

	def moveTest(self, start, end, stepsize):
		print("PoseProcess--moveTest(start,stop,stepsize):", start, end, stepsize)


	def resume(self):
		print("PoseProcessThread resume called ")
		if (self.ThreadActive == False):
			self.ThreadActive = True
			self.resetState()
		else:
			pass #thread is already in resume mode...ignore

	
	def dataUpdate(self, armAngle, wristAngle):
		#put the following code in a temporary thread if used with locks/semaphores
		print("dataUpdate in PoseProcessCalled...", armAngle)
		if armAngle == -1:
			print("Dummy data sent..-1...ignoring")
			return

		# the lock is released automatically outside the with block
		self.qLock.acquire() 
		print("dateUpdate: Lock Acquired")
		if self.AngleCmdQ.qsize() >= self.MaxQueueSize:
			print("Q is full...ignoring: reached MaxQueueSize=", self.MaxQueueSize)
			self.qLock.release()
			return #ignore this...Q is already full
		if self.AngleCmdQ.qsize() == 0: #first item....
			if self.previousAngle == -1: #if things were cleared, then no MT check needed
				self.AngleCmdQ.put(armAngle)
				print("First data added to angle Q:", armAngle)
				print("Q size=",self.AngleCmdQ.qsize(), " of ",self.MaxQueueSize)
			else: #thins are not cleared (by resume/pause), only idle
				if (abs(self.previousAngle - armAngle) > self.armMT):
					self.AngleCmdQ.put(armAngle)
					self.previousAngle = armAngle
					print("first value added- Prev,new,MT:", self.previousAngle, armAngle,self.armMT)
		else: #if one or elements exists in the Q, then check for motion threshold limit
			#previous = self.AngleCmdQ.queue[self.AngleCmdQ.qsize()-1]
			if (abs(self.previousAngle - armAngle) > self.armMT):
				self.AngleCmdQ.put(armAngle)
				self.previousAngle = armAngle
				print("More date added- Previous and new:", self.previousAngle, armAngle, " with motion threshold=",self.armMT)
				print("Q size=",self.AngleCmdQ.qsize(), " of ",self.MaxQueueSize)
			else:
				print("MT not crossed...ignoring..previous, new, mt:", self.previousAngle, armAngle, self.armMT)
		#put an extra check of time stamp difference as well..TBD
		self.qLock.release()




	def updateMT(self, armMT=0, wristMT=0):
		self.armMT = armMT
		self.wristMT = wristMT

	def changeFps(self, poseProcess_fps):
		self.frameRateSleepDuration = 1/poseProcess_fps

	def quit(self):
		global threadConn2, quitThreadsFlag
		quitThreadsFlag = True
		time.sleep(0.1)
		print("PoseProcessThread to quit now")
		self.DisconnectRobot()
		self.resetState()
		self.quit
	








