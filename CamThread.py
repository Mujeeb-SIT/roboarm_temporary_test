from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import cv2
import time
import os

#========== to be moved to PoseModule Thread
import PoseModule_lib as pm 
angle_round = -1
# Wait_Queue_Tag = ()
Queue_Tag = ()
# StopAndClearBuffer = True

detector = pm.poseDetector()


#Ref 1: https://www.youtube.com/watch?v=dTDgbx-XelY
#Ref 2: https://stackoverflow.com/questions/72265287/pyqt5-i-want-to-show-camera-view-to-label

class CamQThread(QThread):
	started = pyqtSignal()
	frameShow = pyqtSignal(QImage)
	passPoseData = pyqtSignal(int)

	ThreadActive = False

	detector = pm.poseDetector()

	def __init__(self, width, height, fps, cameraID, mt_angle=1, mt_wrist = 0): #width and height of the label window
		QThread.__init__(self)

		self.width = width
		self.height = height
		self.poseDelay = 0.03 #approximate
		#self.poseDelay = 0 #approximate
		self.frameDelay = (1/fps) - self.poseDelay
		self.cameraID = cameraID #0 for internal
		self.lpf = [0.5,0.3,0.2]
		#self.lpf = [0.1,0.0,0.0]
		self.previousAngles = [None] * (len(self.lpf)-1)

		#MT
		self.MT_Angle = mt_angle
		self.MT_wrist = mt_wrist
		self.angle_SentAlready = False
		self.angle_Sent_Prev_Val = -1
		self.wrist_SentAlready = False #not in use for now
		self.wrist_Sent_Prev_Val = -1


	def run(self):
		self.started.emit()
		counter = 0
		previous_fps = 0
		Capture = cv2.VideoCapture(self.cameraID)
		pTime = time.time()
		while True:  #use some non-loop methods (which can be interrupted later)
			counter +=1
			if not self.ThreadActive:
				frame = cv2.imread("./Pictures/Camera_Off.jpg")
				frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				frame = cv2.resize(frame, (self.width, self.height), Qt.KeepAspectRatio)
				image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
				self.frameShow.emit(image)
				pTime = time.time()
				time.sleep(2)
				continue
			
			#TODO: specify capture speed and size parameters

			ret, frame = Capture.read()
			if ret:
				frame = cv2.flip(frame, 180) #for left-right mirror imaging
				img = detector.findPose(frame, False)
				lmList = detector.findPosition(img, False)
				if len(lmList) != 0:
					# Right Arm 
					#angle = detector.findAngle(img, 12, 14, 16) #right hand 
					angle = detector.findAngle(img, 11, 13, 15) 
					angle_round = round(angle)
					angle_round = self.ApplyLPF(angle_round)
					#Mjb: let the limit be put in the calling function 10 to 130 

					mt_passed = True
					if (self.angle_SentAlready):
						mt_passed = self.ApplyMT(angle_round)

					if (mt_passed):					
						self.passPoseData.emit(angle_round)
						self.angle_SentAlready = True 
						self.angle_Sent_Prev_Val = angle_round
						print("pose angle sent = ", angle_round)


				cTime = time.time()
				fps = 1 / (cTime - pTime)
				pTime = cTime

				if (counter %10 == 0):
					previous_fps = fps
					cv2.putText(img, str(int(fps)), (50, 100), cv2.FONT_HERSHEY_PLAIN, 5, (255, 0, 0), 5)
					counter = 0
				else:
					cv2.putText(img, str(int(previous_fps)), (50, 100), cv2.FONT_HERSHEY_PLAIN, 5, (255, 0, 0), 5)

				#cv2 and natve QT/PIL opposite color sequencing (RGB vs BGR)...
				# Hence cv2(BGR) cpatured image must be converted to QT(RGB)
				img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
				#img = cv2.resize(img, (self.width, self.height), Qt.KeepAspectRatio)
				img = cv2.resize(img, (self.width, self.height))
				img = QImage(img, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)

				self.frameShow.emit(img)
				time.sleep(self.frameDelay)

	def pause(self):
		self.ThreadActive = False
		print("CamQThread to pause now")

	def resume(self):
		self.ThreadActive = True
		self.previousAngles = [None] * len(self.lpf) #reset the LFP values
		self.angle_SentAlready = False
		self.angle_Sent_Prev_Val = -1
		print("CamQThread to resume now")
	
	def changeFPS(self, fps):
		self.frameDelay = 1/fps - self.poseDelay
		print("CamQThread fps changed to:", fps)
	
	def ApplyLPF(self, angle_round):
		if (None in self.previousAngles): #LPF is not yet filled..
			angle_round_LPF = angle_round
		else:
			angle_round_LPF = angle_round*self.lpf[0] + \
								self.previousAngles[0]*self.lpf[1] + \
								self.previousAngles[1]*self.lpf[2]

		self.previousAngles[1] = self.previousAngles[0]
		self.previousAngles[0] = angle_round_LPF
		
		return angle_round_LPF 

	def ApplyMT(self, angle_round):
		if (abs(angle_round - self.angle_Sent_Prev_Val) >= self.MT_Angle):
			return True
		else:
			return False

	def updateMT(self, angle_mt=0, wrist_mt=0):
		self.angle_mt = angle_mt
		self.wrist_mt = wrist_mt	


	def quit(self):
		print("CamQThread to quit now")
		self.quit
