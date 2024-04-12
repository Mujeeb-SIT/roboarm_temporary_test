#passed to Mjb from Kaarveen on 7-Nov-2023

import cv2
import mediapipe as mp
import time
import math
import socket
from XOR_CheckSum import xor_checksum_string


class poseDetector():

    def __init__(self, mode=False, model=1, smooth=True, segment=False, esegment=True,
                 detectionCon=0.5, trackCon=0.5):

        self.mode = mode
        self.model = 1
        self.smooth = smooth
        self.segment = segment
        self.esegment = esegment
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, self.model, self.smooth, self.segment, self.esegment,
                                     self.detectionCon, self.trackCon)

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,
                                           self.mpPose.POSE_CONNECTIONS)
        return img

    def findPosition(self, img, draw=True):
        self.lmList = []
        # lmList = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                # print(id, lm)
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([id, cx, cy])
                # lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        return self.lmList

    def findLength_1(self, img, p1, p2, draw=True):
        x1, y1 = self.lmList[p1][1:]
        x2, y2 = self.lmList[p2][1:]
        length_1 = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )

        # if length < 0:
        #     length += 1

        if draw:
            cv2.putText(img, str(int(length_1)), (x2 + 50, y2 + 50),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

        return length_1
    def findLength_2(self, img, p2, p3, draw=True):
        x2, y2 = self.lmList[p2][1:]
        x3, y3 = self.lmList[p3][1:]
        length_2 = math.sqrt( ((x2-x3)**2)+((y2-y3)**2) )
        if draw:
            cv2.putText(img, str(int(length_2)), (x2 + 150, y2 - 120),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        return length_2

    def findAngle(self, img, p1, p2, p3, draw=True):

        # Get the landmarks
        x1, y1 = self.lmList[p1][1:]
        # print(f'({x1},{y1})')
        cv2.putText(img, f'({x1},{y1})', (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        x2, y2 = self.lmList[p2][1:]
        # print(f'({x2},{y2})')
        cv2.putText(img, f'({x2},{y2})', (x2, y2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        x3, y3 = self.lmList[p3][1:]
        # print(f'({x3},{y3})')
        cv2.putText(img, f'({x3},{y3})', (x3, y3),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Calculate the Angle
        angle = math.degrees(math.atan2(y3 - y2, x3 - x2) -
                             math.atan2(y1 - y2, x1 - x2))

        if angle < 0:
            angle += 180

        # print(angle)

        # Draw
        if draw:
            cv2.line(img, (x1, y1), (x2, y2), (255, 255, 255), 3)
            cv2.line(img, (x3, y3), (x2, y2), (255, 255, 255), 3)
            cv2.circle(img, (x1, y1), 10, (0, 0, 255), cv2.FILLED)
            cv2.circle(img, (x1, y1), 15, (0, 0, 255), 2)
            cv2.circle(img, (x2, y2), 10, (0, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 15, (0, 0, 255), 2)
            cv2.circle(img, (x3, y3), 10, (0, 0, 255), cv2.FILLED)
            cv2.circle(img, (x3, y3), 15, (0, 0, 255), 2)
            cv2.putText(img, str(int(angle)), (x2 - 50, y2 + 50),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        return angle

def main():
    cap = cv2.VideoCapture(0)
    pTime = 0
    detector = poseDetector()
    # robot = robot_movement
    while True:
        success, img = cap.read()
        img = detector.findPose(img)
        # lmList = detector.findPosition(img)
        # print(lmList)
        lmList = detector.findPosition(img, draw=False)
        if len(lmList) != 0:
            print(lmList[14])
            # length_1 = detector.findLength_1(img, 12, 14)
            # lenght_2 = detector.findLength_2(img, 14, 16)
            cv2.circle(img, (lmList[14][1], lmList[14][2]), 15, (0, 0, 255), cv2.FILLED)

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (70, 50), cv2.FONT_HERSHEY_PLAIN, 3,
                    (255, 0, 0), 3)

        cv2.imshow("Image", img)
        # cv2.setMouseCallback('Point Coordinates', click_event)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()