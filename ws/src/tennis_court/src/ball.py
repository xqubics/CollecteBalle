import cv2 as cv
import numpy as np

class ball:
    def __init__(self,timeStamp,X):
        self.time=timeStamp
        self.score=0
        self.position=X

    def update():
        pass

class terrainBalls:
    def __init__(self,timeStamp,img):
        # self.position=position
        self.time=timeStamp
        self.score=0
        self.terrain=img
        self.balls=[]
    
    def detect_balls(self):        
        frame_HSV = cv.cvtColor(self.terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
        ret,thresh = cv.threshold(frame_threshold,127,255,cv.THRESH_BINARY)
        contours,h = cv.findContours(thresh,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
        ball_centers=[]
        for cnt in contours:
            approx = cv.approxPolyDP(cnt, .03 * cv.arcLength(cnt, True), True)
            area = cv.contourArea(cnt)
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            ball_centers.append([cx,cy])
            circleArea = radius * radius * np.pi
            cv.drawContours(self.terrain, [cnt], 0, (0, 0, 255), -1)
        for i in range(len(ball_centers)):
            t=42
            b=ball(t,ball_centers[i])
            self.balls.append()
        # self.isolatedBalls=img
    
    def update():
        pass

img = cv.imread('terrain_1.png')
frame_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
gray=frame_threshold

ret,thresh = cv.threshold(gray,127,255,cv.THRESH_BINARY)

contours,h = cv.findContours(thresh,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
for cnt in contours:
    approx = cv.approxPolyDP(cnt, .03 * cv.arcLength(cnt, True), True)
    area = cv.contourArea(cnt)
    (cx, cy), radius = cv.minEnclosingCircle(cnt)
    circleArea = radius * radius * np.pi
    cv.drawContours(img, [cnt], 0, (0, 0, 255), -1)

cv.imshow('img',img)
cv.waitKey(0)
cv.destroyAllWindows()