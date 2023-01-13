import cv2 as cv
import numpy as np
from zenith_camera_subscriber import *

class Ball:
    def __init__(self,timeStamp,X,id):
        self.time=timeStamp
        self.time0=timeStamp
        self.elapsedTime=self.time-self.time0
        self.score=0
        self.position=X
        self.id=id

class TerrainBalls:
    def __init__(self,timeStamp):
        self.time=timeStamp
        self.score=0
        self.terrain=None
        self.balls=[]
        rclpy.init()
        self.Camera=ZenithCameraSubscriber(self.update)
        rclpy.spin(self.Camera)

    
    def detect_balls(self):
        frame_HSV = cv.cvtColor(self.terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
        ret,thresh = cv.threshold(frame_threshold,127,255,cv.THRESH_BINARY)
        contours,h = cv.findContours(thresh,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
        ball_centers=[]
        for cnt in contours:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            ball_centers.append([int(cx),int(cy)])
            cv.drawContours(self.terrain, [cnt], 0, (0, 0, 255), -1)
        if len(self.balls)==0:
            for i in range(len(ball_centers)):
                b=Ball(self.time,ball_centers[i],i)
                self.balls.append(b)
        else:
            for i in range(len(ball_centers)):
                b=Ball(self.time,ball_centers[i],i)
                self.findMatch(b)
        for b in self.balls:
            cv2.putText(self.terrain, str(b.position), tuple(b.position), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255,255,0), 1, cv2.LINE_AA)

    def findMatch(self,ball):
        match=False
        ball_positions=np.zeros((2,len(self.balls)))
        for i in range(len(self.balls)):
            ball_positions[:,i]=np.array([*self.balls[i].position])
        ball_distances=np.linalg.norm(ball_positions.T-ball.position,axis=1)
        closest_ball=np.argmin(ball_distances)
        threshold=35
        if ball_distances[closest_ball]<threshold:
            self.balls[closest_ball]=ball
        else:
            self.balls.append(ball)
        return match

    def show_balls(self):
        cv.imshow('Terrain',self.terrain)
        cv.waitKey(1)
    
    def __del__(self):
        self.Camera.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

    def update(self,img,timeStamp):
        self.terrain=img
        self.time=timeStamp
        self.detect_balls()
        self.show_balls()

class PathPlanner :
    def __init__(self) :
        self.click1=0
        self.click2=0
        self.i=0
        self.scale=20/1216
        self.passage1=np.array([0,4.5])
        self.passage2=np.array([0,-4.5])
        self.colect1=np.array([-9.19407895,-4.55592105])
        self.colect2=np.array([9.14473684,4.6875])

        
    def pixel_to_xy(self,lin,col,img_shape):
        X=np.array([lin,col])
        n,p,_=img_shape
        return (X-np.array([p,n])/2)*scale

    def xy_to_pixel(self,x,y,img_shape):
        X=x,y
        n,p,_=img_shape
        X=np.array([int(X[0]/scale+p/2),int(X[1]/scale+n/2)])
        return X

    def path_planner(self,x,y,img):
        X=(x,y)
        n,p,_=img.shape
        if x>0:
            x,y=xy_to_pixel(x,y,img.shape)
            x0,y0=xy_to_pixel(*colect2,img.shape)
            cv2.line(img, (x0,y0),(x,y), (255,0,0), 2)
            # return colect2
        else:
            x,y=xy_to_pixel(x,y,img.shape)
            x0,y0=xy_to_pixel(*colect1,img.shape)
            # x0,y0=int(colect1[0]/scale+p/2),int(colect1[1]/scale+n/2)
            cv2.line(img, (x0,y0),(x,y), (255,0,0), 2)
            # return colect1
        # print(x0,y0)

    def click_event(self,event, x, y, flags, params):
        self.i,self.click1,self.click2
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click2=self.click1
            print(self.click2)
            self.click1=np.array([x,y])
            self.i+=1
            n,p,_=img.shape
            # print((click1-np.array([p,n])/2)*scale)
            if self.i%2==1:
                cv2.circle(img, (x,y), 10, (0,0,255), 5)
                cv2.putText(img, str(self.i//2), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                1, (255,255,0), 1, cv2.LINE_AA)
            if self.i%2==0:
                cv2.circle(img, (x,y), 10, (255,0,0), 5)
                # print(np.linalg.norm(self.click2-click1)*scale)
                cv2.putText(img, str(self.i//2-1), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                1, (255,255,0), 1, cv2.LINE_AA)
            x,y=pixel_to_xy(x,y,img.shape)
            path_planner(x,y,img)
            cv2.imshow('image', img)

t=TerrainBalls(0)
t.Camera.destroy_node()
