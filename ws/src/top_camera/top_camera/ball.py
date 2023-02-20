import cv2 as cv
import numpy as np
from zenith_camera_subscriber import *


class Ball:
    def __init__(self, id, timeStamp, X):
        self.time0 = timeStamp
        self.elapsedTime = 0
        self.score = 0
        self.position = X
        self.id = id

    def get_score(self, robotPosition=(0, 0)):
        """
            Closer the ball is to the robot, higher the score
            Input:
                robotPosition: (x, y) tuple [in pixels, relative to the top left corner of the image]
            Output:
                score: float
        """
        distance_coeff = 1
        time_coeff = 10

        distance = np.linalg.norm(
            np.array(self.position) - np.array(robotPosition)
        )

        self._score = 1 / (distance_coeff * distance +
                           time_coeff * self.elapsedTime)

        # print("distance: ", distance, "time_diff: ",
        #       time_diff, "score: ", self._score)

        return self._score

    def set_position(self, X):
        self.position = X

    def set_time(self, timeStamp):
        self.elapsedTime = timeStamp-self.time0


class TerrainBalls:
    def __init__(self, timeStamp):
        self.time = timeStamp
        self.score = 0
        self.terrain = None
        self.balls = []
        self.nb_balls = 0
        rclpy.init()
        self.Camera = ZenithCameraSubscriber(self.update)
        self.path_=Path_Planner(self.Camera)
        rclpy.spin(self.Camera)

    def detect_balls(self,terrain):
        frame_HSV = cv.cvtColor(terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (30, 1, 1), (50, 255, 255))
        ret, thresh = cv.threshold(frame_threshold, 127, 255, cv.THRESH_BINARY)
        contours, h = cv.findContours(
            thresh, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        ball_centers = []
        for cnt in contours:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            cx_,cy_=self.path_.pixel_to_xy(cx,cy)
            if cx_>0 and radius > 2.5 :
                ball_centers.append([int(cx), int(cy)])
                cv.drawContours(terrain, [cnt], 0, (255, 0, 0), -1)
        n = len(ball_centers)
        for i in range(self.nb_balls):
            # print('boucle')
            k = self.find_match(self.balls[i], ball_centers)
            if k != -1:
                self.balls[i].set_position(ball_centers[k])
                ball_centers.pop(k)

        if self.nb_balls < n:
            for j in range(len(ball_centers)):
                # print('new_ball')
                self.nb_balls = self.nb_balls+1
                self.balls.append(
                    Ball(self.nb_balls, self.time, ball_centers[j]))
        # print(len(self.balls))
        for b in self.balls:
            b.set_time(self.time)
            X=b.position
            X_=self.path_.pixel_to_xy(X[0],X[1])
            self.path_.path_planner(X_[0],X_[1])
            for i in range(len(self.path_.path)-1) :
                cv2.line(self.terrain,self.path_.xy_to_pixel(self.path_.path[i][0],self.path_.path[i][1]),self.path_.xy_to_pixel(self.path_.path[i+1][0],self.path_.path[i+1][1]),(255,0,0),2)
        
            cv.putText(terrain, str(b.id), tuple(b.position), cv.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 0), 1, cv.LINE_AA)

            #  for score testing:
            # if i == 0:
            #     self.score_history = np.vstack([
            #         self.score_history,
            #         np.array([
            #             self.time,
            #             b.get_score(currentTimeStamp=self.time)
            #         ])
            #     ])
            #  ----------

        #  for score testing:
        # plt.cla()
        # # print('score_history', self.score_history[:])

        # plt.plot(self.score_history[:, 0], self.score_history[:, 1])
        # plt.draw()
        # plt.pause(0.01)
        #  ----------

    def find_match(self, ball, ball_centers):

        if len(ball_centers) == 0:
            return -1
        else:
            min = 5000
            # print(len(ball_centers))
            for i in range(len(ball_centers)):
                # print(self.path_.pixel_to_xy(ball_centers[i][0],ball_centers[i][1]))
                # if self.path_.click2!=0 :
                # self.path_.path_planner(self.path_.pixel_to_xy(ball_centers[i][0],ball_centers[i][1])[0],
                #     self.path_.pixel_to_xy(ball_centers[i][0],ball_centers[i][1])[1])
                n = np.sqrt((ball_centers[i][0]-ball.position[0])
                            ** 2+(ball_centers[i][1]-ball.position[1])**2)
                # print(n)
                if n < min :
                    min = n
                    closest_ball = i
        return closest_ball

    def show_balls(self,terrain):
        cv.imshow('Terrain', terrain)
        cv.waitKey(1)

    def __del__(self):
        self.Camera.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

    def update(self, img, timeStamp):
        self.terrain = img
        self.path_=Path_Planner(self.terrain)
        self.time = timeStamp
        self.detect_balls(self.terrain)
        self.show_balls(self.terrain)

class Path_Planner :
    def __init__(self,img,click1=[0,0],click2=[0,0],i=0,scale=20/1216,passage1=np.array([0,4.5]),passage2=np.array([0,-4.5]),collect1=np.array([-9.19407895,-4.55592105]),collect2=np.array([9.14473684,4.6875]),path=[]) :
        self.click1=click1
        self.click2=click2
        self.i=i
        self.scale=scale
        self.passage1=passage1
        self.passage2=passage2
        self.collect1=collect1
        self.collect2=collect2
        self.path=[]
        self.img=img

    def pixel_to_xy(self,lin,col):
        X=np.array([lin,col])
        n,p,_=self.img.shape
        return (X-np.array([p,n])/2)*self.scale

    def xy_to_pixel(self,x,y):
        X=x,y
        n,p,_=self.img.shape
        X=np.array([int(X[0]/self.scale+p/2),int(X[1]/self.scale+n/2)])
        return X

    def path_planner(self,x,y):
        self.click2=self.click1
        self.click1=np.array([x,y])
        X=(x,y)
        n,p,_=self.img.shape
        if len(self.path)!=0 and self.path[0]==self.collect2.tolist() :
                self.i+=1
        if x>0:
            x,y=self.xy_to_pixel(x,y)
            if (self.i==0) :
                self.path.append(self.collect2.tolist())
                x0,y0=self.xy_to_pixel(*self.collect2)
            else :
                x0,y0=self.click2[0],self.click2[1]
            self.path.append(self.pixel_to_xy(x,y).tolist())
        else:
            print("Terrain de gauche")


            #COMPLEXIFIED
            # x,y=self.xy_to_pixel(x,y)
            # if (self.i==0) :
            #     self.path.append(self.collect1.tolist())
            #     x0,y0=self.xy_to_pixel(*self.collect1)
            # else :
            #     x0,y0 = self.click2[0],self.click2[1]
            # # x0,y0=int(collect1[0]/self.scale+p/2),int(collect1[1]/self.scale+n/2)
            # # cv2.line(self.img, (x0,y0),(x,y), (255,0,0), 2)
            # # return collect1

        #utilisation i=indice auquel on va ajouter un nouveau chemin
        # if np.sign(self.pixel_to_xy(x,y)[0])==np.sign(self.pixel_to_xy(x0,y0)[0]) :
            # self.path.append(self.pixel_to_xy(x,y).tolist())
        # else :
        #     self.path=np.concatenate((np.array([self.collect1,self.pixel_to_xy(x,y),self.passage1]),np.array(self.path)),axis=0).tolist()

        #Draw lines
        # for i in range(len(self.path)-1) :
        #     cv2.line(self.img,self.xy_to_pixel(self.path[i][0],self.path[i][1]),self.xy_to_pixel(self.path[i+1][0],self.path[i+1][1]),(255,0,0),2)
        # print(self.path)

    def click_event(self,event, x, y, flags, params):
        if event == cv.EVENT_LBUTTONDOWN:
            x_,y_=self.pixel_to_xy(x,y)
            print(self.i)
            if x_>0 :
                n,p,_=self.img.shape
                cv2.circle(self.img, (x,y), 10, (0,0,255), 5)
                cv2.putText(self.img, str(len(self.path)-self.i+1), (x,y), cv2.FONT_HERSHEY_SIMPLEX,1, (255,255,0), 1, cv2.LINE_AA)
                for i in range(len(self.path)-1) :
                    cv2.line(self.img,self.xy_to_pixel(self.path[i][0],self.path[i][1]),self.xy_to_pixel(self.path[i+1][0],self.path[i+1][1]),(255,0,0),2)
                self.path_planner(x_,y_)
                cv.imshow('Terrain', self.img)

t=TerrainBalls(0)
t.Camera.destroy_node()