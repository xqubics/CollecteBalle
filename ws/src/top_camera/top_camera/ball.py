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
        rclpy.spin(self.Camera)

    def detect_balls(self):
        frame_HSV = cv.cvtColor(self.terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
        ret, thresh = cv.threshold(frame_threshold, 127, 255, cv.THRESH_BINARY)
        contours, h = cv.findContours(
            thresh, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        ball_centers = []
        for cnt in contours:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            ball_centers.append([int(cx), int(cy)])
            cv.drawContours(self.terrain, [cnt], 0, (0, 0, 255), -1)
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
        for b in self.balls:
            b.set_time(self.time)
            cv2.putText(self.terrain, str(b.id), tuple(b.position), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 0), 1, cv2.LINE_AA)

            #  for score testing:
            # if i == 0:
            #     self.score_history = np.vstack([
            #         self.score_history,
            #         np.array([
            #             self.time,
            #             b.get_score(currentTimeStamp=self.time)
            #         ])
            #     ])
            #  ----------

        #  for score testing:
        # plt.cla()
        # # print('score_history', self.score_history[:])

        # plt.plot(self.score_history[:, 0], self.score_history[:, 1])
        # plt.draw()
        # plt.pause(0.01)
        #  ----------

    def find_match(self, ball, ball_centers):

        if len(ball_centers) == 0:
            return -1
        else:
            min = 5000
            # print(len(ball_centers))
            for i in range(len(ball_centers)):
                n = np.sqrt((ball_centers[i][0]-ball.position[0])
                            ** 2+(ball_centers[i][1]-ball.position[1])**2)
                if n < min:
                    min = n
                    closest_ball = i
            return closest_ball

    def show_balls(self):
        cv.imshow('Terrain', self.terrain)
        cv.waitKey(1)

    def __del__(self):
        self.Camera.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

    def update(self, img, timeStamp):
        self.terrain = img
        self.time = timeStamp
        self.detect_balls()
        self.show_balls()

<<<<<<< HEAD
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
=======

t = TerrainBalls(0)
>>>>>>> c9a8e3b09d5af240af968e2d2d59726927f8988d
