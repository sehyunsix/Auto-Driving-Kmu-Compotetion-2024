#!/usr/bin/env python

import cv2
import numpy as np
import os
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from dataclasses import dataclass
import math
from utils import logging_time


class State:
    def __init__(self,vx,vy):
        self.vx=vx
        self.vy=vy

class HistLanes:
    def __init__(self, x_left, x_right, left_confidence, right_confidence):
        self.x_left = x_left
        self.x_right = x_right
        self.left_confidence = left_confidence
        self.right_confidence = right_confidence

# Single lane line
@dataclass(repr=True)
class Line:
    lane_indexes =None
    # pixel positions
    x = []
    y = []

    # Fit a second order polynomial to each
    fit = None
    # Plotting parameters
    fitx = []

    # Histogram
    hist_x = None

    def __repr__(self) -> str:
        return (
            self.__class__.__qualname__ + f"(lane_indedxes={self.lane_indexes!r}\nx={self.x!r}\n "
            f"y={self.y!r}\nfit={self.fit!r}\nfitx={self.fitx!r}\nhist_x={self.hist_x!r})\n"
        )
    


# Data collected during the sliding windows phase
class SlideWindow:
    left = Line()
    right = Line()
    left_center=Line()
    right_center=Line()
    main_center=Line()
    hist = None
    left_avg = None
    right_avg = None
    ploty = None



    def __init__(self, hist, left_lane_indexes, right_lane_indexes, non_zero_x, non_zero_y):

        if len(left_lane_indexes)>0:
            self.left.lane_indexes = np.concatenate(left_lane_indexes)
            self.left.hist_x = hist.x_left
            self.left.x = non_zero_x[self.left.lane_indexes]
            self.left.y = non_zero_y[self.left.lane_indexes]

        if len(right_lane_indexes)>0:
            self.right.lane_indexes = np.concatenate(right_lane_indexes)
            self.right.hist_x = hist.x_right
            self.right.x = non_zero_x[self.right.lane_indexes]
            self.right.y = non_zero_y[self.right.lane_indexes]


    def plot_lines(self, img, color_left=(0, 255, 255), color_right=(0, 255, 255)):
        left = []
        right = []
       
        if (len(self.left.fitx)==len(self.ploty)):
            for i in range(0, len(self.ploty)):
                left.append((self.left.fitx[i], self.ploty[i]))

            cv2.polylines(img, np.int32([left]), False, color_left)
        
        if (len(self.right.fitx)==len(self.ploty)):
            for i in range(0, len(self.ploty)):
                right.append((self.right.fitx[i], self.ploty[i]))

            cv2.polylines(img, np.int32([right]), False, color_right)

        return img

class LaneDriver:
    #343
    middle=353
    lookahead=347
    lane_width = 250

    lf=18
    ld=14
    
    left_lanes=[]
    right_lanes=[]

    width = 640 # 두 좌우 거리간의 최대값이 서류의 폭
    height = 480  # 두 상하 거리간의 최대값이 서류의 높이
    
    pts1 = np.float32([[241,267],[479,261],[633,314],[81,325]])

    # 변환 후 4개 좌표
    pts2 = np.float32([[0, 0], [width, 0],
                        [width, height], [0, height]])
    # 변환 행렬 계산 
    perspective_correction= cv2.getPerspectiveTransform(pts1, pts2)
    perspective_correction_inv =cv2.getPerspectiveTransform(pts2, pts1)
    perspective_trapezoid=None
    perspective_dest=None
    left_fit_avg = None
    right_fit_avg = None
    sw=None
    warp_size=None
    orig_size=None

    #image
    wraped_lane=None

    #next point
    nextPoint=[]

    ##Gray scale image
    img_wraped =None

    #previosu_frame load
    old_main_center_fitx= []
    old_main_center_avg=[]
    old_next_point=[]
    state= State(0,0)
    dt=0.03
    x_ratio=625
    y_ratio=376
    theta=np.pi/2
    pridicted_next_point=[]


    @logging_time
    def edge_detection(self,channel):
        channel=cv2.GaussianBlur(channel, (3,3),sigmaX=0.0, )
        # channel=cv2.GaussianBlur(channel, (7,7),sigmaX=0.0, )
        # channel=cv2.GaussianBlur(channel, (13,13),sigmaX=0.0, )
        adaptive_binary =cv2.adaptiveThreshold(channel, 255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY ,blockSize=11,C=9)
        edge = cv2.Canny(adaptive_binary, 70, 180)
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13)) 
        edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
        edge_to_closing = cv2.medianBlur(edge_to_closing,5) 

        # edge_x = cv2.Canny(channel,100,100)
        # soble_dx=cv2.Sobel(channel, cv2.CV_64F, 1, 0)
        # soble_dy=cv2.Sobel(channel, cv2.CV_64F, 0, 1)
        # edge_x=cv2.magnitude(soble_dx,soble_dy)
        # soble_mag=np.clip(soble_mag,0,255)
        # soble_mag=np.clip(soble_mag)
        # edge_x=cv2.Laplacian(channel,cv2.CV_64F,-1,scale=10)
        # edge_x = np.absolute(edge_x)
        # cv2.imshow("canny",edge_x)
        # cv2.waitKey(0)

        # return save_dir(np.uint8(255 * edge_x / np.max(edge_x)), "edge_", filename)
        return edge_to_closing
    @logging_time
    def threshold(self, channel_edge):
        # Gradient threshold
        binary = np.zeros_like(channel_edge)
        height = binary.shape[0]

        threshold_up = 40
        # threshold_up=200
        threshold_down = 80
        # threshold_down=240 
        threshold_delta = threshold_down - threshold_up

        for y in range(height):
            binary_line = binary[y, :]
            edge_line = channel_edge[y, :]
            threshold_line = threshold_up + threshold_delta * y / height
            binary_line[edge_line >= threshold_line] = 255


        # cv2.imshow("binary",binary)
   

        return binary

    @logging_time
    def histogram(self,img):
        partial_img = img[int(self.height*1/3):, :]  # Select the bottom part (one third of the image)
        hist = np.sum(partial_img, axis=0)

        # if get_save_files() and filename:
            # plt.plot(hist)
            # plt.draw()
            # plt.pausnp.concatenatee(0.01)
            # plt.clf()

        return hist

    @logging_time
    def avg_x(self,hist_lanes):
        return sum([lane.hist_x for lane in hist_lanes]) // len(hist_lanes)



    @logging_time
    def slide_window(self,img, binary_warped, hist, num_windows):
        binary_warped = cv2.dilate(binary_warped, np.ones((3, 3), np.uint8), iterations=1)
        img_height = binary_warped.shape[0]
        window_height = np.int32(img_height / num_windows)
        # Indices (e.g. coordinates) of the pixels that are not zero
        non_zero = binary_warped.nonzero()
        non_zero_y = np.array(non_zero[0])
        non_zero_x = np.array(non_zero[1])
        # print("debug",non_zero_x.shape ,non_zero_y.shape)
        # print(non_zero_x)
        # Current positions, to be updated while sliding the window; we start with the ones identified with the histogram
        left_x = hist.x_left
        right_x = hist.x_right
  

        margin = 50
        # Set minimum number of pixels found to recenter window
        min_pixels =30
        left_lane_indexes = []
        right_lane_indexes = []
        left_x_momoent=0
        right_x_momoent=0
        left_x_con=0
        right_x_con=0
        momoent_cf=0.7
        hist.left_confidence=0
        hist.right_confidence=0


        out_img = img.copy() 

        for idx_window in range(num_windows-5):
            # X range where we expect the left lane to land
            win_x_left_min = left_x - margin
            win_x_left_max = left_x + margin
            # X range where we expect the right lane to land
            win_x_right_min = right_x - margin
            win_x_right_max = right_x + margin
            # Y range that we are analyzing
            win_y_top = img_height - idx_window * window_height
            win_y_bottom = win_y_top - window_height

         
            cv2.rectangle(out_img, (win_x_left_min, win_y_bottom), (win_x_left_max, win_y_top), (255, 255, 255), 2)
            cv2.rectangle(out_img, (win_x_right_min, win_y_bottom), (win_x_right_max, win_y_top), (255, 255, 255), 2)

            # Non zero pixels in x and y
            non_zero_left = ((non_zero_y >= win_y_bottom) & (non_zero_y < win_y_top) & (non_zero_x >= win_x_left_min) & (
                    non_zero_x < win_x_left_max)).nonzero()[0]
            non_zero_right = ((non_zero_y >= win_y_bottom) & (non_zero_y < win_y_top) & (non_zero_x >= win_x_right_min) & (
                    non_zero_x < win_x_right_max)).nonzero()[0]
            if len(non_zero_left) > min_pixels:
                if(idx_window<3):
                    left_x_con +=2
                left_x_con +=1
                next_x =np.int32(np.mean(non_zero_x[non_zero_left]))
                left_x_momoent = next_x -left_x
                left_x = next_x + np.int32(left_x_momoent*momoent_cf)
                if hist.left_confidence <left_x_con:
                    hist.left_confidence=left_x_con
                left_lane_indexes.append(non_zero_left)
            else:
                left_x_con=0
                hist.left_confidence += -1

            if len(non_zero_right) > min_pixels:
                if(idx_window<3):
                    right_x_con +=2
                right_x_con +=1
                next_x = np.int32(np.mean(non_zero_x[non_zero_right]))
                right_x_momoent =next_x-right_x
                right_x = next_x + np.int32(right_x_momoent*momoent_cf)
                if hist.right_confidence <right_x_con:
                    hist.right_confidence=right_x_con
                right_lane_indexes.append(non_zero_right)
            else:
                right_x_con=0
                hist.right_confidence += -1

            # print(hist.left_confidence ,hist.right_confidence)

        # print(hist.right_confidence, hist.left_confidence)
        valid, sw = self.fit_slide_window(binary_warped, hist, left_lane_indexes, right_lane_indexes, non_zero_x, non_zero_y)
        # print(valid)
        if valid:
            img_plot = sw.plot_lines(out_img)
            cv2.imshow("window",img_plot)

        else:
            print("Error : No lane Detected")
        return valid, sw

    @logging_time
    def moving_average(self,prev_average, new_value, beta):
        return beta * prev_average + (1 - beta) * new_value if prev_average is not None else new_value


    @logging_time
    def DisionLeftRight(self,main_center_fitx):
        lane_margin=30
        margin=200
        height=10
        left_point=0
        right_point=0
        window_number =5 
        y_index= int(self.height*3/4)
        for i in range(window_number):
            y_index = y_index-height*2*i
            center = int(main_center_fitx[y_index])
            left_most = max( center-margin ,0)
            right_most = min(self.width-1 , center+margin)
            left = self.img_wraped[y_index-height:y_index+height, left_most : center-lane_margin]
            right =self.img_wraped[y_index-height:y_index+height, center+lane_margin : right_most ]
            cv2.rectangle(self.img_wraped, (left_most, y_index-height), (center-lane_margin, y_index+height), (255, 255, 255), 2)
            cv2.rectangle(self.img_wraped, (center+lane_margin, y_index-height), (right_most, y_index+height), (255, 255, 255), 2)
            left_point +=np.mean(left)
            right_point += np.mean(right)


        # cv2.imshow("checkbox",self.img_wraped)
    
        if left_point> right_point:
            return "left"
        else:
            return "right"
        


    @logging_time
    def fit_slide_window(self,binary_warped, hist, left_lane_indexes, right_lane_indexes, non_zero_x, non_zero_y):



        sw = SlideWindow(hist, left_lane_indexes, right_lane_indexes, non_zero_x, non_zero_y)
        sw.hist= hist
        # y coordinates
        sw.ploty = np.array([float(x) for x in range(binary_warped.shape[0])])

        if len(sw.left.y) == 0 and len(sw.right.y)==0:
            return False, sw

        # Fit a second order polynomial to approximate the points

        if len(sw.left.y) != 0:
            left_fit = np.polynomial.polynomial.polyfit(sw.left.y, sw.left.x, 2)
            self.left_fit_avg = self.moving_average(self.left_fit_avg, left_fit, 0.7)
            sw.left.fitx = self.left_fit_avg[2] * sw.ploty ** 2 + self.left_fit_avg[1] * sw.ploty + self.left_fit_avg[0]



        if len(sw.right.y) !=0:
            right_fit = np.polynomial.polynomial.polyfit(sw.right.y, sw.right.x, 2)
            self.right_fit_avg = self.moving_average(self.right_fit_avg, right_fit, 0.7)
            sw.right.fitx = self.right_fit_avg[2] * sw.ploty ** 2 + self.right_fit_avg[1] * sw.ploty + self.right_fit_avg[0]
        ##make center value
       
        # sw.left_center.fitx =(self.left_fit_avg[2]) * (sw.ploty -a)** 2 + (self.left_fit_avg[1])* (sw.ploty-a)+ self.left_fit_avg[0]+b
        # sw.right_center.fitx =(self.right_fit_avg[2]) * (sw.ploty +a)** 2 + (self.right_fit_avg[1])* (sw.ploty+a)+ self.right_fit_avg[0]-b
        # if(hist.left_confidence <3000 and hist.right_confidence<3000):
        #     print("Not  detected line")   
        #     return False, sw
        if(hist.left_confidence <-2 and hist.right_confidence <-2):
            return False,sw
        
        
        if (hist.left_confidence >hist.right_confidence):
            # result =self.DisionLeftRight(sw.left.fitx)
            # print("left")
            main_fit_avg = self.left_fit_avg
            main_fit = sw.left.fitx 
        if (hist.left_confidence <=hist.right_confidence):
            # result =self.DisionLeftRight(sw.right.fitx)
            # print("right")
            main_fit_avg = self.right_fit_avg
            main_fit =sw.right.fitx 

        #2
        w=self.lane_width
        a= -2*w*main_fit_avg[2]/((4*(main_fit_avg[2]**2)+1)**0.5)
        b= w/((4*(main_fit_avg[2]**2)+1)**0.5)
        print((a**2+b**2)**0.5)

        if(len(self.old_main_center_fitx) != 0 and abs(np.mean(main_fit[len(main_fit)-5:])-self.middle)<50):
            

            # right_fitx = (main_fit_avg[2]) * (sw.ploty -a)** 2 + (main_fit_avg[1])* (sw.ploty-a)+ main_fit_avg[0]+b
            # left_fitx = (main_fit_avg[2]) * (sw.ploty +a)** 2 + (main_fit_avg[1])* (sw.ploty+a)+ main_fit_avg[0]-b
            r_right_fitx = (self.right_fit_avg[2]) * (sw.ploty -a)** 2 + (self.right_fit_avg[1])* (sw.ploty-a)+ self.right_fit_avg[0]+b
            r_left_fitx = (self.left_fit_avg[2]) * (sw.ploty -a)** 2 + (self.left_fit_avg[1])* (sw.ploty-a)+ self.left_fit_avg[0]+b
            l_right_fitx=(self.right_fit_avg[2]) * (sw.ploty +a)** 2 + (self.right_fit_avg[1])* (sw.ploty+a)+ self.right_fit_avg[0]-b
            l_left_fitx=(self.left_fit_avg[2]) * (sw.ploty +a)** 2 + (self.left_fit_avg[1])* (sw.ploty+a)+ self.left_fit_avg[0]-b
            # print(r_left_fitx.shape)
            # print(r_right_fitx.shape)
            # print(l_left_fitx.shape)
            # print(l_right_fitx.shape)
            # print(sw.right.fitx.shape)
            # print(sw.left.fitx.shape)

             

 
            next_lane= np.stack([l_right_fitx, l_left_fitx ,r_left_fitx, r_right_fitx])
            next_point_error =np.abs(next_lane[:,self.lookahead]-self.old_main_center_fitx[self.lookahead])
            idx=np.argmin(next_point_error)
            sw.main_center.fitx=next_lane[idx]
            
            self.old_main_center_avg =self.old_main_center_avg
            # if(np.abs(self.old_main_center_fitx[self.lookahead]-left_fitx[self.lookahead])) > np.abs(self.old_main_center_fitx[self.lookahead]-right_fitx[self.lookahead]):
            #     # print("right")
            #     sw.main_center.fitx = right_fitx
            # else:
            #     # print("rleft")

            #     sw.main_center.fitx= left_fitx
        else:

            if(np.mean(main_fit[len(main_fit)-50:])<self.middle):
                # print("left_fit************")
                sw.main_center.fitx = (main_fit_avg[2]) * (sw.ploty -a)** 2 + (main_fit_avg[1])* (sw.ploty-a)+ main_fit_avg[0]+b
            else:
                # print("right_fit************")
                sw.main_center.fitx = (main_fit_avg[2]) * (sw.ploty +a)** 2 + (main_fit_avg[1])* (sw.ploty+a)+ main_fit_avg[0]-b
            
        
        sw.left.hist_x = int(sw.main_center.fitx[0]-w)
        sw.right.hist_x = int(sw.main_center.fitx[0]+w)
        
        self.old_main_center_fitx = sw.main_center.fitx

   
        return True, sw
   
    @logging_time
    def find_next_point(self):
        y=self.lookahead
      
        if len(self.sw.main_center.fitx) != 0:
            x=self.sw.main_center.fitx[y]
            self.nextPoint=[x,y]
            self.old_next_point=[x,y]
            # cv2.circle(self.wraped_lane, (int(self.predicted_next_point[0]),int(self.lookahead-self.state.vy*self.dt*self.y_ratio)),10,(0,0,255),-1)
            cv2.circle(self.wraped_lane, (self.middle,self.height),10,(255,255,0),-1)
            cv2.circle(self.wraped_lane, (int(x),int(y)),10,(0,255,255),-1) 
            # if (abs(self.predicted_next_point[0] - self.old_next_point[0]) <abs(self.nextPoint[0] - self.old_next_point[0])):
            #      cv2.circle(self.wraped_lane, (int(self.predicted_next_point[0]),int(self.predicted_next_point[1])),10,(255,0,0),-1)
            # else:
            cv2.imshow("final",self.wraped_lane)

        return 
    
    @logging_time
    def find_next_degree(self):
        if len(self.nextPoint) == 0:
            return  0
        # print("nextPoint",self.nextPoint[0],self.nextPoint[1])
        beta = abs(math.atan((math.tan(abs((self.nextPoint[0]-self.middle)/(self.height-self.nextPoint[1]+self.lookahead)))*(self.lf+self.ld)/self.ld)))


        if self.nextPoint[0] >self.middle:
          degree= beta*180/np.pi
          self.sign=1
        else:
          degree = -beta*180/np.pi

        # assert degree < 0 , print("degree", degree)
        # print("degree",degree)
        assert -90<= degree <= 90 , print("degree",degree)
        return degree

    @logging_time
    def show_lanes(self,sw, img_warped, img_orig):
        img = img_warped.copy()

        if len(sw.left.fitx)>0 and len(sw.ploty)>0:
            fitx_points_warped = np.float32([np.transpose(np.vstack([sw.left.fitx, sw.ploty]))])
            fitx_points = cv2.perspectiveTransform(fitx_points_warped, self.perspective_correction_inv)
            left_line_warped = np.int_(fitx_points_warped[0])
            left_line = np.int_(fitx_points[0])
            n = len(left_line)

            for i in range(n - 1):
                cv2.line(img_orig, (left_line[i][0], left_line[i][1]), (left_line[i + 1][0], left_line[i + 1][1]),
                        (0, 255, 0), 5)
                cv2.line(img, (left_line_warped[i][0], left_line_warped[i][1]),
                        (left_line_warped[i + 1][0], left_line_warped[i + 1][1]), (0, 255, 0), 5)

        if len(sw.right.fitx)>0  and len(sw.ploty)>0:
            
            fitx_points_warped = np.float32([np.transpose(np.vstack([sw.right.fitx, sw.ploty]))])
            fitx_points = cv2.perspectiveTransform(fitx_points_warped, self.perspective_correction_inv)
            right_line_warped = np.int_(fitx_points_warped[0])
            right_line = np.int_(fitx_points[0])

            for i in range(len(right_line) - 1):
                cv2.line(img_orig, (right_line[i][0], right_line[i][1]), (right_line[i + 1][0], right_line[i + 1][1]),
                        (0, 0, 255), 5)
                cv2.line(img, (right_line_warped[i][0], right_line_warped[i][1]),
                        (right_line_warped[i + 1][0], right_line_warped[i + 1][1]), (0, 0, 255), 5)
                

        if len(sw.main_center.fitx) !=0: 
                fitx_points_warped = np.float32([np.transpose(np.vstack([sw.main_center.fitx, sw.ploty]))])
                fitx_points = cv2.perspectiveTransform(fitx_points_warped, self.perspective_correction_inv)
                right_line_warped = np.int_(fitx_points_warped[0])
                right_line = np.int_(fitx_points[0])

                for i in range(len(right_line) - 1):
                
                    cv2.line(img_orig, (right_line[i][0], right_line[i][1]), (right_line[i + 1][0], right_line[i + 1][1]),
                                (255, 100, 255), 5)
                    cv2.line(img, (right_line_warped[i][0], right_line_warped[i][1]),
                                (right_line_warped[i + 1][0], right_line_warped[i + 1][1]), (255, 100, 255), 5)
              
                        
                
            
        # cv2.imshow("lanes",img)
        self.wraped_lane=img

        return img_orig

    def lanes_full_histogram(self,histogram ,binary_image):
        size = len(histogram)
        margin=0
        max_index_left = np.argmax(histogram[0:self.middle-margin])
        max_index_right = np.argmax(histogram[self.middle+margin:]) + self.middle+margin   
        
        # print(max_index_left, max_index_right)
        # max_index_left = np.mean(binary_image[0:self.middle-margin])
        # max_index_right =np.mean(binary_image[0:self.middle-margin]) + self.middle+margin  
        
        
        return HistLanes(max_index_left, max_index_right, histogram[max_index_left], histogram[max_index_right])


    def lanes_partial_histogram(self,histogram):
        max_index_left =  max(int(self.left_lanes[-1].hist_x) ,0)
        max_index_right = min(int(self.right_lanes[-1].hist_x) ,len(histogram))
        # print(max_index_left, max_index_right)
        return HistLanes(max_index_left, max_index_right, histogram[0], histogram[0])


    def partial_lane(self,histogram, lanes_previous_frames, tolerance):
        lane_avg = self.avg_x(lanes_previous_frames)
        lane_min = max(lane_avg - tolerance, 0) 
        lane_max = min(lane_avg + tolerance, len(histogram))

        return np.argmax(histogram[lane_min:lane_max]) + lane_min
    
    def filter_white_color(self,hsv_image ,sat_thresh=(0,80), val_thresh=(110,255)):

        # white_mask = cv2.inRange(hsv_image, np.array([0, sat_thresh[0], val_thresh[0]]), np.array([180, sat_thresh[1], val_thresh[1]]))
        white_mask = cv2.adaptiveThreshold(hsv_image, 255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY ,blockSize=9,C=5)
        return white_mask

    @logging_time
    def detect_lanes(self,img_bgr):

        # img_bgr =cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        


        img_warped = cv2.warpPerspective(img_bgr, self.perspective_correction, (self.width, self.height),flags=cv2.INTER_LANCZOS4)
        gray_image =cv2.cvtColor(img_warped, cv2.COLOR_BGR2GRAY)

        self.img_wraped = img_warped.copy()
        img_edge = self.edge_detection(gray_image)


        cv2.imshow( "img_binary" ,img_edge)

    
        hist = self.histogram(img_edge)
        lanes = self.lanes_full_histogram(hist, img_edge)


        ret, sw = self.slide_window(img_warped, img_edge, lanes, 15)

        if ret:
            self.left_lanes.append(deepcopy(sw.left))
            self.right_lanes.append(deepcopy(sw.right))
        else:
            if len(self.left_lanes) != 0:
            # In case of problems, use the previous detection
                sw.left = self.left_lanes[len(self.left_lanes) - 1]
                sw.right = self.right_lanes[len(self.right_lanes) - 1]

                self.left_lanes.append(sw.left)
                self.right_lanes.append(sw.right)
  
        
        img_lane = self.show_lanes(sw, img_warped, img_bgr)
  
        self.sw=sw
        return img_lane
    

    def predict_next_point(self):
        if len(self.old_main_center_fitx)>0:
            self.theta=(self.nextPoint[0]-self.middle)/(self.height-self.nextPoint[1]+self.lookahead*0.4)
            if(abs(self.old_main_center_fitx[int(self.lookahead+self.state.vy*self.dt*self.y_ratio)]-int(self.theta*self.state.vy*self.dt*self.x_ratio)<200)):
                self.predicted_next_point =[self.old_main_center_fitx[int(self.lookahead-self.state.vy*self.dt*self.y_ratio)]-int(self.theta*self.state.vy*self.dt*self.x_ratio),self.lookahead]
            else:
                 self.predicted_next_point =[self.old_main_center_fitx[int(self.lookahead-self.state.vy*self.dt*self.y_ratio)]-int(self.theta*self.state.vy*self.dt*self.x_ratio),self.lookahead]

        else:
            self.predicted_next_point=[0,self.lookahead]

    def Driving(self,image):
        self.detect_lanes(image.copy())
        self.find_next_point()
        degree =self.find_next_degree()
        return degree


    def reset():
        global left_fit_avg, right_fit_avg

        left_fit_avg = None
        right_fit_avg = None

