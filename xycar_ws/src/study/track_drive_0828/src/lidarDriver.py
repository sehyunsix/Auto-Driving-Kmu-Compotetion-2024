#!/usr/bin/env python

import rospy
import numpy as np
import math
import cv2



INT_MAX=987654321

class MovingAverage:

    # 클래스 생성과 초기화 함수 (데이터의 개수를 지정)
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    # 새로운 샘플 데이터를 추가하는 함수
    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data.pop(0)  # 가장 오래된 샘플 제거
            self.data.append(new_sample)

    # 저장된 샘플 데이터의 갯수를 구하는 함수
    def get_sample_count(self):
        return len(self.data)

    # 이동평균값을 구하는 함수
    def get_mavg(self):
        if not self.data:
            return 0.0
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        if not self.data:
            return 0.0
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        if not self.data:
            return 0.0

        s = sum(x * w for x, w in zip(self.data, self.weights[:len(self.data)]))
        return float(s) / sum(self.weights[:len(self.data)])

    # 샘플 데이터 중에서 제일 작은 값을 반환하는 함수
    def get_min(self):
        if not self.data:
            return 0.0
        return float(min(self.data))
    
    # 샘플 데이터 중에서 제일 큰 값을 반환하는 함수
    def get_max(self):
        if not self.data:
            return 0.0
        return float(max(self.data))


class LidarDriver:

    angle_avg_count = 10# 이동평균값을 계산할 데이터 갯수 지정
    angle_avg = MovingAverage(angle_avg_count)
    current_raw_lidar =[]
    current_front_lidar_list=[]
    current_theta_lidar =[]
    current_raw_lidar= []
    filtering_lidar=[]
    boundary_list=[]
    obstacle_list=[]
    second_boundary_list=[]
    # safeBoundaryWidth = 30
    
    #SPEED 8
    # lookahead=20
    # safeBoundaryWidth=20
    # safeBoundaryHegith = 70
    # safeMargin= 20

    #SPEED 10
    lookahead=15
    safeBoundaryWidth=20
    safeBoundaryHegith = 90
    obstacleBoundaryHegith =50
    safeMargin= 20


    LeftPoint=0
    RightPoint=0
    nextPoint=None
    LeftPointNumber=0
    RightPointNumber=0

    lf=18
    ld=14
    
    #SPEED 8


    #avoidance
    avoid_step =0
    go_step =0
    comback_step=0

    ##  -2*pi < theta < 2*pi
    #   -50 < degree  < 50
    def makeMeter(self):
        self.current_raw_lidar = [x*100 for x in self.current_raw_lidar] 

    def makeFrontPointList(self):
        self.current_front_lidar_list= self.current_raw_lidar[380:-1 ]+self.current_raw_lidar[:125]
        # print(self.current_front_lidar_list)
        # print(self.current_front_lidar_list)

    def indextoTheta(self,index):
        delta=2*np.pi/504
        assert len(self.current_raw_lidar) == 505
        return index*delta
    

    def radius_find_next_point(self,):
        max_radius_distance =0
        idx=0
        r=0
        theta =self.indextoTheta(idx)
        max_radius_distance =0
        max_radius_index=[]

        start_idx =0
        end_idx=0
        while(self.indextoTheta(start_idx)<=np.pi and end_idx<len(self.current_front_lidar_list)-1):
            end_idx=start_idx+1
            print(end_idx)
            while(end_idx<len(self.current_front_lidar_list)-1 and self.current_front_lidar_list[end_idx]<15):
                end_idx +=1
                
            distance = ((self.current_front_lidar_list[start_idx] +self.current_front_lidar_list[end_idx])/2)
            theta = self.indextoTheta(end_idx- start_idx)
            radius_distance =distance*theta 
            print("radius_distance", radius_distance)
            if(radius_distance>max_radius_distance):
                max_radius_index= [start_idx,end_idx]
                max_radius_distance= radius_distance
            start_idx=end_idx

        if (len(max_radius_index)==0):
            self.nextPoint=[0,0]
            return
        avg_theta = self.indextoTheta(int((max_radius_index[0]+max_radius_index[1])/2))
        print("next index",max_radius_index)
        avg_distance = (self.current_front_lidar_list[max_radius_index[0]] + self.current_front_lidar_list[max_radius_index[1]])/2
        self.nextPoint = [math.cos(avg_theta)*avg_distance ,math.sin(avg_theta)*avg_distance]
        print("next_point",self.nextPoint)
        return

       
    def radius_driving_next_point(self):
        if (self.nextPoint ==None):
            return 0 
        if(self.nextPoint[1]<0.01):
            return 0
        beta = abs(math.atan((math.sin(math.atan(abs(self.nextPoint[0]/self.nextPoint[1])))*(self.lf+self.ld)/self.ld)))
        
        if self.nextPoint[0] >0:
          degree= beta*180/np.pi
        else:
          degree = -beta*180/np.pi
        # degree =self.angle_avg.get_wmavg()
        # self.angle_avg.add_sample(degree)
        # assert degree < 0 , print("degree", degree)
        assert -90<= degree <= 90 , print("degree",degree)
        return degree
      
        



    def makeBoundaryPointList(self):
        for idx, r in enumerate(self.current_front_lidar_list):
            theta =self.indextoTheta(idx)
            # print(theta,math.cos(theta))
          # print(r,theta)
            if(0<= theta and theta<= np.pi):            
                # print(theta,math.cos(theta))
              
                if(math.cos(theta)<0):
                    # print("left")                        
                    if(abs(math.sin(theta)*r)<10):
                        # print("debug left")map
                        self.LeftPoint += 500
                    else:
                        self.LeftPoint += r
                    self.LeftPointNumber +=1
                else:
                    if(abs(math.sin(theta)*r)<10):
                        self.RightPoint += 500
                    else:   

                        self.RightPoint +=r
                    self.RightPointNumber +=1

                if(np.pi*3.0/7<= theta <= np.pi*4.2/7): 
                    if( abs(math.cos(theta)*r) and 0.1<= abs(math.sin(theta)*r)):       
                        if(abs(math.cos(theta)*r) <= self.safeBoundaryWidth): 
                            if(abs(math.sin(theta)*r) <= self.safeBoundaryHegith):
                                if(16<=abs(r)):
                                    self.boundary_list.append((r,theta))
                                    if(abs(math.sin(theta)*r) <= self.obstacleBoundaryHegith):
                                        self.obstacle_list.append((r,theta))
                
           
                elif(np.pi*2.0/7<= theta <= np.pi*4.2/7):
                        if( abs(math.cos(theta)*r) and 0.1<= abs(math.sin(theta)*r)):       
                            if(abs(math.cos(theta)*r) <= self.safeBoundaryWidth): 
                                if(abs(math.sin(theta)*r) <= self.safeBoundaryHegith):
                                    if(16<=abs(r)):
                                        self.second_boundary_list.append((r,theta))
                                    

   
                

    def findNextMovePoint(self):
        min= (INT_MAX ,0)
        # print(self.boundary_list)
        if (len(self.boundary_list) ==0):
            print("secondary")
            for r, theta in self.second_boundary_list:
                if(2 <r <min[0]):
                    min=(r,theta)
        else:
            for r, theta in self.boundary_list:
                if(2 <r <min[0]):
                    min=(r,theta)

        assert min[0] != INT_MAX
        print('target',math.cos(min[1])*r,math.sin(min[1])*r, min[1] , theta)
        delta_x = (self.safeBoundaryWidth-abs(math.cos(min[1])*r))+self.safeMargin
        delta_y= abs(math.sin(min[1]))*r +self.lookahead
        if(delta_y <0.1):
           return 0
        beta = abs(math.atan((math.tan(delta_x/delta_y)*(self.lf+self.ld)/self.ld)))
    
        # if(self.LeftPointNumber>0 and self.RightPointNumber>0):
        # if min[1] >np.pi/2:
        if self.LeftPointNumber == 0:
           self.LeftPoint += INT_MAX
           self.LeftPointNumber =1

        if self.RightPointNumber == 0:
           self.RightPoint += INT_MAX
           self.RightPointNumber =1
        print(self.LeftPoint/self.LeftPointNumber)
        print(self.RightPoint/self.RightPointNumber)
        degree=0
        if self.LeftPoint/self.LeftPointNumber <self.RightPoint/self.RightPointNumber:
          degree= beta*180/np.pi
        else:
          degree = -beta*180/np.pi
        # self.angle_avg.add_sample(degree)
        # degree = self.angle_avg.get_mavg()
        assert -90<= degree <= 90 , print("degree",degree)
        self.LeftPoint =0
        self.RightPoint=0
        self.RightPointNumber=0
        self.LeftPointNumber=0
        return int(degree)
    
    def ARfindNextMovePoint(self):
        assert self.LeftPoint[2] > self.RightPoint[2], f"LeftPoint : {self.LeftPoint[1]} and RightPoint: {self.RightPoint[1]}"
        self.nextPoint = [self.piLeftPoint[1],self.LeftPoint[2]-self.RightPoint[2]]
        return 
    
    def drivingNextPoint(self):
        assert self.nextPoint != None 
        beta = abs(math.atan((math.tan(abs(self.nextPoint[0]/self.nextPoint[1]))*(self.lf+self.ld)/self.ld)))
        
        if min[1] >np.pi/2:
          degree= beta*180/np.pi
        else:
          degree = -beta*180/np.pi
        # degree =self.angle_avg.get_wmavg()
        self.angle_avg.add_sample(degree)
        assert degree < 0 , print("degree", degree)
        assert -90<= degree <= 90 , print("degree",degree)
        return degree
    
    def findLeftRightPoint(self):
        
      for idx,r in enumerate(self.current_front_lidar_list):
        if(r>1):
          self.LeftPoint= [idx, math.cos(self.current_front_lidar_list[idx]),math.sin(self.current_front_lidar_list[idx])]
          break
      
      for i in range(len(self.current_front_lidar_list),0 ,-1):
        if(r>1):
          self.RightPoint =self.LeftPoint= [idx, math.cos(self.current_front_lidar_list[idx]),math.sin(self.current_front_lidar_list[idx])]
          break
      print(f"findnextPoint Right { self.LeftPoint } Left{self.RightPoint }")
      return

        

    def Driving(self,lidar_points):
       self.boundary_list=[]
       self.obstacle_list=[]
       self.current_raw_lidar =lidar_points
       self.makeMeter()
       self.makeFrontPointList()
       self.makeBoundaryPointList()
      #  print(self.boundary_list)
       if (len(self.second_boundary_list)==0 and len(self.boundary_list)==0):
        #   print("No obstacle")
          return 0
       degree = self.findNextMovePoint()
    #    print(degree)
       return int(degree)
    
    def ObstacleAvoidance(self,sign):
        if(self.danger and self.comback_step==0):
            self.avoid_step =10
            self.go_step =2
            self.comback_step=20
        # if(len(self.boundary_list)==0):
        #     return 0
        
        while self.avoid_step>0:
            self.avoid_step -= 1
            return sign*40
        
        while self.go_step>0:
            self.go_step -=1
            return 0
        
        while self.comback_step>0:
            self.comback_step -=1
            if(self.comback_step==0):
               self.danger =False
            return sign*-40
        
        print("No obstackle")
        return 0
    
    def make_obstacle_list(self):
        self.obstacle_list=[]
        for idx, r in enumerate(self.current_front_lidar_list):
            theta =self.indextoTheta(idx)
            if(0<= theta and theta<= np.pi):            
                if( abs(math.cos(theta)*r) and 10 <= abs(math.sin(theta)*r)):       
                    if(abs(math.cos(theta)*r) <= self.safeBoundaryWidth): 
                        if(abs(math.sin(theta)*r) <= self.safeBoundaryHegith):
                            if(16<=abs(r)):
                                    if(abs(math.sin(theta)*r) <= self.obstacleBoundaryHegith):
                                        self.obstacle_list.append((r,theta))
                


    
    def radius_driving(self,lidar_points):
       self.boundary_list=[]
       self.current_raw_lidar =lidar_points
       self.makeMeter()
       self.makeFrontPointList()
       self.make_obstacle_list()
       if(len(self.current_front_lidar_list)==0):
           print("Error")
           return 0
       self.radius_find_next_point()
       degree =self.radius_driving_next_point()
       print(degree)
       return int(degree)
        

       
    def ARDriving(self,lidar_points):
        self.boundary_list=[]
        self.second_boundary_list=[]
        self.current_raw_lidar =lidar_points
        self.makeMeter()
        self.makeFrontPointList()
        self.findLeftRightPoint()
        self.ARfindNextMovePoint()
        degree =self.drivingNextPoint()
        print(degree)
        return int(degree)

        
    def filter_orange_color(self,image):
        """
        이미지에서 주황색 픽셀을 필터링하고 그 개수를 계산하는 함수.

        Parameters:
        image (np.array): 입력 이미지 (BGR 형식)

        Returns:
        int: 주황색 픽셀의 총 개수
        np.array: 주황색만 필터링된 이진화 이미지
        """
        # BGR 이미지를 HSV 색 공간으로 변환
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 주황색에 해당하는 HSV 범위 정의
        lower_orange = np.array([10, 150, 150])
        upper_orange = np.array([25, 255, 255])                       


        # 주황색 영역을 필터링하여 이진화 이미지 생성
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        orange_mask[370:480,:]=0

        # 주황색 픽셀의 총 개수 계산
        orange_pixel_count = cv2.countNonZero(orange_mask)
        # cv2.imshow('Original Image', image)
        cv2.imshow('Orange Filtered Image', orange_mask)

        return orange_pixel_count
    
    def filter_fluorescent_green(self,image):
        """
        이미지에서 형광 초록색 픽셀을 필터링하고 그 개수를 계산하는 함수.

        Parameters:
        image (np.array): 입력 이미지 (BGR 형식)

        Returns:
        int: 형광 초록색 픽셀의 총 개수
        np.array: 형광 초록색만 필터링된 이진화 이미지
        """
        # BGR 이미지를 HSV 색 공간으로 변환
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 형광 초록색에 해당하는 HSV 범위 정의
        lower_fluorescent_green = np.array([35, 100, 100])  # 예시: H(35), S(100), V(100)
        upper_fluorescent_green = np.array([85, 255, 255])  # 예시: H(85), S(255), V(255)

        # 형광 초록색 영역을 필터링하여 이진화 이미지 생성
        fluorescent_green_mask = cv2.inRange(hsv_image, lower_fluorescent_green, upper_fluorescent_green)
        fluorescent_green_mask [370:480,:]=0
        cv2.imshow("greeb mask",fluorescent_green_mask)
        cv2.imshow ("ROI",image[:370,:] )
        # 형광 초록색 픽셀의 총 개수 계산
        fluorescent_green_pixel_count = cv2.countNonZero(fluorescent_green_mask)

        return fluorescent_green_pixel_count



       
        
    