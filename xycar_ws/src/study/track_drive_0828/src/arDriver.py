
import math
import matplotlib.pyplot as plt
import numpy as np
from utils import logging_time

class ARDriver:
        lf=10
        ld=40
        nextPoint=[]
        LeftPoint=[]
        RightPoint=[]
        next=[]
        safeDegree=-30
        safeDistance=60
        lookahead=10
        ar_msg=[]
        
        @logging_time
        def ARfindNextMovePoint(self):
            if(len(self.ar_msg['DX'])==0):
                return
            if self.LeftPoint[0] == self.RightPoint[0]:
                self.nextPoint = [self.LeftPoint[1],self.LeftPoint[2]]
                return
            
            if self.LeftPoint[2] < self.RightPoint[2]:
               self.nextPoint = [self.RightPoint[1],(self.LeftPoint[2]+self.RightPoint[2])/2]
               return

            # assert self.LeftPoint[2] > self.RightPoint[2], f"LeftPoint : {self.LeftPoint[1]} and RightPoint: {self.RightPoint[1]}"
            

            self.nextPoint = [self.LeftPoint[1],(self.LeftPoint[2]+self.RightPoint[2])/2]
            return 
        
        @logging_time
        def drivingNextPoint(self):
            if(len(self.nextPoint ) == 0):
                return  0
            if(self.nextPoint[1]==0):
                return 0
            beta = abs(math.atan((math.tan(abs(self.nextPoint[0]/(self.nextPoint[1]+self.lookahead)))*(self.lf+self.ld)/self.ld)))
            # l =plt.scatter( [self.nextPoint[0]],  [self.nextPoint[1]], c='r',marker='.',label ="points")
            # self.next.append(l) # 각 점 표시
     
            degree = -beta*180/np.pi

            if self.nextPoint[0]>0:
                degree = beta*180/np.pi

            # assert degree <= 0 , print("degree", degree)
            assert -90<= degree <= 90 ,print("degree",degree)
            return degree
        
        @logging_time
        def checkDistance(self,target,safeDistance):
            min_z =target[1]
            if(min_z<safeDistance):
                return True
            return False
        @logging_time
        def findLeftRightPoint(self,ar_msg):
            if(len(ar_msg['DX'])==0):
                return
            min_x =min(ar_msg['DX'])
            idx=ar_msg['DX'].index(min_x)
            self.LeftPoint= [ar_msg['ID'][idx],ar_msg['DX'][idx],ar_msg['DZ'][idx]]
            
            max_x =max(self.ar_msg['DX'])
            idx=self.ar_msg['DX'].index(max_x)
            self.RightPoint= [self.ar_msg['ID'][idx],ar_msg['DX'][idx],ar_msg['DZ'][idx]]
            
            return
        @logging_time
        def ARDriving(self,ar_msg):
            self.ar_msg=ar_msg
            self.findLeftRightPoint(self.ar_msg)
            self.ARfindNextMovePoint()
            degree  = self.drivingNextPoint()
            if(self.checkDistance(self.nextPoint,self.safeDistance)):
                return self.safeDegree
            print(f"findnextPoint Right { self.LeftPoint } Left{self.RightPoint }")
            print(f"findnextPoint Next { self.nextPoint } ")
            print(f"Driving Degree { degree } ")
          
            return int(degree)
        



        @logging_time
        def draw_lines(self,fixed_point, points):

            line_list=[]
            # 각 점에 대해 선 그리기
            for point in points:
                # plt.plot([fixed_point[0], fixed_point[0]], [point[0], point[1]], 'b-')
                if(point[0]>=0):
                    x_list= [x for x in range(0, point[0])]
                else:
                    x_list= [x for x in range(point[0],0)]
                

                y_list =[x*(point[1]/point[0]) for x in x_list]
                line =plt.scatter( x_list,  y_list, c='g',marker='.',label ="points")  # 각 점 표시
                line_list.append(line)
                
            plt.draw()
            plt.pause(0.001)
            if(len(self.next) != 0):
                for n in self.next:
                    n.remove()
                self.next=[]
                
            for l in line_list:
                l.remove()
      
        def find_first_AR(self):
            self.nextPoint=[-35,100]
            degree =self.drivingNextPoint()
            return degree
        
        def move_to_sign(self,target_pos,offset, stop_distance):
            self.nextPoint =[target_pos[0]+offset[0], target_pos[1]+offset[1]]
            if(self.checkDistance(target_pos,stop_distance)):
                speed=0
                degree=0
                return speed, degree
            degree  = self.drivingNextPoint()
            return 3, degree