#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers
import importlib.util
import matplotlib.pyplot as plt
from arDriver import ARDriver
from utils import draw_dot
from laneDriver import State
from lidarDriver import LidarDriver
from sensor_msgs.msg import Imu

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
lidar_points = None  # 라이다 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ar_msg = {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
roll=None
pitch=None
yaw=None
imu_data=None
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

def imu_callback(data):
    global imu_data ,roll,pitch,yaw
    imu_data= data
    orientation = [data.orientation.x, data.orientation.y, data.orientation.z,
               data.orientation.w] 
    (roll, pitch, yaw) = euler_from_quaternion(orientation)



def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c auto_exposure=1; v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute=' + str(value)
    os.system(command)

def auto_exposuer():
    command = 'v4l2-ctl -d /dev/videoCAM -c auto_exposure=3;'
    os.system(command)


def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
#=============================================
def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges


def ar_callback(data):
    global ar_msg

    # AR태그의 ID값, X 위치값, Z 위치값을 담을 빈 리스트 준비
    ar_msg["ID"] = []
    ar_msg["DX"] = []
    ar_msg["DZ"] = []

    # 발견된 모두 AR태그에 대해서 정보 수집하여 ar_msg 리스트에 담음
    for i in data.markers:
        ar_msg["ID"].append(i.id) # AR태그의 ID값을 리스트에 추가
        ar_msg["DX"].append(int(i.pose.pose.position.x*100)) # X값을 cm로 바꿔서 리스트에 추가
        ar_msg["DZ"].append(int(i.pose.pose.position.z*100)) # Z값을 cm로 바꿔서 리스트에 추가

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
       
#=============================================
# 신호등의 파란불을 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_traffic_sign():
    MIN_RADIUS, MAX_RADIUS = 15, 25
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    cimg = image.copy()
    Center_X, Center_Y = 320, 100  # ROI 영역의 중심위치 좌표 
    XX, YY = 220, 80  # 위 중심 좌표에서 좌우로 XX, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄   

    # ROI 영역에 녹색 사각형으로 테두리를 쳐서 표시함 
    cv2.rectangle(cimg, (Center_X-XX, Center_Y-YY), (Center_X+XX, Center_Y+YY) , Green, 2)
	
    # 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
    roi_img = cimg[Center_Y-YY:Center_Y+YY, Center_X-XX:Center_X+XX]

    # roi_img 칼라 이미지를 회색 이미지로 바꾸고 노이즈 제거를 위해 블러링 처리를 함  
    img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Hough Circle 함수를 이용해서 이미지에서 원을 (여러개) 찾음 
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=40, param2=20, 
                  minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

    # 디버깅을 위해서 Canny 처리를 했을때의 모습을 화면에 표시함
    # 위 HoughCircles에서 param1, param2에 사용했던 값을 아래 canny에서 똑같이 적용해야 함. 순서 조심.
    canny = cv2.Canny(blur, 20, 40)
    # cv2.imshow('Canny image used by HoughCircles', canny)
    # cv2.waitKey(1)

    if circles is not None:
    
        # 정수값으로 바꾸고 발견된 원의 개수를 출력
        circles = np.round(circles[0, :]).astype("int")
        print("\nFound",len(circles),"circles")
        
        # 중심의 Y좌표값 순서대로 소팅해서 따로 저장
        
        y_circles = sorted(circles, key=lambda circle: circle[1])
 
        # 중심의 X좌표값 순서대로 소팅해서 circles에 다시 저장
        circles = sorted(circles, key=lambda circle: circle[0])
         
        # 발견된 원들에 대해서 루프를 돌면서 하나씩 녹색으로 그리기 
        for i, (x, y, r) in enumerate(circles):
            cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Green, 2)
 
    # 이미지에서 정확하게 3개의 원이 발견됐다면 신호등 찾는 작업을 진행  
    if (circles is not None) and (len(circles)==3):
            
        # 가장 밝은 원을 찾을 때 사용할 변수 선언
        max_mean_value = 0
        max_mean_value_circle = None
        max_mean_value_index = None

        # 발견된 원들에 대해서 루프를 돌면서 하나씩 처리 
 	    # 원의 중심좌표, 반지름. 내부밝기 정보를 구해서 화면에 출력
        for i, (x, y, r) in enumerate(circles):
            roi = img[y-(r//2):y+(r//2),x-(r//2):x+(r//2)]
            # 밝기 값은 반올림해서 10의 자리수로 만들어 사용
            mean_value = round(np.mean(roi),-1)
            print(f"Circle {i} at ({x},{y}), radius={r}: brightness={mean_value}")
			
            # 이번 원의 밝기가 기존 max원보다 밝으면 이번 원을 max원으로 지정  
            if mean_value > max_mean_value:
                max_mean_value = mean_value
                max_mean_value_circle = (x, y, r)
                max_mean_value_index = i
                
            # 원의 밝기를 계산했던 사각형 영역을 빨간색으로 그리기 
            cv2.rectangle(cimg, ((x-(r//2))+Center_X-XX, (y-(r//2))+Center_Y-YY),
                ((x+(r//2))+Center_X-XX, (y+(r//2))+Center_Y-YY), Red, 2)

        # 가장 밝은 원을 찾았으면 그 원의 정보를 출력하고 노란색으로 그리기 
        if max_mean_value_circle is not None:
            (x, y, r) = max_mean_value_circle
            print(f" --- Circle {max_mean_value_index} is the brightest.")
            cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Yellow, 2)
            
        # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
        # cv2.imshow('Circles Detected', cimg)
        
        # 제일 위와 제일 아래에 있는 2개 원의 Y좌표값 차이가 크면 안됨 
        vertical_diff = MAX_RADIUS * 2
        if (y_circles[-1][1] - y_circles[0][1]) > vertical_diff:
            print("Circles are scattered vertically!")
            return False
        
        # 제일 왼쪽과 제일 오른쪽에 있는 2개 원의 X좌표값 차이가 크면 안됨 
        horizontal_diff = MAX_RADIUS * 8
        if (circles[-1][0] - circles[0][0]) > horizontal_diff:
            print("Circles are scattered horizontally!")
            return False      
            
        # 원들이 좌우로 너무 붙어 있으면 안됨 
        min_distance = MIN_RADIUS * 3
        for i in range(len(circles) - 1):
            if (circles[i+1][0] - circles[i][0]) < min_distance:
                print("Circles are too close horizontally!")
                return False 
            
        # 3개 중에서 세번째 원이 가장 밝으면 (파란색 신호등) True 리턴 
        if (max_mean_value_index == 2):
            print("Traffic Sign is Blue...!")
            return True
        
        # 첫번째나 두번째 원이 가장 밝으면 (파란색 신호등이 아니면) False 반환 
        else:
            print("Traffic Sign is NOT Blue...!")
            return False

    # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
    cv2.imshow('Circles Detected', cimg)
    
    # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
    #print("Can't find Traffic Sign...!")
    return False






def fisrt_move():
    for  i in range(20):
        drive(-50 , 5)
        rospy.sleep(0.1)
    for  i in range(5):
        drive(0 , 5)
        rospy.sleep(0.1)
    for  i in range(13):
        drive(30 , 5)
        rospy.sleep(0.1)
    for  i in range(10):
        drive(0 , 5)
        rospy.sleep(0.1)

def second_move():
    for  i in range(5):
        drive(0 , 5)
        rospy.sleep(0.1)

# def third_move():



def filter_fluorescent_green(image):
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
        fluorescent_green_mask [300:480,:]=0
        cv2.imshow("greeb mask",fluorescent_green_mask)
        cv2.imshow ("ROI",image[:370,:] )
        # 형광 초록색 픽셀의 총 개수 계산
        fluorescent_green_pixel_count = cv2.countNonZero(fluorescent_green_mask)

        return fluorescent_green_pixel_count


def ar_init():
    global ar_driver ,TraficFlag,motor
    cam_exposure(200)  
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback, queue_size=1 )
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
    print("AR detector Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    ar_driver  =ARDriver()
    TraficFlag=False

def ar_start():

    global motor, image ,ar_msg  ,roll,pitch,ar_driver, TraficFlag,motor
    
    rospy.init_node('Track_Driver')


    step=0
    degree=0
    index=0
    ar_driver  =ARDriver()
    substep =0
    TraficFlag=False
    line_speed=3
    # fisrt_move()


    while not rospy.is_shutdown():
        retry_count = 0
        speed=3

        cv2.imshow('AR following', image)
        cv2.waitKey(1)


        if step==0:
            print("===========STEP0 =================")
            for i in range(10):
                drive(0 ,5)
                rospy.sleep(0.1)
            rospy.sleep(5)
            fisrt_move()
            step +=1
            substep=1
      
        ar_data =ar_msg.copy()
        if (len(ar_data['ID'])!=0 or substep ==1):
            if(len(ar_data['ID'])!=0):   
                  index=np.argmin(np.array(ar_data["DZ"]))
                  target= (ar_data["DX"][index],ar_data["DZ"][index])
                  print(f"target DX: {ar_data['DX'][index]} DZ: {ar_data['DZ'][index]}")
            print(f"speed: {speed} degree {degree}")


            if step==1:
                print("===========STEP1 =================")
                print("substep : see traffic_sign")
                speed=0
                substep=1
                if(TraficFlag==False):
                    TraficFlag= check_traffic_sign()
                    green_count =filter_fluorescent_green(image)
                    if(green_count>1000):
                        TraficFlag=True
                    print("grenn count",green_count)
                else:
                    step+=1
                    second_move()
                    substep=0
                    continue
                    
            
            
            elif step==2:
                    print("===========STEP2 =================")
                    speed ,degree=ar_driver.move_to_sign(target ,(-10,-25),90)
                    print("substep 0:  target Driving")
                    if(speed==0 and substep==0):
                        substep=1
                        continue
                    
                    if(speed==0 and substep==1):
                        print("substep 1: Non target Driving")
                        for i in range(15):
                            drive(-45,5)
                            rospy.sleep(0.1)
                            substep=0
                        step +=1

            
            
            elif step==3:
                    print("===========STEP3 =================")
                    speed ,degree=ar_driver.move_to_sign(target ,(-30,-25),60)
                    print("substep 0:  target Driving")
                    if(speed==0 and substep==0):
                        substep=1
                        continue
                    
                    if(speed==0 and substep==1):
                        print("substep 1: Non target Driving")
                        for i in range(10):
                            drive(-41,5)
                            rospy.sleep(0.1)
                            substep=0
                        for i in range(12):
                            drive(40,5)
                            rospy.sleep(0.1)          
                        for i in range(3):
                            drive(0,5)
                            rospy.sleep(0.1)
                        step+=1
                        substep=0
                        continue
            

            elif step==4:
                    print("===========STEP4 =================")
                    speed ,degree=ar_driver.move_to_sign(target ,(-10,-25),70)
                    
                    print("substep 0:  target Driving")
                    if(speed==0 and substep==0):
                        substep=1
                        
                    
                    if(speed==0 and substep==1):
                        for i in range(15):
                            drive(-50,5)
                            rospy.sleep(0.1)
                      
                        step+=1
                        substep=0
                        continue
                    
                 
                
                  
            elif step==5:
                    print("===========STEP5 =================")
                    speed ,degree=ar_driver.move_to_sign(target ,(-40,-25),60)
                    print("substep 0:  target Driving")
                    if(speed==0 and substep==0):
                        substep=1
                    
                    if(speed==0 and substep==1):
                        for i in range(12):
                            drive(-50,5)
                            rospy.sleep(0.1)
                  
                        step+=1
                        substep=0
                        continue
     
            
            elif step==6:
                    print("===========STEP6 =================")
                    speed ,degree=ar_driver.move_to_sign(target ,(-40,-25),70)
                    print("substep 0:  target Driving")
                    if(speed==0 and substep==0):
                        substep=1
                        print("substep 1: Non target Driving")
                        continue

                    if(substep==1):
                        substep=1
                        for i in range(10):
                            drive(-45, 5)
                            rospy.sleep(0.1)
                        for i in range(10):
                            drive(10,5)
                            rospy.sleep(0.1)
                        step+=1                      
                    
            elif step==7:
                    print("===========STEP7 =================")
                    speed=0
                    print("ARfinish wait 3 seconds")
                    rospy.sleep(3)
                    return
        else:
            # ar_driver.find_first_AR()
            print("Keep going...",  "retry_count :", retry_count) 
            speed=0
                
        print(degree)
        drive(int(degree),speed)
    

       
    
if __name__ == '__main__':
    ar_start()
