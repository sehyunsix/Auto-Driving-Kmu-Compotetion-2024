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
from lidarDriver import LidarDriver

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수\
debug =rospy.get_param("debug")
Fix_Speed = rospy.get_param('speed')  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
lidar_points = None  # 라이다 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
stopline_num = 1 # 정지선 발견때마다 1씩 증가
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치
ar_msg = {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수

#=============================================
# 학습결과 파일의 위치 지정
#=============================================
PATH_TO_CKPT = '/home/pi/xycar_ws/src/study/track_drive/src/detect.tflite'
PATH_TO_LABELS = '/home/pi/xycar_ws/src/study/track_drive/src/labelmap.txt'

#=============================================
# 차선인식 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30  # 카메라 FPS 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ROI_START_ROW = 300  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 380  # 차선을 찾을 ROT 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 40  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 





#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
#=============================================
def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges


    
#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수  
# 입력으로 지속시간을 받아 그 시간동안 속도=0 토픽을 모터로 보냄.
# 지속시간은 0.1초 단위임. 만약 15이면 1.5초가 됨.
#=============================================
def stop_car(duration):
    for i in range(int(duration)): 
        drive(angle=0, speed=0)
        time.sleep(0.1)
    
#=============================================
# 차량을 이동시키는 함수 
# 입력으로 조향각과 속도, 지속시간을 받아 차량을 이동시킴.
# 지속시간은 0.1초 단위임. 만약 15이면 1.5초가 됨. 
#=============================================
def move_car(move_angle, move_speed, duration):
    for i in range(int(duration)): 
        drive(move_angle, move_speed)
        time.sleep(0.1)
		
#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받는다.
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
    


#=============================================
# 라이다 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 모터로 토픽을 보내는 함수
#=============================================
def sensor_drive():
    print("drive")
    global new_angle, new_speed
    ranges = lidar_points
    # print({a: data.__getattribute__(a) for a in dir(data) if not a.startswith('__')})
   
    ran =lidar_points
    ranges = np.zeros(505, dtype=float)
    for i in range(0,253):
        ranges[i+252] = ran[i]
    for i in range(252,504):
        ranges[i-252] = ran[i]
    angles =np.linspace(0, 2*np.pi,len(ranges))
    angles = angles-np.pi/2
    x=ranges *np.cos(angles)
    y=ranges*np.sin(angles)
    
  
    if ((lidar_points[45]*100 - 10 > lidar_points[460]*100 )or lidar_points[45]<0.01):
        new_angle = -50

    # 왼쪽 벽보다 오른쪽 벽이 멀리 있으면, 오른쪽으로 주행
    elif (lidar_points[45]*100 < lidar_points[460]*100 - 10 or lidar_points[460] <0.01):
        new_angle = 50

    # 위 조건들에 해당하지 않는 경우라면 직진 주행
    else:
        new_angle = 0

    ranges1= np.random.uniform(0,1,len(ranges))
    x1 ,y1= ranges1*np.cos(np.zeros(len(ranges))) ,ranges1*np.sin(np.zeros(len(ranges)))
    draw_angle=0
    
    if new_angle<0:
        draw_angle = 180 + new_angle

    else:
        draw_angle= new_angle

    x2, y2 = ranges1*np.cos(np.full(len(ranges),math.radians(draw_angle))) ,ranges1*np.sin(np.full(len(ranges), math.radians(draw_angle)))
    x3, y3 = ranges1*np.cos(np.full(len(ranges),angles[314])) ,ranges1*np.sin(np.full(len(ranges),angles[314]))
    x4, y4 = ranges1*np.cos(np.full(len(ranges),angles[192])) ,ranges1*np.sin(np.full(len(ranges),angles[192]))



    dot1 =plt.scatter(x3, y3,c='r',marker='.',label ="point")
    dot3 =plt.scatter(x1,y1,c='g',marker='.',label ="entry")
    dot =plt.scatter(x,y , c='b',marker='.' ,label='LIDAR points')
    dot2 =plt.scatter(x2,y2,c='y',marker='.',label ="entry")
    dot4 =plt.scatter(x4, y4,c='g',marker='.',label ="points")

    plt.xlim([-1, 1])
    plt.ylim([0, 2])
    plt.show()

    plt.pause(0.001)
    dot.remove()
    dot1.remove()
    dot2.remove()
    dot3.remove()
    dot4.remove()

    # # 모터에 주행명령 토픽을 보낸다
    # print("right_distance :", lidar_points[45]*100,"left_distance :",lidar_points[460]*100)
    # print("angle :", new_angle, "speed :",new_speed) 

    if(debug != 1):
         drive(new_angle, new_speed)
   
      

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image 
    global new_angle, new_speed
   
    STARTING_LINE = 1
    TRAFFIC_SIGN = 2
    SENSOR_DRIVE = 3
    LANE_DRIVE = 4
    AR_DRIVE = 5
    PARKING = 7
    FINISH = 9
		
    # 처음에 어떤 미션부터 수행할 것인지 여기서 결정합니다. 
    cam_exposure(100)  # 카메라의 Exposure 값을 변경
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)  
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)


 
    print("Lidar Ready ----------")
    rospy.wait_for_message("/scan", LaserScan)
    # print("AR detector Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")
	
    # 일단 차량이 움직이지 않도록 정지상태로 만듭니다.  
	
    #=========================================
    # 메인 루프 
    #=========================================
    plt.ion()
    lidarDriver=LidarDriver()
    while not rospy.is_shutdown():
        degree =lidarDriver.Driving(lidar_points)
        drive(degree,8)
        # sensor_drive()
        # time.sleep(0.1)
        # rospy.spin()


if __name__ == '__main__':
    start()