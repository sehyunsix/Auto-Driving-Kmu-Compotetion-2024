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
from arDriver import ARDriver
from utils import draw_dot
from laneDriver import State
from sensor_msgs.msg import Imu

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ar_msg = {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수
imu_data=None
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색

def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)

def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

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
        print(i.pose.pose.position.x, i.pose.pose.position.z)

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

def start():
    global motor, image, ar_msg

    cam_exposure(100)  

    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback, queue_size=1)

    #=========================================
    # 발행자 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    print("AR detector Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")
	
    next_point= [0,0]
    vx=0
    vy=0
    step=0
    degree=0
    index=0
    old_target =[0,0]
    ar_driver  =ARDriver()
    substep =0
    TraficFlag=False

    while not rospy.is_shutdown():
        retry_count = 0
        ID_list=[]
        speed=3
        pass_step =0
        pass_step_limit=1000

        if image is not None:
            # 이미지를 복사하여 사각형을 그릴 이미지 생성
            display_image = image.copy()

            ar_data = ar_msg.copy()
            if (len(ar_data['ID']) != 0):
                rospy.loginfo("now AR_driving")
                 
                # 각 AR 마커에 대해 사각형 그리기
                for i in range(len(ar_data['ID'])):
                    x = ar_data["DX"][i]
                    z = ar_data["DZ"][i]

                    # 좌표 조정
                    top_left = (int(x - 5), int(z - 5))
                    bottom_right = (int(x + 5), int(z + 5))

                    # 사각형 그리기
                    cv2.rectangle(display_image, top_left, bottom_right, (0, 0, 255), 2)
                    cv2.putText(display_image, f"ID:{ar_data['ID'][i]}", (top_left[0], top_left[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            # 수정된 이미지를 화면에 표시
            cv2.imshow('AR following', display_image)
            cv2.waitKey(1)
        
        drive(int(degree), speed)

if __name__ == '__main__':
    start()

