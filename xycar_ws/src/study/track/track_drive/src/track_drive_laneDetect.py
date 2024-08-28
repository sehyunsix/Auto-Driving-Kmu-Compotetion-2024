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
from xycar_msgs.msg import xycar_motor

import importlib.util
import matplotlib.pyplot as plt
import glob
from study.track.track_drive.src.laneDriver import LaneDetecter ,logging_time
bridge = CvBridge() 
motor = None
motor_msg = xycar_motor()  # 모터 토픽 메시지
  # 모터 노드 변수\

def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)

def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)


image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수

def start():

    global motor, ultra_msg, image 

    cam_exposure(100)  # 카메라의 Exposure 값을 변경

    rospy.init_node('Track_Driver')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)  
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
   
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")


    ld =LaneDetecter()



    while not rospy.is_shutdown():
        start = time.time() # 시작
        ld.detect_lanes(image.copy(),'test.jpg')
        ld.find_next_point()
        degree =ld.find_next_degree()
        drive(int(degree),10)
        print(f"{time.time()-start:.4f} sec") # 종료와 함께 수행시간 출력
        cv2.imshow('test',image)
        cv2.waitKey(1)
        
        

#threshold  수정하기
#이동평균 angle 추가하기


if __name__ == '__main__':
    start()
