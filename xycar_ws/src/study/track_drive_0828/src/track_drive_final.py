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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers
from xycar_msgs.msg import xycar_motor

import importlib.util
import matplotlib.pyplot as plt
import glob
from laneDriver import LaneDriver ,State
from lidarDriver import LidarDriver
from track_drive_ar import ar_start ,ar_init
  
from utils import logging_time , draw_dot
from tf.transformations import euler_from_quaternion
 
bridge = CvBridge() 
motor = None
lidar_points=None
imu_data=None
motor_msg = xycar_motor()
roll=None
pitch=None
yaw=None
ar_msg=None
speed=0
degree=0
pass_labacon_mode = False


def kill_node(node_name):
    try:
        # rosnode kill 명령어를 사용하여 노드를 종료
        result = os.system(f"rosnode kill {node_name}")
        if result == 0:
            rospy.loginfo(f"Node {node_name} has been killed successfully.")
        else:
            rospy.logwarn(f"Failed to kill node {node_name}. It may not exist.")
    except Exception as e:
        rospy.logerr(f"Failed to kill node {node_name}: {e}")


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

def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)        

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges

def imu_callback(data):
    global imu_data ,roll,pitch,yaw
    imu_data= data
    orientation = [data.orientation.x, data.orientation.y, data.orientation.z,
               data.orientation.w] 
    (roll, pitch, yaw) = euler_from_quaternion(orientation)



def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)


image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수



def init():
    global motor,  image, imu_data ,roll,pitch,yaw ,laneDriver ,lidarDriver

    # cam_exposure(100)  # 카메라의 Exposure 값을 변경

    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)  
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    
    rospy.wait_for_message("/scan",LaserScan)
    print("Lidar Ready --------------")

  
    laneDriver =LaneDriver()
    lidarDriver=LidarDriver()


def LeftObstacleAvoidance():
        
        for i in range(3):
             drive(0,0)
             rospy.sleep(0.1)

        for i in range(10):
            drive(50,7)
            rospy.sleep(0.1)
        
        
        for i in range(17):
            drive(-50,7)
            rospy.sleep(0.1)
        
        for i in range(5):
            drive(25,7)
            rospy.sleep(0.1)
                
        
        
        print("No obstackle")
        return 0

def RightObstacleAvoidance():
    for i in range(3):
        drive(0,0)
        rospy.sleep(0.1)

    for i in range(15):
        drive(-50,7)
        rospy.sleep(0.1)
        
    for i in range(13):
        drive(50,7)
        rospy.sleep(0.1)
    
    for i in range(5):
        drive(-25,7)
        rospy.sleep(0.1)

        
        print("No obstackle")
        return 0

def ObstacleAvoidance(sign):
        
        for i in range(7):
            drive(sign*50,7)
            rospy.sleep(0.1)
        
        for i in range(2):
            drive(0,7)
            rospy.sleep(0.1)
        
        for i in range(10):
            drive(-sign*50,7)
            rospy.sleep(0.1)
        
        print("No obstackle")
        return 0

def auto_exposuer():
    command = 'v4l2-ctl -d /dev/videoCAM -c auto_exposure=3;'
    os.system(command)

def start():
    global motor,  image, imu_data ,roll,pitch,yaw ,laneDriver ,lidarDriver,speed,degree , pass_labacon_mode ,labacon_stack
    labacon_stack=0
    labacon_mode= False
    obstacle_mode= False
    pass_labacon_mode=False
    stack=0
    
    # cam_exposure(200)
    auto_exposuer()
    while not rospy.is_shutdown():

        lanedegree= laneDriver.Driving(image)
        lidardegree= lidarDriver.radius_driving(lidar_points)
        orangeCount =lidarDriver.filter_orange_color(image)
        greenCount = lidarDriver.filter_fluorescent_green(image)
        
        print("obstacle list :",lidarDriver.obstacle_list)
        print("orange count :" ,orangeCount)
        print("green count :",greenCount)
        print("speed :",speed)
        print("degree :",degree)
        print("stack: ",stack)

        if( orangeCount>20000 and abs(lidardegree)>1):
            stack=0
            labacon_mode=True


        if((len(lidarDriver.obstacle_list)>0) and (labacon_mode==False) and (greenCount>=400) and (pass_labacon_mode==True)):
            lidarDriver.danger=True
            obstacle_mode=True            

        if labacon_mode:
            print("================= LidarDrivingMode ==================")
            degree =lidardegree
            speed =17


            if(orangeCount<10000 and  len(lidarDriver.obstacle_list)==0):
                stack +=1

            print(stack)
            if(stack>30 ):
                labacon_mode=False
                obstacle_mode=False
                pass_labacon_mode = True
                labacon_stack=0
                stack=0
                speed=10

        
        elif obstacle_mode:
            print("================= ObstacleMode ===================")
            if(lidardegree<0):
                RightObstacleAvoidance()
                print("Right Obstacle go left")
                
            else:
                LeftObstacleAvoidance()
                print("Left Obstacle go right")

            obstacle_mode=False
            pass_labacon_mode = False

        else:
            print("================= LaneDrivingMode ==================")
            degree=lanedegree
            speed=12

        drive(int(degree),speed)
        cv2.imshow("origianl",image)
        cv2.waitKey(1)


if __name__ == '__main__':
    ar_init()
    init()
    ar_start()
    kill_node("ar_track_alvar")      
    start()
