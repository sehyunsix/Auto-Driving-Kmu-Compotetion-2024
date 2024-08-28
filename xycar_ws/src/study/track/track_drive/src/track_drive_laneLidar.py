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
 
 # 모터 토픽 메시지        print("degree",degree)

  # 모터 노드 변수\

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

def start():

    global motor, ultra_msg, image, imu_data ,roll,pitch,yaw

    cam_exposure(100)  # 카메라의 Exposure 값을 변경

    rospy.init_node('Track_Driver')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)  
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber("/imu",Imu,imu_callback)


    rospy.AnyMsg
   
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")

    rospy.wait_for_message("/imu",Imu)
    
    rospy.wait_for_message("/scan",LaserScan)
    print("Lidar Ready --------------")
    CvBridge
    laneDriver =LaneDriver()
    lidarDriver=LidarDriver()

    plt.ion()
    plt.legend()
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Lines from Fixed Point to Given Points')
    # plt.xlim((-200,2000))
    # plt.ylim((-200,2000))

    old_next_point= 0
    vx=0
    vy=0
    new_state=State(10,10)




    labacon_mode= False
    obstacle_mode= False

    while not rospy.is_shutdown():
        start = time.time() # 시작 


        lanedegree= laneDriver.Driving(image,new_state)
        lidardegree= lidarDriver.Driving(lidar_points)
        print("lidar_degree",lidardegree)
        orangeCount =lidarDriver.filter_orange_color(image)
        print(lidarDriver.obstacle_list)
        print(orangeCount)
        # iniatial Mode
        if( orangeCount>18000 and abs(lidardegree)>1):
            stack=0
            labacon_mode=True


        if(len(lidarDriver.obstacle_list)>0):
            lidarDriver.danger=True
            obstacle_mode=True
                        #Driving

        if labacon_mode:
            print("lidarDriving")
            degree =lidardegree
            speed =10
            
            if(orangeCount<10000 and  len(lidarDriver.obstacle_list)==0):
                stack +=1
            print(stack)
            if(stack>10 ):
                labacon_mode=False
                obstacle_mode=False
                degree = -20
                stack=0
        
        elif obstacle_mode:
            print("ObstacleMode")
            if(lidardegree<0):
                degree=lidarDriver.ObstacleAvoidance(-1)
                
            else:
                degree=lidarDriver.ObstacleAvoidance(1)
            if(lidarDriver.danger==False):
                obstacle_mode=False
            speed=7
            print(degree)

        else:
            print("laneDriving")
            degree=lanedegree
            speed=10
        # print("degree",degree)

        drive(int(degree),speed)
        # draw_dot(laneDriver.nextPoint[0]-old_next_point,laneDriver.nextPoint[1])
        # old_next_point = laneDriver.nextPoint[0]
        # draw_dot(next_point[0],next_point[1])
        ax =np.array(imu_data.linear_acceleration.x, dtype=np.float32)
        ay= np.array(imu_data.linear_acceleration.y,dtype=np.float32)
        dt= np.array(1/30,dtype=np.float32)
        yaw= np.array(yaw)
        # print("yaw :",yaw)
        vx=new_state.vx+ax*dt
        vy=new_state.vy+ay*dt
        new_state=State(vx,vy)
        cv2.imshow("origianl",image)
        cv2.waitKey(1)



        # next_point[0] = next_point[0]+np.cos(yaw)*vx*dt
        # next_point[1] = next_point[1]+np.sin(yaw)*vy*dt
        


0

#threshold  수정하기
#이동평균 angle 추가하기


if __name__ == '__main__':
    start()
