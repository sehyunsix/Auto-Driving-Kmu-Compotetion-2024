#!/usr/bin/env python

import pickle
import rospy
from xycar_msgs.msg import xycar_motor
import time
motor_msg = xycar_motor()  # 모터 토픽 메시지

save_list = []

with open('/home/xytron/xycar_ws/src/study/track/track_drive/src/path/test.pkl','rb') as f:
    path_list =pickle.load(f)

for i in range(len(path_list)):
    save_list.append(path_list[i])
'''
for i in range(len(path_list)):
    print(save_list[i])
'''


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

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

rospy.init_node("xycar_motor_printer")
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)


rospy.sleep(10)
# RightObstacleAvoidance()
LeftObstacleAvoidance()
