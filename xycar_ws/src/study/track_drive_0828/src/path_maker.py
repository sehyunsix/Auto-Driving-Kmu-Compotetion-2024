#!/usr/bin/env python


import numpy as np
import cv2, rospy, time, math, os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import matplotlib.pyplot as plt
import pickle

path_list =[]

def motor_callback(data):
    if data.speed > 0:
        path_list.append((data.angle ,data.speed))
        print(data.speed)
        with open('/home/xytron/xycar_ws/src/study/track/track_drive/src/path/test.pkl', 'wb') as f:
            pickle.dump(path_list, f)

if __name__ =="__main__":
    rospy.init_node("Path maker")
    rospy.Subscriber('/xycar_motor', xycar_motor, motor_callback, queue_size=1 )
    rospy.spin()