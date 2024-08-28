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
rospy.init_node("xycar_motor_printer")
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

for i in range(len(path_list)):
    angle = save_list[i][0]
    speed = save_list[i][1]
    drive(angle, speed)
    rospy.sleep(0.01)

'''
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

rospy.init_node('xycar_motor_printer')
       
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)


next=0

while not rospy.is_shutdown():
    if (next <len(path_list)):
        data= path_list[next]
        drive(data.angle,data.speed)
        cv2.waitKey(1)
        next += 1
'''