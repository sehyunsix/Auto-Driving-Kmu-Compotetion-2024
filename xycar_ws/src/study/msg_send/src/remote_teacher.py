#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg
from std_msgs.msg import String

def call_back(data):
    st_name = data.last_name + ' ' + data.first_name
    print("Name : ", st_name)
	pub.publish(st_name)
    
rospy.init_node('remote_teacher', anonymous=True)

pub = rospy.Publisher('msg_from_xycar', String, queue_size=1)
sub = rospy.Subscriber('msg_to_xycar', my_msg, call_back)

rospy.spin() 

