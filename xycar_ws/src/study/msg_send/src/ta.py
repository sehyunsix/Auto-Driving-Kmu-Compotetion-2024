#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

value = None

def func(msg):  
    global value
    value = msg.data

rospy.init_node('ta') 
sub = rospy.Subscriber('my_topic1',Int32, func)
pub = rospy.Publisher('my_topic2', Int32) 

while not rospy.is_shutdown():
    
    if(value == None):
        continue
    pub.publish(value*100)
    value = None
