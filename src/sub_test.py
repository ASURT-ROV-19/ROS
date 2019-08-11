#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

def callback(data):
    print(data)

def listener():

    rospy.init_node('sub_ray2')
    rospy.Subscriber('ray2',String,callback)
    rospy.spin()

listener()
