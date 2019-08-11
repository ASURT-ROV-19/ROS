#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time 
import sys
def talker():
        pub = rospy.Publisher('ray2', String, queue_size=10)
        rospy.init_node('pub_ray2')

        counter =0
        while not rospy.is_shutdown():
         try:
           counter = counter +1 
           print("Publishing ",counter)
           pub.publish(str(counter))
           time.sleep(1)
         except:
           print("Exception Msh Ray2:",sys.exc_info()[0])
talker()
