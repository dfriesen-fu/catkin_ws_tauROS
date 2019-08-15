#/usr/bin/env python

import rospy
from autominy_msgs.msg import Speed

def callback(msg):
	print (msg)

rospy.init_node("sub_node")

rospy.Subscriber("/sensors/speed", Speed, callback)

rospy.spin()
