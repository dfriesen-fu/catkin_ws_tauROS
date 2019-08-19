#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
# from autominy_msgs.msg import Speed

def callback(msg):
    print(msg)


rospy.init_node("sub_node")
rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, callback)
rospy.spin()
