#usr/bin/env python

import rospy
import std_msgs.msg
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

rospy.init_node("pub_node")

publisher1 = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand, queue_size=10)
publisher2 = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
h = std_msgs.msg.Header()

while not rospy.is_shutdown():
	publisher1.publish(h,1.0)
	publisher2.publish(h,0.3)
	rospy.sleep(0.5)
