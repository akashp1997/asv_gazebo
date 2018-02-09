#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

msg = geometry_msgs.msg.TwistStamped()

def listener():
	rospy.init_node("gps_cal")
	rospy.Subscriber("/vel_vector3", geometry_msgs.msg.Vector3Stamped, callback)
	talker()
	rospy.spin()

def callback(data):
	global msg
	msg.header = data.header
	msg.header.frame_id="garmin"
	msg.twist.linear = data.vector
	
def talker():
	global msg
	pub = rospy.Publisher("/vel", geometry_msgs.msg.TwistStamped, queue_size=10)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()

listener()

