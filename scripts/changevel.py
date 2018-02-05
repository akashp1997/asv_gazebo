#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs.msg
import gazebo_msgs.msg

cmd_vel = geometry_msgs.msg.Twist()

def listener():
	rospy.init_node("vel_gazebo_talker")
	rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, callback)
	rospy.spin()

def callback(data):
	global cmd_vel
	cmd_vel = data
	rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback_1)

def callback_1(data):
	global cmd_vel
	msg = gazebo_msgs.msg.ModelState()
	msg.model_name = data.name[0]
	msg.pose = data.pose[0]
	msg.twist = cmd_vel
	rospy.loginfo(msg)
	pub = rospy.Publisher("/gazebo/set_model_state", gazebo_msgs.msg.ModelState, queue_size=10)
	pub.publish(msg)

listener()