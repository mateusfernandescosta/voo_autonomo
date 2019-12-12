#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#import cv2
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse


class Controller:

	def __init__(self):
		# Cria um novo nó
		rospy.init_node('Control_pose', anonymous=True)
		#self.odom = rospy.Subscriber('/bebop2/odometry_sensor1/pose', Pose , self.callback_control)
		self.odom = rospy.Subscriber('/car_pose', PoseStamped , self.callback_control)
		self.ctrl_pub = rospy.Publisher('/bebop2/command/pose', PoseStamped, queue_size=10)

		rospy.Service('follow_car', Empty, self.follow_car_service)
		rospy.Service('stop_follow_car', Empty, self.stop_follow_car_service)
		
		self.follow_car = False

		self.tf = tf.TransformListener()

	def follow_car_service(self, data):
		self.follow_car = True

	def stop_follow_car_service(self, data):
		self.follow_car = False

	def callback_control(self, data):
		if self.follow_car:
			rospy.loginfo(data)
			ctrl_comm = PoseStamped()
			#ctrl_comm.pose.position.x = data.position.x + 1.0
			#ctrl_comm.pose.position.y = data.position.y
			ctrl_comm.header = data.header 
			orientation_q = data.pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

			ctrl_comm.pose.position.x = data.pose.position.x - 1.0*math.cos(yaw)
			ctrl_comm.pose.position.y = data.pose.position.y - 1.0*math.sin(yaw)
			ctrl_comm.pose.position.z = 0.0
			ctrl_comm.pose.orientation.z = data.pose.orientation.z
			ctrl_comm.pose.orientation.w = data.pose.orientation.w

			try:
				target_frame, source_frame = 'world', ctrl_comm.header.frame_id
				self.tf.waitForTransform(target_frame, source_frame, ctrl_comm.header.stamp , rospy.Duration(1.0))
				ctrl_comm_world = self.tf.transformPose(target_frame, ctrl_comm)
				ctrl_comm_world.pose.position.z = 1
				rospy.loginfo(ctrl_comm_world)
				self.ctrl_pub.publish(ctrl_comm_world)
			except Exception as e:
				print e

			


if __name__ == '__main__':
	try:
		Controller()		   		
		rospy.spin()
	except rospy.ROSInterruptException: pass	