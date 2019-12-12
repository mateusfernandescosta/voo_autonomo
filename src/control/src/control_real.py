#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf

from geometry_msgs.msg 	import Twist, PoseStamped, Pose
from nav_msgs.msg 		import Odometry
from std_msgs.msg 		import Float32
from std_msgs.msg 		import Empty as Empty_msg

from std_srvs.srv 		import Empty, EmptyResponse

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Controller:

	def __init__(self):
		# Creates a new node
		rospy.init_node('Control_pose', anonymous=True)

		rospy.loginfo('CONTROL_POSE: Starting the tracking control node.')

		# Publisher
		self.cmd_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)		
		self.pub_car_pose = rospy.Publisher('/car_pose', PoseStamped, queue_size=10)
		self.pub_e_x = rospy.Publisher('/e_x', Float32, queue_size=10)
		self.pub_e_x_int = rospy.Publisher('/e_x_int', Float32, queue_size=10)
		self.takeoff = rospy.Publisher('/bebop/takeoff', Empty_msg, queue_size=10)
		self.camera_control = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
		
		# Subscriber
		self.sub_car_pose = rospy.Subscriber('/car_pose', PoseStamped , self.callback_pose)
		self.sub_odom = rospy.Subscriber('/bebop/odom', Odometry , self.callback_odom)
		# Services
		rospy.logwarn("waiting for service follow_car...")
		rospy.wait_for_service('/bebop/image_raw/theora/set_parameters')
		rospy.sleep(5)
		rospy.Service('follow_car', Empty, self.follow_car_service)
		rospy.Service('stop_follow_car', Empty, self.stop_follow_car_service)
		rospy.Service('mock_car_pose', Empty, self.mock_car_pose_service)
		rospy.Service('stop_mock_car_pose', Empty, self.stop_mock_car_pose_service)
		
		# Flags and Variables		
		self.car_pose = PoseStamped()	
		self.car_pose.pose.position.x = 0.0	

		self.start_time = rospy.Time.now().to_sec()
		self.last_time = self.start_time

		self.control_period = 0.1
		self.max_vel = 50
		self.e_x_int = 0
		self.e_y_int = 0

		self.count_int_x = 0
		self.count_int_y = 0

		self.e_x_last = 0
		self.e_y_last = 0

		self.odom = Odometry()

		self.follow_car = False

		self.mock_pose = False
		self.mock_style = ''

		self.Vx = 1.0
		self.Vy = 0	

		self.watchDog_counter = 0
		self.watchDog_nemMsg = False	
		self.watchDog_publish = False	

		# tf
		self.tf = tf.TransformListener()

		camera_ctl = Twist()
		camera_ctl.angular.y = -45
		self.camera_control.publish(camera_ctl)
		rospy.logwarn("Enviando comando para camera ficar a -45 graus!!!")

		self.takeoff.publish()
		rospy.logwarn("Enviando comando takeoff para o drone!!!")

		# Timed functions		
		rospy.Timer(rospy.Duration(self.control_period), self.poseWatchDog)
		rospy.Timer(rospy.Duration(self.control_period), self.controlLoop)		


	def follow_car_service(self, data):
		self.follow_car = True
		rospy.loginfo("FOLOW_CAR!!!")
		rospy.loginfo("FOLOW_CAR!!!")
		rospy.loginfo("FOLOW_CAR!!!")
		rospy.loginfo("FOLOW_CAR!!!")
		rospy.loginfo("FOLOW_CAR!!!")
		return {}

	def stop_follow_car_service(self, data):
		self.follow_car = False
		rospy.loginfo("STOP_FOLOW_CAR!!!")
		rospy.loginfo("STOP_FOLOW_CAR!!!")
		rospy.loginfo("STOP_FOLOW_CAR!!!")
		rospy.loginfo("STOP_FOLOW_CAR!!!")
		rospy.loginfo("STOP_FOLOW_CAR!!!")
		
		vel = Twist()		
		self.cmd_vel.publish(vel)
		
		return {}

	def mock_car_pose_service(self, data):
		rospy.loginfo("MOCK_CAR_SERVICE: The simulation will mock the car position.")
		self.mock_pose = True
		if self.mock_style in ['sinusoidal', 'straight']:
			rospy.loginfo("The service will mock a " + self.mock_style + ' trajectory.')
			rospy.Timer(rospy.Duration(self.control_period), self.mock_car_pose)
		else:
			rospy.loginfo("Publish the car_pose with enough rate")
		return {}

	def stop_mock_car_pose_service(self, data):
		rospy.loginfo("STOP_MOCK_CAR_SERVICE: Stop mocking the car pose.")
		self.mock_pose = False
		return {}


	def callback_pose(self, data):
		# rospy.loginfo("Callback_Pose")
		self.watchDog_nemMsg = True
		self.watchDog_counter = 0
		car_pose = data

		if self.mock_pose:
			# This simulates the drone realy seeing the car to the cameraless simulator
		# self_car_pose = data
			car_pose.pose.position.x = data.pose.position.x - self.odom.pose.pose.position.x
			car_pose.pose.position.y = data.pose.position.y - self.odom.pose.pose.position.y
		
    		# print "car_pose_old\n", self_car_pose.pose.position

			# source_frame = car_pose.header.frame_id
			# target_frame = 'base_link'

			# self.tf.waitForTransform(target_frame, source_frame, rospy.Time.now(), rospy.Duration(1.0))		
			# car_pose = self.tf.transformPose(target_frame, car_pose)

			# orientation_q = self.odom.pose.pose.orientation
			# orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
   #  		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

   #  		position_odom = self.odom.pose.pose.position

   #  		Rot_odom2bl = np.array([[ np.cos(yaw), -np.sin(yaw), 0],[np.sin(yaw), np.cos(yaw), 0],np.array([0,0,1])])
   #  		T_odom2bl = np.array([-position_odom.x, -position_odom.y, -position_odom.z]).reshape(3,1)
   #  		H_odom2bl = np.concatenate((Rot_odom2bl, T_odom2bl), 1)
   #  		H_odom2bl = np.concatenate((H_odom2bl, np.array([[0,0,0,1]])), 0)
   #  		print "\n H_odom2bl\n", H_odom2bl
   #  		# print "np.shape(H_odom2bl)", np.shape(H_odom2bl)
   #  		# print "\n position_odom", np.array([[position_odom.x, position_odom.y, position_odom.z, 1]]).reshape(4,1)
   #  		# print "np.shape(odom)", np.shape(np.array([[position_odom.x, position_odom.y, position_odom.z, 1]]).reshape(4,1))

   #  		position_bl = np.matmul(H_odom2bl,(np.array([[car_pose.pose.position.x, car_pose.pose.position.y, car_pose.pose.position.z, 1]]).reshape(4,1)))

   #  		print "\n position_bl\n", position_bl
   #  		# print "np.shape(position_bl)", np.shape(position_bl)

   #  		car_pose.pose.position.x = position_bl[0]
   #  		car_pose.pose.position.y = position_bl[1]
   #  		car_pose.pose.position.z = position_bl[2]
       		# print "car_pose_new\n", car_pose.pose.position


		# print car_pose

		self.car_pose = car_pose


	def mock_car_pose(self, event):		
		if self.mock_pose:

			now = rospy.Time.now()

			car_pose = PoseStamped()

			car_pose.header.frame_id = 'odom'
			car_pose.header.stamp = now
			t = now.to_sec() - self.start_time
			delta_t = now.to_sec() - self.last_time

			# Straight trajectory
			if self.mock_style=='straight':				
				if t < 10.0:
					car_pose.pose.position.x += self.Vx*delta_t
					car_pose.pose.position.y += self.Vy*delta_t
				else: 
					rospy.loginfo("Straight trajectory: changing direction")
					self.Vx = -self.Vx
					self.Vy = -self.Vy
					self.start_time = now.to_sec()

				print "car_pose.x", car_pose.pose.position.x
				print "car_pose.y", car_pose.pose.position.y

				self.pub_car_pose.publish(car_pose)

			# Sinusoidal trajectory
			if self.mock_style=='sinusoidal':
				Ax = 0.0
				Ay = 1.0
				wx = wy = 0.5
				phix = 0.0
				phiy = 0.0				
				car_pose.pose.position.x = Ax*np.cos(wx*t+phix)
				car_pose.pose.position.y = Ay*np.cos(wy*t+phiy)

				self.pub_car_pose.publish(car_pose)

			self.last_time = now.to_sec()
			

	def callback_odom(self, data):
		self.odom = data		

	def poseWatchDog(self, event):
		if not self.watchDog_nemMsg:
			self.watchDog_counter+=1			
		else:
			self.watchDog_publish = True	

		if self.watchDog_counter>=1:	# See if a higher limit is nedeed
			if self.follow_car: 
				rospy.logwarn("WATCHDOG: Car_pose rate slow (cmd_vel = 0)")
				self.watchDog_publish = False			
				vel = Twist()
				self.cmd_vel.publish(vel)

		self.watchDog_nemMsg = False

	def controlLoop(self, event):			
		if self.follow_car and self.watchDog_publish:
			# rospy.loginfo("car_pose:\n"+str(self.car_pose))

			car_pose = self.car_pose

			# Control gains
			Kp_x = 2.5 #0.15
			Kp_y = 3.0 #0.15
			Ti_x = 15.0
			Ti_y = 15.0
			Td_x = 3.0
			Td_y = 3.0

			# Considering that the msg car_pose comes in the base_link
			e_x = 0.0 - car_pose.pose.position.x
			e_y = -car_pose.pose.position.y

			dt = self.control_period

			ve_x = (e_x - self.e_x_last)/dt
			ve_y = (e_y - self.e_y_last)/dt


			# print "e_x", e_x
			# print "e_y", e_y

			if self.watchDog_publish: 
				self.e_x_int += e_x*dt
				self.e_y_int += e_y*dt

			# Anti WindUp
			if abs(self.e_x_int)>self.max_vel*Ti_x/Kp_x:
				# print "Anti-WindUp_x"
				self.e_x_int = self.max_vel*Ti_x/Kp_x*np.sign(self.e_x_int)
			if abs(self.e_y_int)>self.max_vel*Ti_y/Kp_y:
				# print "Anti-WindUp_y"
				self.e_y_int = self.max_vel*Ti_y/Kp_y*np.sign(self.e_y_int)

			# Anti OverShoot
			if (self.e_x_int*e_x < 0): self.count_int_x+=1
			else: self.count_int_x = 0
			if (self.e_y_int*e_y < 0): self.count_int_y+=1
			else: self.count_int_y = 0					

			if (self.count_int_x > 3):
				self.e_x_int = 0
				print "ZEROU!!!"
			if (self.count_int_y > 3):
				self.e_y_int = 0
				
												

			v_x = 0.1*self.odom.twist.twist.linear.x
			v_y = 0.1*self.odom.twist.twist.linear.y			

			vel = Twist()
			# PI with drones speed feedback
			# vel.linear.x = -Kp_x*(e_x + self.e_x_int/Ti_x) - Td_x*v_x
			# vel.linear.y = -Kp_y*(e_y + self.e_y_int/Ti_y) - Td_y*v_y

			# P with drones speed feedback
			# vel.linear.x = -Kp_x*(e_x) - Td_x*v_x
			# vel.linear.y = -Kp_y*(e_y) - Td_y*v_y

			vel.linear.x = (-Kp_x*(e_x) - Td_x*ve_x)/10.0
			vel.linear.y = (-Kp_y/5.0*(e_y) - Td_y/5.0*ve_y)/10.0
			vel.angular.z = (-Kp_y*(e_y) - Td_y*ve_y)/10.0

			# print "-----------------------------"
			# print "vel.linear.x", vel.linear.x
			# print "vel.linear.y", vel.linear.y
			# print "-----------------------------"


			# vel.linear.x = -Kp_x*(e_x + self.e_x_int/Ti_x) - Td_x*ve_x
			# vel.linear.y = -Kp_y*(e_y + self.e_y_int/Ti_y) - Td_y*ve_y


			# Saturating velocity for safety
			if abs(vel.linear.x)>self.max_vel:
				vel.linear.x = self.max_vel*np.sign(vel.linear.x)
			if abs(vel.linear.y)>self.max_vel:
				vel.linear.y = self.max_vel*np.sign(vel.linear.y)

			if self.watchDog_publish: 
				# print "vel_x", vel.linear.x
				# print "vel_y", vel.linear.y
				self.cmd_vel.publish(vel)
			else:
				vel.linear.x = 0
				vel.linear.y = 0
				self.cmd_vel.publish(vel)

			self.pub_e_x.publish(Kp_x*e_x)
			self.pub_e_x_int.publish(Kp_x*self.e_x_int/Ti_x)

			self.e_x_last = e_x
			self.e_y_last = e_y


			# ctrl_comm = PoseStamped()
			# #ctrl_comm.pose.position.x = data.position.x + 1.0
			# #ctrl_comm.pose.position.y = data.position.y
			# ctrl_comm.header = self.car_pose.header 
			# orientation_q = self.car_pose.pose.orientation
			# orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			# (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

			# ctrl_comm.pose.position.x = self.car_pose.pose.position.x - 1.0*math.cos(yaw)
			# ctrl_comm.pose.position.y = self.car_pose.pose.position.y - 1.0*math.sin(yaw)
			# ctrl_comm.pose.position.z = 0.0
			# ctrl_comm.pose.orientation.z = self.car_pose.pose.orientation.z
			# ctrl_comm.pose.orientation.w = self.car_pose.pose.orientation.w

			# try:
			# 	target_frame, source_frame = 'world', ctrl_comm.header.frame_id
			# 	self.tf.waitForTransform(target_frame, source_frame, ctrl_comm.header.stamp , rospy.Duration(1.0))
			# 	ctrl_comm_world = self.tf.transformPose(target_frame, ctrl_comm)
			# 	ctrl_comm_world.pose.position.z = 1
			# 	rospy.loginfo(ctrl_comm_world)
			# 	self.ctrl_pub.publish(ctrl_comm_world)
			# except Exception as e:
			# 	print e


if __name__ == '__main__':
	try:
		Controller()		   		
		rospy.spin()
	except rospy.ROSInterruptException: pass	