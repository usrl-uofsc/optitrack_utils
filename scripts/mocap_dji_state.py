#!/usr/bin/python

import numpy as np
import rospy as rp

import threading

from tf.transformations import *

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped
from dic_uav.msg import ObservedState

class MocapNode():

	def __init__(self):
		rp.init_node("state_pub")
		
		# Get publisher topic
		self.pub_topic = rp.get_param("state_topic")
		
		# Set rate of publisher
		self.rate = 50.0
		
		# System state
		self.isInit = False
		self.timeBefore = None
		self.state = ObservedState()
		
		# Set up Subscribers
		self.mocap_pose_sub = rp.Subscriber("/opti_state/pose", PoseStamped, self.mocap_pose_callback, queue_size = 1)
		self.mocap_rate_sub = rp.Subscriber("/opti_state/rates", TwistStamped, self.mocap_rates_callback, queue_size = 1)
		self.drone_atti_sub = rp.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.atti_callback, queue_size = 1)
		self.drone_imu_sub  = rp.Subscriber("/dji_sdk/imu", Imu, self.imu_callback, queue_size = 1)
		
		# Set up Publisher
		self.state_pub = rp.Publisher(self.pub_topic, ObservedState, queue_size = 1)
		
		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()

		rp.spin()
	
	def mocap_pose_callback(self, msg):
		# Update position
		self.state.drone_p.x = msg.pose.position.x
		self.state.drone_p.y = msg.pose.position.y
		self.state.drone_p.z = msg.pose.position.z
		
		# Update drone heading
		(r, p, y) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		self.state.drone_q.z = y
		
		self.isInit = True
	
	def mocap_rates_callback(self, msg):
		# Update drone velocity
		self.state.drone_pdot.x = msg.twist.linear.x
		self.state.drone_pdot.y = msg.twist.linear.y
		self.state.drone_pdot.z = msg.twist.linear.z

	def atti_callback(self, msg):
		# Update drone roll and pitch
		(r, p, y) = euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
		self.state.drone_q.x = r
		self.state.drone_q.y = p

	def imu_callback(self, msg):
		# Update angular rates
		self.state.drone_qdot.x = msg.angular_velocity.x
		self.state.drone_qdot.y = msg.angular_velocity.y
		self.state.drone_qdot.z = msg.angular_velocity.z

	def statePublisher(self):
		r = rp.Rate(self.rate)
		while not rp.is_shutdown():
			if (self.isInit):
				self.state_pub.publish(self.state)
			
			r.sleep()

m = MocapNode()
