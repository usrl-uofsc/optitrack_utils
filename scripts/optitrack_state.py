#!/usr/bin/python

"""
MIT License

Copyright(c) 2019 Michail Kalaitzakis (Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Michail Kalaitzakis

"""
import sys

import numpy as np
import rospy as rp

from geometry_msgs.msg import PoseStamped, TwistStamped

from kalman_filter import KalmanFilter

from quaternion import Quaternion
from quaternion import quat2rpy
from quaternion import rpy2quat

import threading

class FilterNode():

	def __init__(self,):
		rp.init_node("filter")

		# Get rate of state publisher
		self.rate = rp.get_param("rate", 100)
		
		# Get VRPN client topic
		self.vrpn_topic = rp.get_param("vrpn_topic")

		# Number of states
		self.n = 12

		# System state
		self.X = np.matrix(np.zeros((self.n, 1)))

		# Initial State Transition Matrix
		self.F = np.asmatrix(np.eye(self.n))

		# Initial Process Matrix
		self.P = np.asmatrix(1.0e3 * np.eye(self.n))

		# Process Noise Level
		self.N = 1.0e-3

		# Initialize H and R matrices for optitrack pose
		self.Hopti = np.matrix(np.zeros((6, self.n)))
		self.Hopti[0:3, 0:3] = np.matrix(np.eye(3))
		self.Hopti[3:6, 6:9] = np.matrix(np.eye(3))

		self.Ropti = np.matrix(np.zeros((6, 6)))
		self.Ropti[0:3, 0:3] = 1.0e-3 * np.matrix(np.eye(3))
		self.Ropti[3:6, 3:6] = 1.0e-5 * np.matrix(np.eye(3))

		# Initialize Kalman Filter
		self.kalmanF = KalmanFilter()
		self.isInit = False
		self.lastTime = None

		# Set up Subscriber
		self.vrpn_sub = rp.Subscriber(self.vrpn_topic, PoseStamped, self.state_callback, queue_size = 1)

		# Set up Publisher
		self.state_pub = rp.Publisher("opti_state/pose", PoseStamped, queue_size = 1)
		self.state_rate_pub = rp.Publisher("opti_state/rates", TwistStamped, queue_size = 1)

		# Create thread for publisher
		t = threading.Thread(target=self.statePublisher)
		t.start()

		rp.spin()
	
	def state_callback(self, msg):
		attiQ = Quaternion(msg.pose.orientation.x,
						   msg.pose.orientation.y,
						   msg.pose.orientation.z,
						   msg.pose.orientation.w)
		
		rpy = quat2rpy(attiQ)

		pose = np.matrix([[msg.pose.position.x],
						  [msg.pose.position.y],
						  [msg.pose.position.z],
						  [rpy[0]],
						  [rpy[1]],
						  [rpy[2]]])

		if self.isInit:
			timeNow = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
			dt = timeNow - self.lastTime

			# Prediction
			self.kalmanF.updateF(dt)
			self.kalmanF.updateQ()
			self.kalmanF.predict()

			# Correction
			self.kalmanF.correct(pose, self.Hopti, self.Ropti)

			# Update state
			self.X = self.kalmanF.getState()

			self.lastTime = timeNow

		else:
			# Initialize state
			self.X[0] = pose[0]
			self.X[1] = pose[1]
			self.X[2] = pose[2]
			self.X[6] = pose[3]
			self.X[7] = pose[4]
			self.X[8] = pose[5]

			# Initialize filter
			self.kalmanF.initialize(self.X, self.F, self.P, self.N)
			self.lastTime = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
			self.isInit = True
	
	def statePublisher(self):
		r = rp.Rate(self.rate)

		while not rp.is_shutdown():
			if self.isInit:
				timeNow = rp.Time.now()
				stateMsg = PoseStamped()
				stateMsg.header.stamp = timeNow
				stateMsg.header.frame_id = 'optitrack'
			
				stateMsg.pose.position.x = self.X.item(0)
				stateMsg.pose.position.y = self.X.item(1)
				stateMsg.pose.position.z = self.X.item(2)

				q = rpy2quat(self.X.item(6), self.X.item(7), self.X.item(8))

				stateMsg.pose.orientation.x = q.x
				stateMsg.pose.orientation.y = q.y
				stateMsg.pose.orientation.z = q.z
				stateMsg.pose.orientation.w = q.w

				twistMsg = TwistStamped()
				twistMsg.header.stamp = timeNow
				twistMsg.header.frame_id = 'optitrack'

				twistMsg.twist.linear.x = self.X.item(3)
				twistMsg.twist.linear.y = self.X.item(4)
				twistMsg.twist.linear.z = self.X.item(5)

				twistMsg.twist.angular.x = self.X.item(9)
				twistMsg.twist.angular.y = self.X.item(10)
				twistMsg.twist.angular.z = self.X.item(11)

				self.state_pub.publish(stateMsg)
				self.state_rate_pub.publish(twistMsg)
			
			r.sleep()

# Start Node
filtered = FilterNode()
