#!/usr/bin/python

import rospy as rp

from geometry_msgs.msg import PoseStamped

class OptiToPX4Node():

    def __init__(self,):
        rp.init_node("opti_to_px4")

        # Set up Subscriber
        self.vrpn_sub = rp.Subscriber("vrpn/Magdrone/pose", PoseStamped, self.vrpn_callback, queue_size = 1)

        # Set up Publisher
        self.px4_pub = rp.Publisher("px4/Magdrone/pose", PoseStamped, queue_size = 1)

        rp.spin()

    def vrpn_callback(self, msg):
        px4_pose = PoseStamped

        px4_pose.header = msg.header
        px4_pose.pose.position.x =  msg.pose.position.x
        px4_pose.pose.position.y =  msg.pose.position.z
        px4_pose.pose.position.z = -msg.pose.position.y

        px4_pose.pose.orientation.w =  msg.pose.orientation.w
        px4_pose.pose.orientation.x =  msg.pose.orientation.x
        px4_pose.pose.orientation.y =  msg.pose.orientation.z
        px4_pose.pose.orientation.z = -msg.pose.orientation.y

        px4_pub.publish(px4_pose)

# Start Node
filtered = FilterNode()
