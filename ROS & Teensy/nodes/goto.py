#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

import argparse
import roslib
import rospy
import math

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

goals = { "Nick's Office": [-1.271, 5.108, 0.000, 1.512],
          "Lynn's Office": [-4.112, 5.416, 0.000, 1.385],
          "Tomek's Office": [-5.792, 5.701, 0.000, 3.049],	
          "Mike's Desk": [0.668, 2.886, 0.000, -1.136],
          "Ken's Desk": [3.148, 2.630, 0.000, -0.959],
          "Samira's Office": [4.705, 4.592, 0.000, 1.350],
          "The Lab": [3.027, -0.966, 0.000, 0.011],
          "The Hallway": [-0.785, -1.890, 0.000, -1.601] }

class maxwell_goto:

    def __init__(self):
        self.msg = PoseStamped()
        self.msg.header.frame_id = "/map"

        self.pub_ = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        rospy.spin()

    def setMsg(self, goal):
        self.msg.pose.position.x = goals[goal][0]
        self.msg.pose.position.y = goals[goal][1]
        self.msg.pose.position.z = goals[goal][2]
        q = quaternion_from_euler(0, 0, goals[goal][3], 'sxyz')
        self.msg.pose.orientation.x = q[0]
        self.msg.pose.orientation.y = q[1]
        self.msg.pose.orientation.z = q[2]
        self.msg.pose.orientation.w = q[3]
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        self.msg.header.stamp = rospy.Time.now()

        if msg.data.find("lab") > -1: 
            self.setMsg("The Lab")
        elif msg.data.find("hallway") > -1:
            self.setMsg("The Hallway")
        elif msg.data.find("ken") > -1:
            self.setMsg("Ken's Desk")
        elif msg.data.find("nick") > -1:
            self.setMsg("Nick's Office")
        else:
            rospy.loginfo("unknown goal!")
            return

        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('lindorobot_goto')
    try:
        lindorobot_goto()
    except:
        pass
