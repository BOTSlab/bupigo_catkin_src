#!/usr/bin/env python

"""
    AV - Based on base_controller.py.  Provides connectivity between RTbot
    related functions and ROS.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os
import math
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16
from tf.broadcaster import TransformBroadcaster
 
""" Class to receive Twist commands and publish Odometry data """
class RTbotController:
    def __init__(self, arduino):
        self.arduino = arduino
        self.stopped = False
        
        # Subscribe to rtbot_motors
        rospy.Subscriber("rtbot_motors", Int16, self.rtbotMotorsCallback)

        # Setup the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    def poll(self):
        (x, y, theta) = self.arduino.rtbot_read_odometry()

        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(theta / 2.0)
        quaternion.w = cos(theta / 2.0)
    
        # Create the odometry transform frame broadcaster.
        now = rospy.Time.now()
        self.odomBroadcaster.sendTransform(
            (x, y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            now,
            "base_link",
            "odom"
            )
    
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = now
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        
        # Possible issue: If we were commanding the robot via Twist messages
        # then we would have some sensible values to put here.  But by setting
        # the motor commands directly, we don't have access to plausible values
        # for the forward and angular speeds.  Does this even matter???
        #odom.twist.twist.linear.x = self.forwardSpeed
        odom.twist.twist.linear.x = 0
        odom.twist.twist.linear.y = 0
        #odom.twist.twist.angular.z = self.angularSpeed
        odom.twist.twist.angular.z = 0

        self.odomPub.publish(odom)

    def stop(self):
        self.stopped = True
        self.arduino.rtbot_set_motors(0)
            
    def rtbotMotorsCallback(self, msg):
        self.arduino.rtbot_set_motors(msg.data)    
