#!/usr/bin/env python

"""
    AV - Based on base_controller.py.  Provides connectivity between BuPiGo
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
from std_msgs.msg import UInt8
from bupigo_msgs.msg import Blobs
from tf.broadcaster import TransformBroadcaster
 
""" Class to receive Twist commands and publish Odometry data """
class BuPiGoController:
    def __init__(self, arduino):
        self.arduino = arduino
        self.stopped = False
        self.forwardSpeed = 0
        self.angularSpeed = 0
        
#        # Subscribe to bupigo_speeds
#        rospy.Subscriber("bupigo_wheel_speeds", WheelSpeeds, self.bupigoSetSpeedsCallback)

        # Subscribe to cmd_vel
        rospy.Subscriber("bupigo_cmd_vel", Twist, self.bupigoCmdVelCallback)

        # Subscribe to bupigo_servo_angle
        rospy.Subscriber("bupigo_servo_angle", UInt8, self.bupigoSetServoCallback)

        # Setup blob publisher
        self.blob_publisher = rospy.Publisher('bupigo_blobs', Blobs, queue_size=1)

        # Setup the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()
        
    def poll(self):
        (x, y, theta) = self.arduino.bupigo_read_odometry()

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
        
        odom.twist.twist.linear.x = self.forwardSpeed
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.angularSpeed

        self.arduino.bupigo_set_velocity(self.forwardSpeed, self.angularSpeed)

        self.odomPub.publish(odom)

        blobs = self.arduino.get_blobs()
        blobs.header.stamp = rospy.Time.now()
        self.blob_publisher.publish(blobs)

    def stop(self):
        self.stopped = True
        self.forwardSpeed = 0
        self.angularSpeed = 0
        self.arduino.bupigo_set_velocity(0, 0)
            
#    def bupigoSetSpeedsCallback(self, msg):
#        self.arduino.bupigo_set_speeds(msg.data)    

    def bupigoCmdVelCallback(self, cmd_vel):
        self.forwardSpeed = cmd_vel.linear.x         # m/s
        self.angularSpeed = cmd_vel.angular.z        # rad/s

    def bupigoSetServoCallback(self, msg):
        self.arduino.bupigo_set_servo(msg.data)    
