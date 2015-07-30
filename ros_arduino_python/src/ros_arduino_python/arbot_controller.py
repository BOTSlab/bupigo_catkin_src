#!/usr/bin/env python

"""
    AV - Based on base_controller.py.  Provides connectivity between ARbot
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
#from arbot_control.srv import *
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.broadcaster import TransformBroadcaster
#from cmvision.msg import Blobs, Blob
 
""" Class to receive Twist commands and publish Odometry data """
class ARbotController:
    def __init__(self, arduino):
        self.arduino = arduino
        self.stopped = False
        
        self.cmd = 1

        self.forwardSpeed = 0
        self.angularSpeed = 0
                 
        # Subscribe to cmd_vel
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("cmd_vel2", Twist, self.cmdVel2Callback)
        s = rospy.Service("pick_cmd_vel", PickCmdVel, self.pickCmdVel)

        # Subscribe to arbot_play_led and arbot_advance_led
        rospy.Subscriber("arbot_play_led", Bool, self.arbotPlayLedCallback)
        rospy.Subscriber("arbot_advance_led", Bool, self.arbotAdvanceLedCallback)
        
        # Setup the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
        # Setup blob publisher
#        self.blob_publisher = rospy.Publisher('blobs', Blobs, queue_size=10)
        rospy.loginfo("Started ARbot controller.")
        
    def poll(self):
        (x, y, theta) = self.arduino.arbot_read_odometry()

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

        self.arduino.arbot_set_velocity(self.forwardSpeed, self.angularSpeed)

        self.odomPub.publish(odom)

#        blobs = self.arduino.get_blobs()
#        blobs.header.stamp = rospy.Time.now()
#        self.blob_publisher.publish(blobs)
            
    def stop(self):
        self.stopped = True
        self.forwardSpeed = 0
        self.angularSpeed = 0
        self.arduino.arbot_set_velocity(0, 0)
            
    def cmdVelCallback(self, cmd_vel):
        if self.cmd == 2:
            return
        self.forwardSpeed = cmd_vel.linear.x         # m/s
        self.angularSpeed = cmd_vel.angular.z        # rad/s

    def cmdVel2Callback(self, cmd_vel2):
        if self.cmd == 1:
            return
        self.forwardSpeed = cmd_vel2.linear.x
        self.angularSpeed = cmd_vel2.angular.z

    def pickCmdVel(self, req):
        self.cmd = req.cmd_vel
        #print "got cmd_vel: {0}".format(req.cmd_vel)
        return PickCmdVelResponse(req.cmd_vel)
        
    def arbotPlayLedCallback(self, msg):
        self.arduino.arbot_set_play_led(mgs.data)    
    def arbotAdvanceLedCallback(self, msg):
        self.arduino.arbot_set_advance_led(msg.data)
