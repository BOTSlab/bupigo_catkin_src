#!/usr/bin/env python

"""
Demo which simply steers the robot towards pucks.
"""
import rospy
from geometry_msgs.msg import Twist
from bupigo_msgs.msg import Blobs

def blobsCallback(blobs):
    twist = Twist()

    if blobs.blob_count > 0:

        # Just get the x position of the first blob
        x = blobs.blobs[0].x

        # Go forward and left or right (depending on x):
        twist.linear.x = 0.5
        twist.angular.z = 0.10 * (160 - x)

    pub.publish(twist)

if __name__ == '__main__':
    global pub
    rospy.init_node('test')
    pub = rospy.Publisher('/bupigo_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/bupigo_blobs', Blobs, blobsCallback)
    rospy.spin()
