#!/usr/bin/env python

"""
Just opens and closes the gripper.
"""

from std_msgs.msg import UInt8
import rospy

if __name__ == '__main__':
    global pub
    rospy.init_node('test_gripper')
    pub = rospy.Publisher('/bupigo_servo_angle', UInt8, queue_size=1)


    #
    # SCOTT PLAY WITH THESE VALUES TO MAKE GRIPPER OPEN AND CLOSE.
    #

    pub.publish(0)
    rospy.sleep(5)
    pub.publish(180)
