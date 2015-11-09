#!/usr/bin/env python
"""
    This node subscribes to /blobs and publishes /pucks and /vis_pucks (more
suitable for rviz).  A Blob has a type and image coordinates, whereas a Puck
also has coordinates in the robot's reference frame from the calibration
process.
"""
import cv
import csv
import rospkg
import rospy
import roslib
import math
roslib.load_manifest('blobs_to_pucks')
from bupigo_msgs.msg import Blobs, Blob, Puck, PuckArray
from sensor_msgs.msg import PointCloud, ChannelFloat32

def callback(blobs_msg):
    puck_array = PuckArray()
    
    vis_pucks = PointCloud()
    vis_pucks.header.frame_id = '/base_link'
    channel = ChannelFloat32()
    channel.name = "intensity"

    for blob in blobs_msg.blobs:
        try: 
            puck = Puck()
            puck.xi = blob.x
            puck.yi = blob.y
            puck.type = blob.type
            (puck.position.x, puck.position.y) = corresDict[puck.xi, puck.yi]
            puck_array.pucks.append(puck)
        
            vis_pucks.points.append(puck.position)
            channel.values.append(0)
        except:
            continue 

    puck_publisher.publish(puck_array)
    vis_pucks.channels.append(channel)
    vis_pucks_publisher.publish(vis_pucks)

def readCorrespondences(filename):
    f = open(filename, 'rt')
    x = []
    y = []
    Xr = []
    Yr = []
    try:
        reader = csv.reader(f, delimiter=',')
        # Skip the first (header) row
        next(reader)
        for row in reader:
            x.append(int(row[0]))
            y.append(int(row[1]))
            Xr.append(float(row[2]))
            Yr.append(float(row[3]))
        return [x, y, Xr, Yr]
    finally:
        f.close()

if __name__ == '__main__':
    global puck_publisher, vis_pucks_publisher, corresDict

    rospy.init_node('blobs_to_pucks')
    
    calib_dir = rospkg.RosPack().get_path('puck_calib')
    [Xi, Yi, Xr, Yr] = readCorrespondences(\
            calib_dir + '/scripts/interpolated_correspondences.csv')

    corresDict = {}
    for i in range(len(Xi)):
        corresDict[(Xi[i], Yi[i])] = (Xr[i], Yr[i])

    puck_publisher = rospy.Publisher('/pucks', PuckArray)

    vis_pucks_publisher = rospy.Publisher('/vis_pucks', PointCloud)

    rospy.Subscriber('/blobs', Blobs, callback)
    
    rospy.spin()
