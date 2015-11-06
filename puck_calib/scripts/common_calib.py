#!/usr/bin/env python  

"""
Constants and common functionality for the calibration process.  The term
'target' here is generic and could mean an AprilTag fiducial or any other
marker.
"""

from math import floor
    
# The (xr, yr) position of the upper-left target in the calibration grid,
# measured in metres.
upper_left_x = 0.60 #couldn't get all rows in the picture
# 0.95 for 10x8, 1.13 for 8x10, 0.95 for black dots, 0.72 for blue dots
upper_left_y = 0.267 
# 0.45 for 10x8, 0.35 for 8x10, 0.40 for black dots, 0.267 for blue dots

# The distance between targets (assuming that the same distance exists between
# rows as between columns).
inter_target_distance = 0.09
#0.10 for 10 by 8, 0.20 for 5 by 4, 0.13 for black dots, 0.09 for blue dots 

# Width in targets.  In other words, the number of targets in a row
#7 for dots, 10, 8, 5, or 4 for tags
width_in_targets = 7

# Height in targets---the number of targets in a column
#7 for dots, 8, 10, 4, or 5 for tags
height_in_targets = 6 #couldn't get all blue dots on camera
def get_corresponding_point(index):
    """
    Return the (xr, yr) tuple associated with the target with the given index.
    The index values are assumed to start at 0 for the upper-left target and
    increase from left-to-right and top-to-bottom.

    Its very important to note that the robot reference frame has its x-axis
    aligned with the robot's forward's direction and its y-axis oriented 90
    degrees counter-clockwise to this.  Thus, rows in the calibration grid
    all have the same xr coordinates, but vary in their yr coordinates.
    """

    row = floor(index / width_in_targets)
    col = index - row * width_in_targets

    return (upper_left_x - row * inter_target_distance, \
            upper_left_y - col * inter_target_distance)
