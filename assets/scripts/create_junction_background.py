#**************************************************************************************/
## Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
## This code is licensed (see LICENSE for details)

# Create the distance field image for the junction scenario. There are options to select 
# two-way junction, as well as the road/lane sizes etc.
#**************************************************************************************/
import numpy as np
import cv2
from create_distance_field import *

# Image will be 1000 x 1000. Obstacles are BLACK and background is WHITE
OBSTACLE_COLOR = (0,0,0)
h = 1000
w = 1000
img = 255*np.ones((h, w, 3), dtype = np.uint8)

# create a black image
world_size = 100
robot_radius = 10
lane_width = 4 * robot_radius
n_lanes = 2
twoway = True

if twoway:
    n_lanes *= 2
road_width = lane_width * n_lanes

# display the image using opencv
rectangles = [[(0,0), (w//2 - road_width//2,h//2 - road_width//2)],
              [(w//2 + road_width//2, 0), (w,h//2 - road_width//2)],
              [(0, h//2 + road_width//2), (w//2 - road_width//2, h)],
              [(w//2 + road_width//2, h//2 + road_width//2), (w,h)]]
for r in rectangles:
    cv2.rectangle(img, r[0], r[1], OBSTACLE_COLOR, -1)
# Create distance field
blur = create_distance_field(img)

# # save output
# cv2.imwrite('junction_twoway.png', 255 * blur)
