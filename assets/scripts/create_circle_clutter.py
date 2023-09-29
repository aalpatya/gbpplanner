#**************************************************************************************/
## Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
## This code is licensed under MIT license (see LICENSE for details)

# Create obstacles for the cluttered scene, consisting of triangles and squares
# Create the distance field image
#**************************************************************************************/
# Needs to blur an input image
import numpy as np
import cv2
from create_distance_field import *

# Image will be 1000 x 1000. Obstacles are BLACK and background is WHITE
OBSTACLE_COLOR = (0,0,0)
img = 255*np.ones((1000, 1000, 3), dtype = np.uint8)

# Create Obstacle shapes
triangles = [[(380, 410), (355, 423), (410, 454)],
             [(527, 492), (527, 540), (602, 510)]]
for t in triangles:
    cv2.fillPoly(img, np.array([t]), OBSTACLE_COLOR)

rectangles = [[(573, 366), (603, 426)],
             [(420, 550), (460, 590)],
             [(600,575), (650,625)],
             [(468, 412), (498, 442)]]
for r in rectangles:
    cv2.rectangle(img, r[0], r[1], OBSTACLE_COLOR, -1)

# Create distance field
blur = create_distance_field(img)

# # save output
# cv2.imwrite('circle_cluttered.png', 255 * blur)
