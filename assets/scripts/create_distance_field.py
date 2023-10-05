#**************************************************************************************/
## Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
## This code is licensed (see LICENSE for details)

# This function takes an image as an input (with obstacles in BLACK and background as WHITE)
# And returns an image which is blurred. The image can then be read by Simulator.cpp as a distance field,
# for use with static obstacle avoidance.

# This can also be run as python3 create_distance_field.py -i path_to_input_obstacle_file.png -o path_to_output.png
#**************************************************************************************/
import numpy as np
import cv2
from PIL import Image
from argparse import ArgumentParser
import os.path
from scipy.ndimage import gaussian_filter

''' Input: img of square dimensions.
    Output: Blurred image for use as a distance field 
    Obstacles should be BLACK, background is WHITE '''
def create_distance_field(img):
    if (img.shape[0]!=1000 or img.shape[1]!=1000):
        print("Resizing to 1000 x 1000")
        img = cv2.resize(img, (1000, 1000))

    # Convolve with a Gaussian to effect a blur.
    blur = gaussian_filter(img, sigma=5)
    blur = cv2.normalize(blur, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    
    distance_field = blur
    return distance_field

def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)
    else:
        return open(arg, 'r')  # return an open file handle



if __name__=="__main__":
    parser = ArgumentParser(description="Input and output files for createing distance field")
    parser.add_argument("-i", "--input", help="Input obstacle file to convert to distance field. Obstacles must be BLACK on WHITE background", default=None)
    parser.add_argument("-o", "--output", help="Output file name for distance field. Default = replaces input", default=None)
    args = parser.parse_args()
    input_file = args.input
    if ((input_file is None) or (not os.path.isfile(input_file))):
        print("Invalid file path given as input")
        exit()
    output_file = args.output
    if args.output is None:
        output_file = input_file


    # Load in input obstacle file 
    image = np.asarray(Image.open(input_file).convert('L'))
    
    # Create distance field with a blur
    blurred = create_distance_field(image)

    # Write to output
    cv2.imwrite(output_file, 255*blurred)