import numpy as np
import cv2
import math

#Constants
two_pi = math.degrees(math.pi+math.pi)
half_a_degree = 0.5
angle_lower_bound = two_pi - half_a_degree
angle_upper_bound = half_a_degree

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), invert_filter=False):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if invert_filter == False:
        above_thresh = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,1] > rgb_thresh[1]) & (img[:,:,2] > rgb_thresh[2])
    else:
        above_thresh = (img[:,:,0] < rgb_thresh[0]) & (img[:,:,1] < rgb_thresh[1]) & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def yellow_thresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV,3) # convert to HSV space from RGB    
    # set upper and lower bounds for threshold and mask it
    low_yellow = np.array([20, 100, 100], dtype = "uint8")
    high_yellow = np.array([25, 255, 255], dtype = "uint8")
    mask = cv2.inRange(hsv, low_yellow, high_yellow)
    return mask

def find_rocks(img, levels=(110, 110, 50)):
    rockpix = ((img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1]) & (img[:,:,2] < levels[2]))
    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image    
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask

# Validate the pitch and roll for accurate mapping (it increases fidelity)
def is_valid_position(pitch, roll):
    # Your perspective transform is technically only valid when roll and pitch angles are near zero
    return ((pitch > angle_lower_bound or pitch < angle_upper_bound) and (roll > angle_lower_bound or roll < angle_upper_bound))

# Check if a Rock is detected
def is_rock_ahead(rock_thresh):
    return rock_thresh.any()
	
# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    img = Rover.img	
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain_thresh = color_thresh(warped)
    obstacle_thresh = color_thresh(warped, invert_filter=True)
    #obstacle_thresh = np.absolute(np.float32(terrain_thresh) - 1) * mask # alternative way
    rock_thresh = yellow_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_thresh * 255
    #Rover.vision_image[:,:,1] = rock_thresh
    Rover.vision_image[:,:,2] = terrain_thresh * 255
    # 5) Convert map image pixel values to rover-centric coords
    xpix_terrain, ypix_terrain = rover_coords(terrain_thresh)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacle_thresh)
    xpix_rock, ypix_rock = rover_coords(rock_thresh) 
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    navigable_x_world, navigable_y_world = pix_to_world(xpix_terrain, ypix_terrain, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if is_valid_position(Rover.pitch, Rover.roll):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1#10
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    if is_rock_ahead(rock_thresh):
        dist, angles = to_polar_coords(xpix_rock, ypix_rock)
        Rover.sample_detected = True
        Rover.rock_spot_count = 0 # The rover will reset the rock spot counter
        rock_idx = np.argmin(dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        Rover.worldmap[rock_ycen, rock_xcen, 1] += 1
        Rover.vision_image[:, :, 1] = rock_thresh * 255
        #print("Rock found at ", rock_xcen, rock_ycen)
    else:
        dist, angles = to_polar_coords(xpix_terrain, ypix_terrain)
        #print("No rock")

    Rover.nav_angles = angles
    Rover.nav_dists = dist

    return Rover