from controller import Robot, Motor, Lidar, Supervisor, gps
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

# Set up Webots
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Get devices
lidar = robot.getDevice('lidar')
gps = robot.getDevice('gps')
lidar.enable(timestep)
gps.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 0.5

# Set up map parameters
map_resolution = 0.1
map_size = 20.0
map_grid_size = int(map_size / map_resolution)
coverage_threshold = 8

# Create occupancy grid map
map_image = np.zeros((map_grid_size, map_grid_size), dtype=np.uint8)

# Create a figure for the heat map
fig = plt.figure()

# Create a heat map plot
heatmap_plot = plt.imshow(map_image, cmap='hot', interpolation='nearest', vmin=0, vmax=255)


# Set up the plot axis
plt.axis('off')
plt.title('Occupancy Grid Map')

# Show the plot
plt.show(block=False)

# Define color codes
FREE_SPACE_COLOR = 0
OBSTACLE_COLOR = 255

# Define helper function to convert coordinates to grid indices
def coords_to_grid(coords):
    grid_x = int((coords[0] + map_size / 2) / map_resolution)
    grid_y = int((coords[1] + map_size / 2) / map_resolution)
    return grid_x, grid_y

# Define helper function to check if the entire environment has been explored
def is_map_explored(map_image, coverage_threshold):
    unexplored_pixels = np.count_nonzero(map_image == 0)
    total_pixels = map_image.size
    coverage = 1 - unexplored_pixels / total_pixels
    return coverage >= coverage_threshold

# Define helper function to compute grid indices of a circle centered at given coordinates
def circle_to_grid(coords, radius):
    grid_x, grid_y = coords_to_grid(coords)
    x, y = np.meshgrid(np.arange(-radius, radius + 1), np.arange(-radius, radius + 1))
    mask = (x**2 + y**2) <= radius**2
    circle_grid_x = grid_x + x[mask]
    circle_grid_y = grid_y + y[mask]
    return circle_grid_x, circle_grid_y

# Main loop
while robot.step(timestep) != -1:
    # Get LIDAR range data
    ranges = lidar.getRangeImage()
    # Get field of view of the LIDAR sensor
    fov = lidar.getFov()

    # Compute angle increment between adjacent range values
    angle_increment = fov / (lidar.getHorizontalResolution() - 1)

    # Convert LIDAR range data to (x,y) coordinates
    coords = []
    for i in range(lidar.getHorizontalResolution()):
        angle = -fov / 2 + angle_increment * i
        distance = ranges[i]
        if distance > 0:
            x = distance * math.sin(angle)
            y = distance * math.cos(angle)

            if (not math.isnan(x) and not math.isnan(y)):
                if str(x) == "inf" or str(y) == "inf" or str(x) == "-inf" or str(y) == "-inf":
                    continue
                coords.append((x, y))

    # Update occupancy grid map
    for coord in coords:
        grid_x, grid_y = coords_to_grid(coord)
        if 0 <= grid_x < map_grid_size and 0 <= grid_y < map_grid_size:
            if ranges[i] < OBSTACLE_THRESHOLD:
                # Set obstacle pixel to OBSTACLE_COLOR
                map_image[grid_y, grid_x] = OBSTACLE_COLOR
            else:
                # Set free space pixel to FREE_SPACE_COLOR
                map_image[grid_y, grid_x] = FREE_SPACE_COLOR


    # Compute a circle around the robot's current position and mark all the grid cells within that circle as explored in the occupancy grid map
    circle_grid_x, circle_grid_y = circle_to_grid(gps.getValues(), int(0.5 / map_resolution))
    for x, y in zip(circle_grid_x, circle_grid_y):
        if 0 <= x < map_grid_size and 0 <= y < map_grid_size:
            map_image[y, x] = FREE_SPACE_COLOR


    # Check if the entire environment has been explored
    if is_map_explored(map_image, coverage_threshold):
        break

    # Perform obstacle avoidance and update motor speeds
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED


    # Obstacle avoidance behavior
    # Check for obstacles and determine their location and size
    obstacle_detected = False
    obstacle_start = 0
    obstacle_end = 0
    for i in range(lidar.getHorizontalResolution()):
        if ranges[i] < OBSTACLE_THRESHOLD:
            if not obstacle_detected:
                obstacle_detected = True
                obstacle_start = i
            obstacle_end = i
    
    if obstacle_detected:
        obstacle_size = obstacle_end - obstacle_start + 1
        vacant_size = lidar.getHorizontalResolution() - obstacle_size
        if obstacle_start < obstacle_end:
            # Obstacle is on the left side
            left_speed -= MAX_SPEED * (obstacle_size / vacant_size)
        else:
            # Obstacle is on the right side
            right_speed -= MAX_SPEED * (obstacle_size / vacant_size)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    # Update the occupancy grid map and display the live heat map
    heatmap_plot.set_data(map_image)
    plt.draw()
    plt.pause(0.001)

    # Display occupancy grid map
    cv2.imshow('Occupancy Grid Map', map_image)
    cv2.waitKey(1)

# Stop the robot
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Save the occupancy grid map to a file
cv2.imwrite('occupancy_grid_map.png', map_image)

