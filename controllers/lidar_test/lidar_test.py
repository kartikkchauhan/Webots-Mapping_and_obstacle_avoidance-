from controller import Robot, Motor, Lidar, Supervisor
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice('lidar')
lidar.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 0.5

# Define the occupancy grid map parameters
map_resolution = 0.1
map_size = 10.0
map_width = int(map_size / map_resolution)
map_height = int(map_size / map_resolution)
occupancy_grid = np.zeros((map_height, map_width), dtype=np.uint8)

# Define the robot path list
robot_path = []

# Define the map visualization parameters
map_scale = 10
map_color = (0, 0, 255)
map_thickness = -1

# Get the robot's GPS sensor
gps = robot.getDevice('gps')
gps.enable(timestep)

while robot.step(timestep) != -1:
    ranges = lidar.getRangeImage()
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # Check for obstacles and update the occupancy grid map
    for i in range(lidar.getHorizontalResolution()):
        if ranges[i] < OBSTACLE_THRESHOLD:
            x = int((ranges[i] * np.sin(i / 180.0 * np.pi) + map_size / 2) / map_resolution)
            y = int((-ranges[i] * np.cos(i / 180.0 * np.pi) + map_size / 2) / map_resolution)
            if x >= 0 and x < map_width and y >= 0 and y < map_height:
                occupancy_grid[y, x] = 255

    # Record the robot's position from GPS sensor and add it to the path list
    robot_position = gps.getValues()
    robot_x = int((robot_position[0] + map_size / 2) / map_resolution)
    robot_y = int((-robot_position[2] + map_size / 2) / map_resolution)
    robot_path.append((robot_x, robot_y))

    # Update the map visualization
    map_image = cv2.resize(occupancy_grid, (map_width * map_scale, map_height * map_scale), interpolation=cv2.INTER_NEAREST)
    map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
    for i in range(lidar.getHorizontalResolution()):
        if ranges[i] < OBSTACLE_THRESHOLD:
            x = int((ranges[i] * np.sin(i / 180.0 * np.pi) + map_size / 2) / map_resolution)
            y = int((-ranges[i] * np.cos(i / 180.0 * np.pi) + map_size / 2) / map_resolution)
            if x >= 0 and x < map_width and y >= 0 and y < map_height:
                cv2.circle(map_image, (x * map_scale, y * map_scale), map_scale // 2, map_color, thickness=map_thickness)

    # Draw the robot path on the map image
    if len(robot_path) > 1:
        for i in range(len(robot_path) - 1):
            cv2.line(map_image, robot_path[i], robot_path[i+1], (0, 0, 0), thickness=2)

    cv2.imshow('Occupancy Grid Map', map_image)
    cv2.waitKey(1)

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

    # If an obstacle is detected, adjust the left and right speeds of the wheels
    if obstacle_detected:
        obstacle_size = obstacle_end - obstacle_start + 1
        vacant_size = lidar.getHorizontalResolution() - obstacle_size
        if obstacle_start < obstacle_end:
            # Obstacle is on the left side
            left_speed -= MAX_SPEED * (obstacle_size / vacant_size)
        else:
            # Obstacle is on the right side
            right_speed -= MAX_SPEED * (obstacle_size / vacant_size)

    # Update the motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

