# Webots Robot Mapping and Obstacle Avoidance

## Overview

This project involves creating a robot in Webots that maps and navigates through an environment while avoiding obstacles in real-time. The robot uses LIDAR range data and GPS coordinates to update an occupancy grid map, and obstacle avoidance behavior is performed to update the motor speeds.

The project has several real-world applications, including improving safety and efficiency in autonomous vehicles, search and rescue operations, and warehouse automation. By accurately mapping the environment and avoiding obstacles, the robot can navigate through complex environments, reducing accidents and increasing productivity.

## Getting Started

To get started with this project, you will need to download and install [Webots](https://www.cyberbotics.com/#download), an open-source robotics simulator. 

Once you have installed Webots, you can clone this repository and open the `worlds/robot_mapping.wbt` file in Webots to start the simulation.

## How it Works

The robot uses LIDAR range data and GPS coordinates to update an occupancy grid map. Obstacle avoidance behavior is performed to update the motor speeds. The occupancy grid map provides a visual representation of the environment, including obstacles and free space, and the robot's path.

The mapping and obstacle avoidance techniques used in this project can be applied to various industries, providing an opportunity for individuals to gain hands-on experience in robotics, mapping, and obstacle avoidance.

## Repository Contents

- `worlds/robot_mapping.wbt`: Webots world file for the robot mapping and obstacle avoidance simulation
- `controllers/robot_mapping/robot_mapping.py`: Python controller for the robot, responsible for mapping and obstacle avoidance
- `README.md`: This README file

## Contributing

Contributions to this project are welcome. If you find a bug or have an idea for an improvement, feel free to open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
