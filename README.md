# Autonomous Sanitization Robot

## Project Overview
This project focuses on developing an autonomous sanitizing robot designed to automate the disinfection of public spaces or confined, unknown environments. The robot leverages ROS2 for advanced robotic capabilities and features UV lamp-based disinfection.

### Key Features
- **Simultaneous Localization and Mapping (SLAM):** Real-time map generation and localization within the environment.
- **Autonomous Navigation:** Goal-oriented navigation and obstacle avoidance using the Nav2 stack.
- **UV-Based Sanitization:** Effective disinfection using UV light, with energy distribution visualization.
- **Dynamic Exploration:** Frontier-based exploration to cover unknown areas efficiently.
- **Room-specific Sanitization:** Targeted cleaning of identified rooms with systematic coverage.

---

## Problem Statement
The robot aims to autonomously operate in unknown, confined environments, creating a map, localizing itself, and sanitizing the area using UV lamps.

---

## Objectives
1. **Environment Setup:** Configure simulation environments using TurtleBot3 and Gazebo.
2. **Autonomous Mapping:** Implement SLAM and frontier-based exploration.
3. **Localization & Navigation:** Enable precise localization and goal-oriented navigation.
4. **Sanitization:** Perform thorough cleaning of spaces with UV light.

---

## Installation and Setup

### Prerequisites
- ROS2 (Humble Hawksbill)
- TurtleBot3 Packages
- Gazebo Simulation Environment

## Tasks and Execution

### **Task 1: Environment Setup**
- Modify `model.sdf` files for house and robot models to fit simulation needs.
- Configure SLAM parameters for optimized map creation.
- Adjust Navigation parameters for efficient path planning.

#### Launch Commands:

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
ros2 launch slam_toolbox online_async_launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

---

### **Task 2: Autonomous Mapping**

- Implement frontier exploration and path planning using A* and B-spline smoothing.
- Manage exploration states via a finite state machine (FSM).

#### Launch Commands:

ros2 run map_maker frontier_explorer_node
ros2 run map_maker path_planning_navigator_node

---

-after map making is accomplished, save map.

ros2 run nav2_map_server map_saver_cli -f maps/house_maps

---

### **Task 3: Localization and Navigation**

- Switch middleware to CycloneDDS and configure AMCL parameters for TurtleBot3.
- Localize the robot on the prebuilt map and navigate to predefined goals.

#### Launch Commands:

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/path/to/house_maps.yaml
ros2 run localization_sanitization localization_node
ros2 run localization_sanitization navigation_node

---

### **Task 4: Sanitization**

- Define rooms and UV energy thresholds.
- Visualize UV energy and sanitized areas in RViz.
- Systematically sanitize rooms and track progress.

#### Launch Commands:
-After task 3 is completed, and navigator node  show following message :
-No more goals. Stopping Navigator

ros2 run localization_sanitization sanitization_node

---

