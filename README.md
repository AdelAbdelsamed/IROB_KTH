# IROB_KTH
Introduction to Robotics (IROB) coursework at KTH in the autumn term of 2024.

# Introduction to Robotics Coursework
This repository contains the coursework for the **Introduction to Robotics** course, covering five assignments designed to introduce and explore foundational concepts in robotics using ROS (Robot Operating System). Each assignment builds on the knowledge gained in previous tasks, culminating in practical implementations for robot control and navigation.

## Assignments Overview

### 1. **Introduction to ROS**
   This assignment introduces ROS and sets up the environment to control a simulated Turtlebot3 Burger robot. Key objectives include:
   - Understanding the ROS master, launching nodes, and interacting with topics, services, and actions.
   - Using RViz to visualize the robot and the environment.
   - Implementing a controller to explore unknown space by:
     - Fetching paths from the `/explore` action server.
     - Ensuring safe navigation via the `/collision_avoidance` service.
     - Transforming points to the robot's frame using TF2.
     - Publishing velocity commands to the `/cmd_vel` topic.
   - Preventing the robot from getting stuck and optimizing exploration by managing linear and angular velocities.

### 2. **Assignment 2: Inverse Kinematics**

**Description**: 
- Program the inverse kinematic (IK) algorithm for a robot to move its joints so that the end-effector follows a desired path.

**Structure**: 
1. **3 DOF SCARA Robot**:
   - Develop an analytic solution for the inverse kinematics of a 3 DOF SCARA robot.

2. **7 DOF KUKA Robot**:
   - Implement the inverse kinematics solution using iterative algorithms.

### 3. **Planning: RRT* Dubin's Car**
In this assignment, a robotic planning method, RRT*, is implemented to drive a Dubins car with the following dynamics:

```
x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t])
```

The goal is to move the car from an initial position `(x0, y0)` to a target position `(xt, yt)` while avoiding collisions with obstacles and staying within bounds.

### 4. **Mapping **
This assignment involves updating a grid-based occupancy map using laser scan data and the robot's pose. It is split into two tasks:

## Part 1:
In this part, the `update_map(self, grid_map, pose, scan)` function is implemented to convert laser scan readings into map coordinates and mark occupied spaces in the grid. The goal is to accurately fill in the map with detected obstacles.

## Part 2:
Building on Part 1, this task involves addressing two issues:
1. **Noise**: Using ray tracing to clear free space between the robot and obstacles.
2. **Free Space**: Marking free spaces and expanding occupied cells to create a **C-space** map for path planning. 

Only the affected portions of the map are updated to improve efficiency.

### 5. **Mission Planner for TIAGo Robot**

The final project involves simulating a TIAGo robot in an apartment using Gazebo. The robot is equipped with sensors and a manipulator arm that allow it to autonomously navigate and perform simple manipulation tasks. The goal is to implement a mission planner for TIAGo to execute three different missions.

## Project Format
In this project, multiple pre-existing modules (e.g., planning, sensing, navigation, manipulation) are integrated to create a mission planner node. This node will be responsible for high-level task planning, commanding the robot where to go and what actions to perform based on its current state and mission specifications.

Your tasks will include:
- **Mission Planner Node**: Implement the `sm_students.py` file to create a state machine (SM) for executing the mission's sequence of steps. The sequence will depend on the robot's state and the available modules. An example SM is provided in the working package.
- **Behavior Tree (BT)**: Implement a behavior tree equivalent of the state machine in the `bt_students.py` file.
- **Launch File**: Create a `launch_project.launch` file to deploy and connect the necessary nodes and components to perform the tasks. The launch file requires to substitute placeholder names with meaningful parameters.

### Main Task States
The robot must:
1. Localize itself in the apartment.
2. Navigate to the picking pose.
3. Detect the cube.
4. Complete the picking task.
5. Navigate with the cube to a second table.
6. Complete the placing task.
7. Check if the cube is placed on the table:
   - If **yes**, end the mission.
   - If **no**, repeat the process from state 2 and respawn the cube.

### Observations:
1. **Kidnapping**: The robot must detect if it has been manually moved and react appropriately to regain its true position and avoid collisions. Test by manually "kidnapping" the robot during the implementation.
2. **Cube Dropping**: During navigation, the cube may be manually removed from the robot's gripper. The robot must detect this failure and retry the placing task.
3. **Localization**: The robot uses a particle filter for localization. You must use the state of the particle distribution to determine when the filter has converged.
4. **No True State Usage**: Do not use the true state of models (e.g., `/gazebo/model_states`) to check if the cube is grasped or if localization has failed.

---



