---
authors: 
  - name: DylDev
    link: https://github.com/DylDevs
    avatar: https://avatars.githubusercontent.com/u/110776467?v=4
date: 2024-10-22
icon: home
order: 2
---

# Welcome to PySplanner!


## What is PySplanner?
PySplanner is a robot control software primarily designed for use in First LEGO League (FLL) competitions but is adaptable for other robotic applications. It uses the Pure Pursuit algorithm to enable precise navigation for robots like LEGO's EV3 and SPIKE hubs. PySplanner supports a flexible mission structure, referred to as "table runs," where the robot follows pre-defined paths on a competition table. These runs are tailored with parameters such as speed, turning curvature, and proximity to targets, making it ideal for FLL missions that require high accuracy in navigation.

The program is equipped with a visualization feature that allows users to see the robot's movement in real time on a connected device, enhancing debugging and performance monitoring. This feature is supported through either a TCP connection (for EV3) or BLE (for SPIKE). PySplanner is designed with multi-threading capabilities to handle various tasks concurrently, such as running table missions and managing sensor feedback without sacrificing performance. It provides a user-friendly interface with a run selector, enabling easy mission selection and execution.

In addition to FLL, PySplanner's adaptable architecture makes it a strong tool for other robotics projects that need reliable navigation and control, thanks to its scalable design and customizable run parameters.

<hr>

## How PySplanner Operates
PySplanner employs an optimized version of the Pure Pursuit algorithm to balance both speed and precision when guiding a robot along a pre-defined path. By utilizing a combination of real-time data and carefully calculated path parameters, PySplanner ensures the robot follows its intended course efficiently, adapting dynamically to changes in terrain and conditions.

### Path Planner
At the core of PySplanner is its path planner, built using OpenCV-Python, which enables users to define precise routes for the robot to follow. Users can plot waypoints along the robot’s intended route, which are then connected using cubic polynomial splines for smooth transitions. The path planner also integrates an FLL (First LEGO League) competition table background, allowing for the paths to be created with the season-specific table layout. This feature is invaluable for competition teams looking to simulate real-world conditions in their path planning.

!!!
The path planner is being transitioned to a web-based UI hosted at https://pysplanner.github.io (Currently not usable)
!!!

### Path Editor
The path editor provides users with the flexibility to adjust and refine the planned routes with millimeter-level precision. By moving waypoints within the path, users can optimize the robot's trajectory for maximum performance. Additionally, the path editor supports the addition of specific actions along the route, such as activating motors or performing other tasks. These actions can be configured as either blocking or non-blocking, allowing for precise timing and coordination during execution. This degree of customization ensures that robots can handle complex missions autonomously with minimal intervention.

### Coordinate Updater
The foundation of PySplanner is its coordinate updater, a continuously running function on the robot’s hub. This system utilizes data from various sensors—such as angular velocity, gyro heading, and distance encoders—coupled with advanced trigonometric calculations, to accurately determine the robot’s position relative to its starting point. By consistently monitoring and updating the robot’s coordinates, PySplanner ensures the robot is staying on course and making real-time adjustments to maintain path accuracy, even in the presence of external factors like wheel slippage or unexpected obstacles.

### Pure Pursuit Algorithm
The Pure Pursuit algorithm is the driving force behind PySplanner's navigation. This algorithm dynamically calculates where the robot should go next by looking ahead to a future point on the path, allowing the robot to "pursue" the path efficiently. It determines the optimal turn angle and speed based on the sharpness of the curves, ensuring smooth transitions between points. As the robot nears the end of its path, the algorithm gradually reduces speed, ensuring controlled deceleration and preventing overshoot. Additionally, the algorithm incorporates slow starting mechanics to prevent slippage and ensure the robot gains traction smoothly from a standstill.

### Visualization
To facilitate testing and optimization, PySplanner includes a visualization module that connects the robot hub to a PC. This feature opens a window displaying a real-time visual representation of the robot’s path, its current position, and other critical data points. The visualization allows developers to monitor the robot’s behavior in action, making it easier to fine-tune the robot’s performance and validate path accuracy before real-world deployment.

### PySplanner Run File
All components of PySplanner—path planning, coordinate updating, and pure pursuit execution—are consolidated into a streamlined run file. This file also includes a run selector for easy management of multiple missions, alongside optimizations tailored for both EV3 and SPIKE PRIME/Inventor platforms. With the option to choose between these platforms, users can leverage PySplanner’s full capabilities on their preferred hub, benefiting from its robust, optimized, and heavily-tested framework.