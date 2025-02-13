<div align="center">
  
# PySplanner - The All-In-One Solution to FLL
![image](https://github.com/user-attachments/assets/87b21e7c-dc66-425c-86f1-01af1bf3f9b8)

</div>

# What is this?
PySplanner is a compilation of programs that all make up a simple and easy to use path follower for your Spike PRIME and EV3 robots. It is ecspecially helpful in FLL. Its also extremely begginer friendly, as all of the front end interfaces are visual and involve no coding, but even for advanced users theres an ability to code regularly with a lot of documentation on various class and functions.


# How does it work?
PySplanner uses a modified pure pursuit algorithim to follow the pre-planned path with a mix of speed and accuracy.

### Path Planner
PySplanner includes a path planner program made using OpenCV-Python that allows you to plot points for the robot to follow. These puints are connected with cubic polynominal splines. It also has support for using the FLL table as a background, updated according to the current season.

### Path Editor
Allows you to edit a path created with teh path planner, you can move points by the millimeter for maximum precision. You can also add actions for the hub to perform at points along the path like moving a motor, these can be blocking or non-blocking.

### Coordinate Updater
The base of our program is the coordinate updater, which is a function that repeatedly runs on the hub and uses info like angular velocity, gyro heading, distance encoders and trigonometry to accurately assume the robot's location relative to its starting point. The position is used to ensure the robot is following the path with accuracy.

### Pure Pursuit Algorithim
The algorithim helps a robot follow a path by determining where it should go next and how fast it should move. It looks ahead on the path to find a point for the robot to aim at, calculates how much the robot should turn, and adjusts its speed based on how sharp the path curves. It also checks if the robot is getting close to the end of the path and slows down as it approaches. It also includes slow starting to ensure that slippage does not occur.

### Visualization
While testing out paths made, we have a script that connects the hub to the PC and shows a visualization window of the robot's path, location and various other data types.

### PySplanner Run File
All of this has been concatenated into one file with a run selector and additional optimizations. There is currently a version for EV3 and Spike PRIME/Inventor.
