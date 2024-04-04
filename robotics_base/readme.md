# Catkin ROS Structure

This repository contains a Catkin workspace for a ROS (Robot Operating System) project. Below is a brief description of each directory:

- `CMakeLists.txt`: The CMakeLists.txt file is used by CMake to build the ROS package. It defines how the package should be built and what dependencies it has.

- `coppelia_scenes`: This directory contains CoppeliaSim scene files. CoppeliaSim is a robotics simulation software that is often used in ROS projects for simulation purposes.

- `include`: The include directory typically contains header files (`.h` or `.hpp`) for C++ code. These headers define the interfaces for classes and functions that are implemented in the `src` directory.

- `launch`: This directory contains launch files (`.launch`) for launching ROS nodes and configuring the ROS environment.

- `package.xml`: The package.xml file is used to specify metadata about the ROS package, including its name, version, maintainer, dependencies, and other information.

- `readme.md`: You are currently reading this file! This is the README.md file for the Catkin ROS structure.

- `rviz`: This directory may contain RViz configuration files (`.rviz`) for visualizing ROS data and robot models in 3D.

- `script`: The script directory typically contains executable scripts (`.py`, `.sh`, etc.) that are used in the ROS package.

- `src`: The src directory contains the source code (`.cpp` or `.py`) for ROS nodes and other components of the package.

Feel free to modify and expand upon this structure to suit your specific ROS project needs.

## Launch files 

It launches the c++ code
```bash
 roslaunch robotics_base cpp_robotics_base.launch 
 ```
 
 It launches the python code
 ```bash
 roslaunch robotics_base python_robotics_base.launch 
 ```

## CoppeliaSIM

Download CoppeliaSIM [here](https://www.coppeliarobotics.com/downloads)  *(version 4.6)*


- In one terminal run 

```bash
roscore
```
- After unzipped CoppeliaSIM archive, go to the directory and run it by typing 

```bash
./coppeliaSim.sh 
```
- Open one of the simulated scene you can find in *coppelia_scenes* folder

###
In CoppeliaSIM folder we have different scenes:

In the Sphere scene we have a simple example with just a sphere, that will publish the position of the sphere. So you can play with this scene

```
sphere_pub = simROS.advertise('/sphere/pose','geometry_msgs/PoseStamped')
```


## Nodes Description

<details>
<summary><strong> SIMPLE C++ NODE </strong></summary>

# Simple Explanation of Robotics Base Code

This code defines a class `RoboticsBase` and a `main()` function to control a robotic system. Here's a breakdown of what each part does:

1. **Initialization**: 
   - The `RoboticsBase` constructor takes a reference to a ROS node handle (`nh`) and a private node handle (`private_nh`). It initializes member variables and sets up ROS parameters.
   - The `RoboticsBase` destructor is empty.

2. **Initialization Function (`init()`)**:
   - This function initializes parameters and advertises a topic for publishing poses.

3. **Update Function (`update()`)**:
   - This function publishes a predefined pose (`pose_ee`) to a topic.

4. **Transformation Function (`getTransform()`)**:
   - This function tries to obtain a transformation between two frames ("world" and "base_link") using TF (Transform) library.
   - It constructs an Eigen Affine transform and converts it to a quaternion.
   - It also sends a transformation to represent "link_1" relative to "world".

5. **Main Function (`main()`)**:
   - Initializes ROS, creates node handles (`nh`, `pnh`), and creates an instance of `RoboticsBase`.
   - Calls the `init()` function and enters a loop where `update()` is called repeatedly at a specified rate.
   - It also handles the ROS event processing with `ros::spinOnce()` and warns if the loop rate cannot be maintained.

This code essentially sets up a ROS node to control a robotic system, initializes parameters and publishers, and repeatedly updates the system state.

</details>



<details>
<summary><strong> SIMPLE PYTHON NODE </strong></summary>

# Simple Explanation of Python Robotics Base Code

This Python code defines a class `RoboticsBase` and a `main()` function to control a robotic system using ROS (Robot Operating System). Here's a breakdown of what each part does:

1. **Imports**:
   - `numpy`, `rospy`: Import necessary libraries for numerical operations and ROS functionality.
   - `tf`: Import the `tf` library for handling transformations in ROS.
   - `PoseStamped`, `Pose`, `Point`, `Quaternion`: Import message types from the `geometry_msgs` package.

2. **Class Definition (`RoboticsBase`)**:
   - Constructor (`__init__`): Initializes the class, retrieves ROS parameters, creates a publisher for poses, and sets up a TF listener and broadcaster.
   - `get_transform()`: Obtains a transformation between two frames ('world' and 'base_link'), performs necessary calculations, and broadcasts the transformation between 'link_1' and 'world'.
   - `update()`: Calls `get_transform()` to update the transformation and publishes a predefined pose to a topic.

3. **Main Function (`main()`)**:
   - Initializes the ROS node with the name "robotics_base".
   - Creates an instance of `RoboticsBase`.
   - Enters a loop where `update()` is called repeatedly at a specified rate (`rate`).
   - The loop continues until ROS is shut down.

This code sets up a ROS node to control a robotic system, retrieves transformations between frames, and publishes poses to a topic.

</details>

