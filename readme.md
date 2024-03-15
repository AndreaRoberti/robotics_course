# ROBOTICS 

- Simple ros tutorial inside the ros package. Read the relative [readme](/robotics_base/readme.md)

## Requirements

- Ubuntu 20.04

- ROS Noetic , follow [this](http://wiki.ros.org/noetic/Installation/Ubuntu) guide

- Install catkin tools [doc](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get)


## How to create catkin workspace

This will create the folders and initialize the catkin workspace

```bash
mkdir -p robotics_ws/src
cd robotics_ws
catkin init
```

## How to clone existing workspace

```bash
cd robotics_ws
cd src
git clone REPO_NAME.git
```


## How to create a *NEW* catkin package

```bash
cd robotics_ws
cd src
catkin create pkg robotics_base
```

## How to build

Inside the catkin workspace

```bash
catkin build
```
