NDTPSO-SLAM
===========

NDTPSO-SLAM is a ROS package which provides an inhanced implementation of the scan matching algorithm proposed in [1] and [2].
It uses NDT (Normal Distribution Transform) to model the environnement and PSO (Particle Swarm Optimisation) to perform the scan matching optimization and solve the pose problem.

This package has been tested with ROS Melodic on Ubuntu 18.04, however; it should work with newer ROS1 versions.

This project is devided to two parts, the `libndtpso_slam` which is a C++ library that implements the proposed scan matching method, and in the other part, the ROS node `ndtpso_slam_node` which provides the ROS interface to this library.

Abdelhak BOUGOUFFA
<`abdelhak [dot] bougouffa [at] universite-paris-saclay.fr`>

### If you used this work, please cite:
- [1]: **Sara BOURAINE**, **Abdelhak BOUGOUFFA** and **Ouahiba AZOUAOUI**, _Particle Swarm Optimization for Solving a Scan-Matching Problem Based on the Normal Distributions Transform_, [`10.1007/s12065-020-00545-y`](https://doi.org/10.1007/s12065-020-00545-y), Evolutionary Intelligence, Jan 2021.
- [2]: **Sara BOURAINE**, **Abdelhak BOUGOUFFA** and **Ouahiba AZOUAOUI**, _NDT-PSO, a New NDT based SLAM Approach using Particle Swarm Optimization_, [`10.1109/ICARCV50220.2020.9305519`](https://doi.org/10.1109/ICARCV50220.2020.9305519), 16th International Conference on Control, Automation, Robotics and Vision (ICARCV 2020), Dec 2020.

# Build NDTPSO-SLAM
This package has been tested on ROS Melodic and ROS Noetic

## Dependencies
1. Eigen3 (can be installed from package manager)
2. OpenCV3+ (OPTIONAL: used to export map image)

To build NDTPSO-SLAM, clone it to your ROS Catkin workspace.

```shell
cd path/to/catkin_ws/src
git clone https://github.com/abougouffa/ndtpso_slam.git
cd ..
catkin_make --pkg ndtpso_slam
```

# Using the provided node
You can edit the provided launch files to fit your LiDAR topic name and run:

```shell
roslaunch ndtpso_slam scan.launch
```
