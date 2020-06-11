NDTPSO-SLAM
===========

NDTPSO-SLAM is a ROS package which provides an implementation of the scan matching algorithm proposed in [ref](#),
It uses NDT (Normal distribution transform) to modelize the environnement and PSO (Particle swarm
optimisation) to perform the scan matching and solve the pose problem.

This package has been tested with ROS Melodic on Ubuntu 18.04.

This project is devided to two parts, the `libndtpso_slam` which is a C++ library that implements the
proposed scan matching method, and in the other part, the ROS node `ndtpso_slam_node` which provides the ROS interface to this
library.

### Maintainer:
Abdelhak BOUGOUFFA, Paris-Saclay University, SATIE Laboratory <[abdelhak.bougouffa[at]universite-paris-saclay.fr](mailto:abdelhak.bougouffa@universite-paris-saclay.fr)>

