# Autonomous Driving Systems

![Work in Progress](https://img.shields.io/badge/status-work%20in%20progress-yellow.svg)

This repository holds the C++ projects and assignments for the university course on Autonomous Driving Systems (ADS). The projects are developed using **ROS2** and are containerized with **Docker** for easy setup and reproducibility.

This is an ongoing effort, and new projects will be added as the course progresses.

---

## Stack

* **Programming Language:** C++
* **Framework:** ROS2
* **Build System:** CMake, Colcon
* **Environment:** Docker, Docker Compose

---

## Contents

This repository is structured into different modules, each corresponding to a specific ADS concept or assignment.

* **`1_clustering`**: Implementations of clustering algorithms for obstacle detection from lidar pointcloud
* **`2_kalman_filter`**: Application of Kalman Filters for object tracking and state estimation.
* **`3_particle_filter`**: Implementation of a Particle Filter (Monte Carlo Localization) for robot localization in structured environment.
* *(More projects to be added...)*

---

## Getting Started

Follow these instructions to set up the environment, build, and run the projects.

### Prerequisites

* [Git](https://git-scm.com/)
* [Docker](https://docs.docker.com/get-docker/)
* [Docker Compose](https://docs.docker.com/compose/install/)

### 1. Setup the Environment

First, clone the repository and launch the Docker environment.

```bash
# 1. Clone this repository
git clone <git@github.com:FilippoGib/Autonomous_Driving_Systems.git>
cd Autonomous_Driving_Systems

# 2. Make the helper scripts executable (only need to do this once)
chmod +x start.sh
chmod +x stop.sh

# 3. Build and start the Docker container(s)
./start.sh
```
This will launch the services defined in `docker-compose.yml` and attach you to the shell of the main development container.

To stop the containers when you're done, run `./stop.sh` from the host machine.

### 2. Build and Run a Project

My recommendation is to attach to the container inside vscode via the [vscode docker extention](https://code.visualstudio.com/docs/containers/overview) to make the necessary changes (eg: change source data path), navigate to the sub folder you are interested in and build it using ither `cmake` if the project does not use ROS2:
```bash
# 1. (e.g., for the kalman filter)
cd /path/to/2_kalman_filter

# 2. create a 'build' directory
cmake -S ./code -B build

# 3. build the project
cmake --build build

# 4. run the executable
./build/main
```
or using `colcon`

```bash
# 1. 
cd /path/to/3_particle_filter
# 2.
source /opt/ros/rolling/setup.bash
# 3. 
colcon build --symlink-install
# 3.
source install/setup.bash
# 4.
ros2 run pf pf_node
```

## Disclaimer

To actually run the code successfully you will need the datasets which are not inlcuded in this repository. These datasets are property of __Hipert srl__. To access them please contact me directly to my private email.

