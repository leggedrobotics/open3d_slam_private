# open3d_slam_private

This project provides tools for robot **LiDAR-based mapping and localization**.

This is an advanced and fine-tuned version of open3d_slam that is private to RSL for internal projects and development.

## Build
For now to build this repository, you need to:
1. make sure CMake version is correct
2. install dependencies from `open3d_catkin` 
3. install dependencies for `open3d_slam`

Clone the repository to:
``` bash
cd catkin_ws/src
git clone git@github.com:leggedrobotics/open3d_slam_private.git
```

1. Make sure your CMake > 3.18
```bash
cmake --version
```
**If NOT** install the cmake from https://cmake.org/download download tar.
Unpack tar and
``` bash
cd cmake-<version>
./configure
make -j<num_of_processes>
sudo make install
```

2. Install the necessary dependencies from `open3d_catkin`:
```bash
cd open_3d_slam_rsl/open3d_catkin/
./install_deps.sh
```

3. Install libraries required from `open3d-slam`
```bash
sudo apt install libgoogle-glog-dev libglfw3 libglfw3-dev liblua5.2-dev
``` 

4. Install `message_logger` from Anybotics (Temporary) by cloning it to `catkin_ws/src`

```bash
git clone git@github.com:ANYbotics/message_logger.git
```

And finally:

```bash
cd catkin_ws
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
```

## Use

```bash
source devel/setup.sh
```
To perform **mapping**, call:
```bash
roslaunch open3d_slam_ros mapping.launch
```

While playing the rosbag file:

```bash
rosbag play --clock --pause -s 0 -r 1.0 <your_great_bag>.bag
```
Make sure you have correct topics for lidar point clouds and odometry.
