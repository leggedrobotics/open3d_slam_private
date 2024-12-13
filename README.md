# Degeneracy Mitigation in LiDAR SLAM
Official implementations from the paper "Informed, Constrained, Aligned: A Field Analysis on Degeneracy-aware Point Cloud Registration in the Wild" [arxiv link](https://arxiv.org/abs/2408.11809). The paper version will be made publically available after acceptance.

## Components
The work investigates different degeneracy mitigation methods for the problem of degenerate point cloud registration. To achieve this, this work relies on many open-sourced libraries. These libraries / packages are subject to their licenses. 

- This work builds on the open-source version of [Open3D SLAM](https://github.com/leggedrobotics/open3d_slam).
- This work uses libpointmatcher point cloud registration library in the backend. The locally available libpointmatcher library is based on this [version](https://github.com/ANYbotics/libpointmatcher).
- The locally available `pointmatcher_ros` package is based on [this version](https://github.com/ANYbotics/pointmatcher-ros).
- For NL-Reg. and NL-Solver methods, this work depends on [Ceres](https://github.com/ceres-solver/ceres-solver).
- For the Ineq. Con. method, this work relies on the QPmad quadratic problem library [link](https://github.com/asherikov/qpmad).
- To replicate the result of the work of Petracek et al. (RMS), the authors [forked](https://github.com/leggedrobotics/RMS) the [original work](https://github.com/ctu-mrs/RMS).


## Dependencies (on top of the Components / or part of them)
1. Ubuntu 20.04
2. ROS Noetic
3. CMake > 3.18
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
4. Open3D, automatically installed by `open3d_catkin` package (this take 6-7gb of space).
5. libnabo (dependency of libpointmatcher) [link](http://github.com/anybotics/libnabo)
6. install dependencies for `open3d_slam` (`sudo apt install libgoogle-glog-dev libglfw3 libglfw3-dev liblua5.2-dev`)
7. message_logger [repository link](https://github.com/ANYbotics/message_logger)

## Build
For now to build this repository, you need to:

Clone the repository to:
``` bash
cd catkin_ws/src
git clone git@github.com:leggedrobotics/open3d_slam_private.git.
cd ..
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build open3d_slam_ros
```

## Running / Result replicating
Here, we describe the process for 1 dataset but this process apply to all datasets (with their respective config files).

1. Get the `.bag` file for the [ANYmal forest experiment](https://drive.google.com/drive/folders/1Y8w1Twdv2db-PMsx9uKMwGQ48yDisjx6)
``` bash
roslaunch open3d_slam_ros replay_forest.launch rosbag_filepath:=/path/to/your_file.bag
```
2. An RVIZ window pop-up and replay the rosbag as fast as possible.
3. The results of the test will be saved to the pre-defined folder in the launch file.

## Changing the method

All the degeneracy point cloud registration method parameters are located in (open3d_slam_rsl/ros/open3d_slam_ros/param/icp.yaml)

<details>
<summary>P2Plane</summary>

```
degeneracyAwareness:
  None:
```


</details>



## Acknowledgement
We would like to thank all the researchers who made their work open-source to the community, which allowed us to compose this work.

## License
All the 3rd party libraries and repositories are subject to the license of their own.