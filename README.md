# LIO-IMM: An Effective LiDAR-Inertial Odometry Using Interacting Multiple Models for Localization and Mapping
LIO-IMM is a high-level fusion framework for multiple motion models, capable of flexibly estimating both linear and non-linear trajectories resulting from aggressive movements, using point-to-point GICP for registration. 
The paper has been submitted to ICRA 2025 and is currently under review.

<p align='center'>
    <img src="./doc/SEM-UNR.png" alt="drawing" width="1000"/>
</p>

# Dependencies
The framework has been tested with ROS Noetic and Ubuntu 20.04. The following configuration, along with the required dependencies, has been verified for compatibility:

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (```roscpp```, ```std_msgs```, ```sensor_msgs```, ```geometry_msgs```)
- C++ 14
- [CMake >= 3.16.3](https://cmake.org/download/)
- [PCL >= 1.8](https://pointclouds.org/downloads/)
- [Eigen >= 3.3.4](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ZED SDK >= 3.5](https://www.stereolabs.com/developers)
- [CUDA](https://developer.nvidia.com/cuda-downloads) (Recommend to use CUDA toolkit >= 11 for Ubuntu 20.04)

```
    sudo apt install libomp-dev libpcl-dev libeigen3-dev
```

The LIO-IMM has been tested with various LiDAR configurations, including Velodyne, Ouster, and Hesai, requiring the point cloud in the format of ```sensor_msgs::PointCloud2``` and the IMU in the format of ```sensor_msgs::IMU```. To ensure optimal performance, the LiDAR-IMU system should be well-calibrated and time-synchronized.

# Install
Clone the ``` LIO-IMM ``` repository into the catkin workspace ``` ~/catkin_ws/src/ ``` folder, and compile package (LIO-IMM supports implementation in ROS1 as default)

```
    mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
    git clone https://github.com/aralab-unr/LIO-IMM.git
    cd ..
    source devel/setup.bash
    catkin_make
```
# Running

Download [test data](https://drive.google.com/drive/folders/1NFlAAppbvM5jfm1xsrYEr3mfzbS9PRHb?usp=drive_link) which are collected in UNR campus for testing. The dataset is collected by using a VLP16 PuckLite and a Zedm camera. The point cloud topc is ``` "/velodyne points ```, and the IMU topic is ``` "/zed/zed_nodelet/imu/data ``` 

```
    roslaunch lio-imm liom.launch \pointcloud:=/velodyne_points \imu_topic:=/zed/zed_nodelet/imu/data
```
```
    rosbag play file-name.bag
```

# Acknowledgement
We thank the authors of [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry), and [FastGICP](https://github.com/koide3/fast_gicp) for providing open-source packages:

- K. Chen, R. Nemiroff and B. T. Lopez, "Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 3983-3989, doi: 10.1109/ICRA48891.2023.10160508.

- Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in IEEE International Conference on Robotics and Automation (ICRA), IEEE, 2021, pp. 11 054–11 059.

# Contact
- [An Nguyen](mailto:anguyenduy@nevada.unr.edu)
- [Hung La](mailto:hla@unr.ed

