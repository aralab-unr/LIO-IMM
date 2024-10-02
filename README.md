# LIO-IMM: An Effective LiDAR-Inertial Odometry Using Interacting Multiple Models for Localization and Mapping
LIO-IMM is a high-level fusion framework for multiple motion models, capable of flexibly estimating both linear and non-linear trajectories resulting from aggressive movements, using point-to-point GICP for registration. 
The paper has been submitted to ICRA 2025 and is currently under review.

<p align='center'>
    <img src="./doc/SEM-UNR" alt="drawing" width="800"/>
</p>

# Dependencies
The framework has been tested with ROS Noetic and Ubuntu 20.04. The following configuration, along with the required dependencies, has been verified for compatibility:

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (```roscpp```, ```std_msgs```, ```sensor_msgs```, ```geometry_msgs```)
- C++ 14
- [PCL >= 1.8](https://pointclouds.org/downloads/)
- [Eigen >= 3.3.4](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [ZED SDK >= 3.5](https://www.stereolabs.com/developers)
- [CUDA](https://developer.nvidia.com/cuda-downloads) (Recommend to use CUDA toolkit >= 11 for Ubuntu 20.04)

  
