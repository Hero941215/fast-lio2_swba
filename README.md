# fast-lio2_swba
**A LiDAR-Inertial mapping package based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), and sliding window based [lidar BA (balm2, PA) algorithm](https://github.com/hku-mars/BALM).**

** **

## 0. Features
Build a loosely coupled mapping system with Fast-lio2 as the front-end and a sliding window based laser BA as the back-end. Integrate the latest BA algorithms such as BALM2 and PA into the backend of the system to refine voxel maps. The improved system can achieve centimeter level positioning accuracy in structured scenes such as buildings, with better mapping quality than [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.3, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

## 2. Build

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/Hero941215/fast-lio2_swba
    cd fast-lio2_swba
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)

## 3. Run
### 3.1. **run with hilti dataset**

roslaunch robotrun tight_slam_ouster_indoor.launch (hilti-2021)

roslaunch robotrun tight_slam_pandar_indoor.launch (hilti-2022)

### 3.2. **run with UrbanNav dataset**

roslaunch robotrun tight_slam_velodyne_outdoor.launch

### 3.3. **run with livox mid-360**

roslaunch robotrun tight_slam_mid360_indoor.launch

## 4. Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [BALM2](https://github.com/hku-mars/BALM).
