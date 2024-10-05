# LCC_init
LiDAR camera extrinsic parameter initialization calibration
This is a motion-based coarse calibration method for LiDAR-camera extrinsic parameters, with input being a monocular visual odometry and a LiDAR-only odometry.

## 1. Prerequisites
#### 1. ubuntu and ROS
#### 2. PCL Ceres G2O
#### 3. LiDAR-only odometry
#### 4. monocular visual  odometry

## 2. coarse calibration 
```bash
git clone https://github.com/af-doom/LCC_init.git
```
```bash
cd LCC_motion && catkin_make
```
Enter the following folders to modify "LIDAR_ODOM" and "Camera_odom" parameters
```bash
cd LCC_motion/src/LCC_motion/launch/livox_avia.launch
```
