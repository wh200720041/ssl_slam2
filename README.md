# SSL_SLAM2
## Lightweight 3-D Localization and Mapping for Solid-State LiDAR (Intel Realsense L515 as an example)

### This repo is an extension work of [SSL_SLAM](https://github.com/wh200720041/SSL_SLAM). Similar to RTABMAP, SSL_SLAM2 separates the mapping module and localization module. Map saving and map optimization is enabled in the mapping unit. Map loading and localization is enabled in the localziation unit.

This code is an implementation of paper "Lightweight 3-D Localization and Mapping for Solid-State LiDAR", published in IEEE Robotics and Automation Letters, 2021 [paper](https://arxiv.org/pdf/2102.03800.pdf)

A summary video demo can be found at [Video](https://youtu.be/Uy_2MKwUDN8) 

**Modifier:** [Wang Han](http://wanghan.pro), Nanyang Technological University, Singapore

Running speed: 20 Hz on Intel NUC, 30 Hz on PC

## 1. Solid-State Lidar Sensor Example
### 1.1 Scene reconstruction example
<p align='center'>
<a href="https://youtu.be/D2xt_5xm_Ew">
<img width="65%" src="/img/mapping.gif"/>
</a>
</p>

### 1.2 Localization with built map 
<p align='center'>
<a href="https://youtu.be/D2xt_5xm_Ew">
<img width="65%" src="/img/localization.gif"/>
</a>
</p>

### 1.3 Comparison
<p align='center'>
<a href="https://youtu.be/D2xt_5xm_Ew">
<img width="65%" src="/img/comparison.gif"/>
</a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4. **GTSAM**
Follow [GTSAM Installation](https://gtsam.org/get_started/).

### 2.5. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 3. Sensor Setup
If you have new Realsense L515 sensor, you may follow the below setup instructions

### 3.1 L515
<p align='center'>
<img width="35%" src="/img/realsense_L515.jpg"/>
</p>

### 3.2 Librealsense
Follow [Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### 3.3 Realsense_ros
Copy [realsense_ros](https://github.com/IntelRealSense/realsense-ros) package to your catkin folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    catkin_make
```

## 4. Build SSL_SLAM2
### 4.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/ssl_slam2.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 4.2 Download test rosbag
You may download our recorded data: [MappingTest.bag](https://drive.google.com/file/d/1XRXKkq3TsUiM4o9_bWL8t9HqWHswfgWo/view?usp=sharing) (3G) and [LocalizationTest.bag](https://drive.google.com/file/d/1-5j_jgraus0gJkpFRZS5hFUiKlT7aQtG/view?usp=sharing) (6G)if you dont have realsense L515, and by defult the file should be under home/user/Downloads

unzip the file (it may take a while to unzip) 
```
cd ~/Downloads
unzip LocalizationTest.zip
unzip MappingTest.zip
```

### 4.3 Map Building
map optimization and building
```
    roslaunch ssl_slam2 ssl_slam2_mapping.launch
```
The map optimization is performed based on loop closure, you have to specify the loop clousre manually in order to trigger global optimization. To save map, open a new terminal and 
```
  rosservice call /save_map
```
Upon calling the serviece, the map will be automatically saved. It is recommended to have a loop closure to reduce the drifts. Once the service is called, loop closure will be checked. 
For example, in the rosbag provided, the loop closure appears at frame 1060-1120, thus, when you see "total_frame 1070" or "total_frame 1110" you may immediately type 
```
  rosservice call /save_map
```
Since the current frame is between 1060 and 1120, the loop closure will be triggered automatically and the global map will be optimized and saved 

### 4.4 Localization

Type
```
    roslaunch ssl_slam2 ssl_slam2_localization.launch
```
If your map is large, it may takes a while to load

### 4.5 Parameters Explanation
The map size depends on number of keyframes used. The more keyframes used for map buildin, the larger map will be. 

min_map_update_distance: distance threshold to add a keyframe. higher means lower update rate. 
min_map_update_angle: angle threshold to add a keyframe. higher means lower update rate. 
min_map_update_frame: time threshold to add a keyframe. higher means lower update rate. 


### 4.6 Relocalization
The relocalization module under tracking loss is still under development. You must specify the robot init pose w.r.t. the map coordinate if the starting position is not the origin of map. You can set this by  
```
    <param name="offset_x" type="double" value="0.0" />
    <param name="offset_y" type="double" value="0.0" />
    <param name="offset_yaw" type="double" value="0.0" />
```

### 4.7 Running speed
The realsense is running at 30Hz and some computer may not be able to support such high processing rate. You may reduce the processing rate by skipping frames. 
You can do thid by setting the 
```
<param name="skip_frames" type="int" value="1" />
```
1 implies no skip frames, i.e., 30Hz;  implies skip 1 frames, i.e., 15Hz. For small map building, you can do it online. however, it is recommended to record a rosbag and build map offline for large mapping since the dense map cannot be generated in real-time.

## 5 Map Building with multiple loop closure places 
### 5.1 Dataset
You may download a larger dataset [LargeMappingTest.bag](https://drive.google.com/file/d/18HWUpgv7G6MV6brtbA4uEluTOgxDEdX3/view?usp=sharing) (10G), and by defult the file should be under home/user/Downloads

unzip the file (it may take a while to unzip) 
```
cd ~/Downloads
unzip LargeMappingTest.zip
```

### 5.2 Map Building
Two loop closure places appear at frame 0-1260 and 1270-3630, i.e., frame 0 and frame 1260 are the same place, frame 1270 adn 3630 are the same place. Run
```
    roslaunch ssl_slam2 ssl_slam2_large_mapping.launch
```
open a new terminal, when you see "total_frame 1260", immediately type
```
  rosservice call /save_map
```
when you see "total_frame 3630", immediately type again
```
  rosservice call /save_map
```

## 6. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated 
```
@article{wang2021lightweight,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lightweight 3-D Localization and Mapping for Solid-State LiDAR}, 
  year={2021},
  volume={6},
  number={2},
  pages={1801-1807},
  doi={10.1109/LRA.2021.3060392}}
```