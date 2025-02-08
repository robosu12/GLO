# GLO - General LiDAR-only Odometry

## Compile

### Dependency

- **`ubuntu20.04 + ROS noetic`**
- **`Eigen`**
- **`ceres`**
- **`PCL`**
- **`yaml-cpp`**
```
    sudo apt update
    sudo apt install libyaml-cpp-dev
```

### Build
```
mkdir -p ~/catkin_ws_grodom/src
cd ~/catkin_ws_grodom/src
git clone https://github.com/robosu12/GLO.git
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

```

## Usage

### Run glo with kitti dataset

First, modify the dataset_path in GLO/params/3DSLAM_kitti.yaml with your dataset path.
```
config file: GLO/params/3DSLAM_kitti.yaml
```
Second, launch the GLO node and rviz: 
```
roslaunch grodom_ros1 grodom_kitti.launch.
```
You can change the kitti_sequence number in GLO/params/3DSLAM_kitti.yaml to operate glo in different sequence.

### Run glo with NTU dataset

First, launch the GLO node and rviz: 
```
roslaunch grodom_ros1 grodom_NTU.launch.
```
Second, play NTU rosbag: 
```
rosbag play xxx.bag. 
```

### Run glo with you rosbag

First, modify the cloud_topic_name in GLO/params/3DSLAM_RS16.yaml with your topic name.
```
config file: GLO/params/3DSLAM_RS16.yaml
```
Second, launch the GLO node and rviz: 
```
roslaunch grodom_ros1 grodom_RS16.launch
```
Third, play your rosbag: 
```
rosbag play xxx.bag. 
```
please ensure the topic in rosbag is same with in config file.

#### Developers
YunSu


