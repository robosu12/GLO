#!/bin/bash
source install/setup_rely.bash

if [ -f /userdata/share/robot/fastrtps_config.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/userdata/share/robot/fastrtps_config.xml
else
    DIR_T="$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=${DIR_T}/install/GR_SLAM/fastrtps_config.xml
fi

ros2 run GR_SLAM lidar_slam_mission_manager --ros-args --params-file ./install/GR_SLAM/params/3DSLAM.yaml

