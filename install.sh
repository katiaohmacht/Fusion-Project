#!/bin/bash
source /opt/ros/humble/setup.bash

workspace=`pwd`
workspace_cam=$workspace/cam

cd $workspace/cam
colcon build 
source install/setup.bash

cd $workspace/mmwave_ti_ros2/ros2_driver
colcon build 
source install/setup.bash

cd $workspace/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch
ros2 launch 6843AOP_Standard.py &

cd $workspace/cam/src
sed s/MYWORKSPACE/$workspace_cam/ usb.patch >fix.patch
patch -p0 < fix.patch


cd $workspace/cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb -p camera_info_url:=$workspace/cam/src/camera_info.yaml -p framerate:=15.0 &

ros2 launch radar_camera_fusion overlay.launch.py &