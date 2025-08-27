#!/bin/bash
source /opt/ros/humble/setup.bash

workspace=`pwd`
workspace_cam=$workspace/cam

read -p "Do you want to rebuild the packages (y/n): " rebuild_cam
rebuild_cam=${rebuild_cam,,}

cd $workspace/cam
if [[ "$rebuild_cam" == "y" ]]; then
    # rosdep install --from-paths src --ignore-src -y
    colcon build --packages-select usb_cam radar_camera_fusion my_camera_pkg yolo_overlay
fi
source install/setup.bash

cd $workspace/mmwave_ti_ros/ros2_driver
if [[ "$rebuild_cam" == "y" ]]; then
    colcon build 
fi
source install/setup.bash

cd $workspace/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch
ros2 launch 6843AOP_Standard.py &

cd $workspace/cam/src
sed s/MYWORKSPACE/$workspace_cam/ usb.patch > fix.patch
patch -p0 < fix.patch

cd $workspace/cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb -p camera_info_url:=$workspace/cam/src/camera_info.yaml -p framerate:=15.0 &
pid_usb_cam=$!

ros2 run yolo_overlay yolo_overlay_node &
pid_yolo=$!

ros2 launch radar_camera_fusion overlay.launch.py
pid_overlay=$!

trap "echo 'Stopping nodes'; kill $pid_usb_cam $pid_yolo $pid_overlay; exit" SIGINT

wait