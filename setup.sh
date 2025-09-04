#!/bin/bash
source /opt/ros/humble/setup.bash

workspace=`pwd`
workspace_cam=$workspace/cam

read -p "Do you want to rebuild the packages (y/n): " rebuild_cam
rebuild_cam=${rebuild_cam,,}


if [[ "$rebuild_cam" == "y" ]]; then
    cd $workspace/cam/src
    sed s%MYWORKSPACE%$workspace_cam% usb.patch > fix.patch
    patch -p0 --forward < fix.patch || true
    cd $workspace/cam
    # rosdep install --from-paths src --ignore-src -y
    colcon build --packages-select usb_cam radar_camera_fusion my_camera_pkg yolo_overlay
fi
cd $workspace/cam
source install/setup.bash

cd $workspace/mmwave_ti_ros/ros2_driver
if [[ "$rebuild_cam" == "y" ]]; then
    colcon build 
fi
source install/setup.bash

cd $workspace/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch
ros2 launch 6843AOP_Standard.py &

cd $workspace/cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb -p camera_info_url:=file://$workspace/cam/src/camera_info.yaml -p framerate:=15.0 &
pid_usb_cam=$!

cd $workspace/cam/src  
ros2 launch usb_cam_launch.py
pid_remapping=$!

# cd $workspace/cam/src/isaac_ros_common/scripts
# ./run_dev.sh
# ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize.launch.py model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan  input_binding_names:=['images'] output_binding_names:=['output0'] network_image_width:=640 network_image_height:=640  force_engine_update:=False image_mean:=[0.0,0.0,0.0] image_stddev:=[1.0,1.0,1.0] input_image_width:=640 input_image_height:=640 confidence_threshold:=0.25 nms_threshold:=0.45 &
# pid_isaac_yolo=$!




# source install/setup.bash
# ros2 run yolo_overlay yolo_overlay_node &
# pid_yolo=$!

# ros2 launch radar_camera_fusion overlay.launch.py &
# pid_overlay=$!

trap "echo 'Stopping nodes'; kill $pid_usb_cam $pid_yolo $pid_overlay; exit" SIGINT

wait
