#!/bin/bash
# ====================================================================
# Launch Front ZED X Camera with Default CAD Extrinsics & Intrinsics
# Vehicle: Autonomous Buggy (Gigabit Ethernet to Intel ROS Master)
# ====================================================================

source /opt/ros/noetic/setup.bash
source /home/infy/catkin_ws/devel/setup.bash
source /home/infy/connect_to_intel_ethernet.sh

exec roslaunch zed_wrapper zedx.launch \
    camera_model:=zedx \
    camera_id:=1 \
    cam_pos_x:=1.3894 \
    cam_pos_y:=0.32 \
    cam_pos_z:=0.5717 \
    cam_roll:=0.0 \
    cam_pitch:=0.0 \
    cam_yaw:=0.0 \
    base_frame:=base_link \
    publish_tf:=true \
    publish_map_tf:=false
