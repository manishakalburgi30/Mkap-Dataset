#!/bin/bash
# ====================================================================
# Autonomous Buggy - Dataset Collection & Auto-Exporter Launch Script
#
# Usage:
#   ./record_dataset.sh
#
# Operations:
#   1. Starts synchronized multi-modal recording (LiDAR, Camera, IMU, NDT Pose, CAN).
#   2. Displays live recording duration, file size, and frame count.
#   3. Press [Ctrl+C] to stop recording.
#   4. Automatically exports data to standard KITTI & PCD format ready for annotation.
# ====================================================================

source /opt/ros/noetic/setup.bash
source /home/infy/catkin_ws/devel/setup.bash 2>/dev/null
source /home/infy/connect_to_intel_ethernet.sh

exec python3 /home/infy/dataset_recorder/record_dataset.py "$@"
