#!/usr/bin/env python3
"""
Autonomous Buggy Multi-Modal Dataset Recorder & Auto-Exporter
Interactive tool: Run this script to start recording. Press Ctrl+C to stop.
On stop, it cleanly finishes the ROS bag and automatically exports the data
into standard KITTI & PCD format ready for 3D/2D annotation tools.
"""

import os
import sys
import time
import signal
import subprocess
import shutil
import datetime
import yaml

# Required topic list for full multi-modal annotation
RECORD_TOPICS = [
    "/velodyne_points",
    "/zedx/zed_node/right/image_rect_color/compressed",
    "/zedx/zed_node/right/image_rect_color",
    "/zedx/zed_node/right/camera_info",
    "/imu/data",
    "/ndt_pose",
    "/vehicle_speed",
    "/currentSteerAngle",
    "/tf",
    "/tf_static",
    "/frontLeft/sick_tim_5xx",
    "/frontRight/sick_tim_5xx",
    "/backCenter/sick_tim_5xx"
]

def check_env():
    """Verify ROS environment and connection."""
    master = os.environ.get("ROS_MASTER_URI", "http://192.168.0.228:11311")
    os.environ["ROS_MASTER_URI"] = master
    os.environ["ROS_IP"] = os.environ.get("ROS_IP", "192.168.0.244")

    # Check connection to master
    try:
        res = subprocess.run(["rostopic", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4)
        if res.returncode != 0:
            print(f"[ERROR] Cannot connect to ROS Master at {master}")
            return False, []
        active_topics = res.stdout.decode().splitlines()
        return True, active_topics
    except Exception as e:
        print(f"[ERROR] ROS environment check failed: {e}")
        return False, []

def check_disk_space(path="/home/infy", min_gb=5):
    """Check available disk space."""
    stat = shutil.disk_usage(path)
    free_gb = stat.free / (1024**3)
    if free_gb < min_gb:
        print(f"[WARNING] Low disk space on {path}: {free_gb:.1f} GB free!")
    return free_gb

def main():
    print("===========================================================")
    print("      AUTONOMOUS BUGGY DATASET COLLECTION SUITE            ")
    print("===========================================================")

    # 1. Pre-flight check
    print("\n[1/4] Checking ROS Master & Sensor Status...")
    ok, active_topics = check_env()
    if not ok:
        print("[!] Sourcing ROS setup...")
        subprocess.run(["bash", "-c", "source /opt/ros/noetic/setup.bash && source /home/infy/connect_to_intel_ethernet.sh"])
        ok, active_topics = check_env()
        if not ok:
            print("[ERROR] Failed to reach ROS Master. Check network cable and IP 192.168.0.228.")
            sys.exit(1)

    print(f" -> ROS Master Connected: {os.environ.get('ROS_MASTER_URI')}")
    print(f" -> Local ROS IP:         {os.environ.get('ROS_IP')}")

    # Check LiDAR
    has_lidar = "/velodyne_points" in active_topics
    print(f" -> 3D LiDAR (/velodyne_points):     [{'OK' if has_lidar else 'OFFLINE'}]")

    # Check Camera
    has_cam = any("zedx" in t for t in active_topics)
    print(f" -> ZED X Camera (/zedx/...):        [{'OK' if has_cam else 'OFFLINE'}]")

    # If camera is offline, start it
    if not has_cam:
        print(" -> Camera is offline. Launching ZED X Camera node in background...")
        cam_start_script = "/home/infy/start_front_camera.sh"
        if os.path.exists(cam_start_script):
            subprocess.Popen(["bash", cam_start_script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(" -> Waiting for ZED X Camera topics to become active (up to 12s)...")
            for _ in range(12):
                time.sleep(1.0)
                _, active_topics = check_env()
                has_cam = any("zedx" in t for t in active_topics)
                if has_cam:
                    break
            print(f" -> ZED X Camera Status:             [{'OK' if has_cam else 'WARNING: Failed to start'}]")

    # Check Pose & IMU
    has_pose = "/ndt_pose" in active_topics
    has_imu = "/imu/data" in active_topics
    print(f" -> Ego Pose (/ndt_pose):             [{'OK' if has_pose else 'OFFLINE'}]")
    print(f" -> IMU (/imu/data):                  [{'OK' if has_imu else 'OFFLINE'}]")

    # Disk Space
    free_gb = check_disk_space("/home/infy")
    print(f" -> Available Disk Space:            {free_gb:.1f} GB (~{int(free_gb * 4)} minutes recording capacity)")

    # 2. Setup Session Directory
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_id = f"session_{timestamp_str}"
    session_dir = os.path.join("/home/infy/datasets", session_id)
    raw_bag_dir = os.path.join(session_dir, "raw_bag")
    kitti_dir = os.path.join(session_dir, "kitti_format")
    os.makedirs(raw_bag_dir, exist_ok=True)
    os.makedirs(kitti_dir, exist_ok=True)

    bag_base = os.path.join(raw_bag_dir, session_id)
    bag_path = f"{bag_base}.bag"

    # Filter active topics to record
    topics_to_record = [t for t in RECORD_TOPICS if t in active_topics or t in ["/tf", "/tf_static"]]
    # Always include core topics even if latched
    for core in ["/velodyne_points", "/zedx/zed_node/right/camera_info"]:
        if core not in topics_to_record and core in active_topics:
            topics_to_record.append(core)

    print(f"\n[2/4] Initialized Session:")
    print(f" -> Session Folder: {session_dir}")
    print(f" -> Raw Bag Output: {bag_path}")
    print(f" -> Topics to record ({len(topics_to_record)}):")
    for t in topics_to_record:
        print(f"    * {t}")

    # Write session metadata
    session_info = {
        "session_id": session_id,
        "date": datetime.datetime.now().isoformat(),
        "vehicle": "Autonomous Buggy",
        "master_uri": os.environ.get("ROS_MASTER_URI"),
        "topics_recorded": topics_to_record,
        "calibrated_extrinsics": {
            "Tr_velo_to_cam": [
                0.000000, -1.000000,  0.000000,  0.260000,
                0.173648,  0.000000, -0.984808, -1.053000,
                0.984808,  0.000000,  0.173648,  1.586000
            ],
            "quaternion_xyzw": [-0.455, 0.455, -0.542, 0.542],
            "rpy_deg": [-80.0, 0.0, -90.0]
        }
    }
    with open(os.path.join(session_dir, "session_info.yaml"), 'w') as f:
        yaml.dump(session_info, f, default_flow_style=False)

    # 3. Start Recording
    print("\n[3/4] STARTING DATASET RECORDING...")
    print("-----------------------------------------------------------")
    print(" >>> RECORDING IN PROGRESS <<<")
    print(" >>> DRIVE THE BUGGY NORMALLY. PRESS [Ctrl+C] TO FINISH. <<<")
    print("-----------------------------------------------------------")

    cmd = ["rosbag", "record", "-b", "2048", "-O", bag_path] + topics_to_record

    record_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    start_time = time.time()
    try:
        while True:
            elapsed = time.time() - start_time
            mins, secs = divmod(int(elapsed), 60)
            size_mb = 0
            if os.path.exists(bag_path):
                size_mb = os.path.getsize(bag_path) / (1024 * 1024)
            elif os.path.exists(bag_path + ".active"):
                size_mb = os.path.getsize(bag_path + ".active") / (1024 * 1024)

            status_line = f"\r [RECORDING] ⏱️  {mins:02d}:{secs:02d} | 📦 Size: {size_mb:.1f} MB | Press Ctrl+C to Stop & Export "
            sys.stdout.write(status_line)
            sys.stdout.flush()
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n\n[!] Stopping recording gracefully...")
        # Send SIGINT to the rosbag process group so it flushes and closes cleanly
        os.killpg(os.getpgid(record_proc.pid), signal.SIGINT)
        record_proc.wait(timeout=15)
        print(" -> Raw ROS bag flushed and closed successfully.")

    # 4. Automatic KITTI / Annotation Export
    print("\n[4/4] EXPORTING DATASET TO ANNOTATION FORMAT...")
    # Find actual bag name
    final_bag = None
    if os.path.exists(bag_path):
        final_bag = bag_path
    elif os.path.exists(bag_base + ".bag"):
        final_bag = bag_base + ".bag"
    elif os.path.exists(bag_path + ".bag"):
        final_bag = bag_path + ".bag"
        # normalize to bag_path
        shutil.move(final_bag, bag_path)
        final_bag = bag_path
    elif os.path.exists(bag_path + ".active"):
        shutil.move(bag_path + ".active", bag_path)
        final_bag = bag_path

    exporter_script = "/home/infy/dataset_recorder/export_bag_to_kitti.py"
    if os.path.exists(final_bag) and os.path.exists(exporter_script):
        # Run export at 2.0 Hz sampling rate (ideal for annotation)
        cmd_export = [
            "python3", exporter_script,
            final_bag,
            "-o", session_dir,
            "-r", "2.0",
            "--max_dt", "0.035"
        ]
        res = subprocess.run(cmd_export)
        if res.returncode == 0:
            print("\n===========================================================")
            print("             DATASET COLLECTION COMPLETE!                  ")
            print("===========================================================")
            print(f" -> Dataset Session ID: {session_id}")
            print(f" -> Raw Bag File:       {final_bag}")
            print(f" -> Session Directory:  {session_dir}")
            print("    ├── raw_bag/             (Raw ROS bag permanent backup)")
            print("    ├── calibration/         (YAML intrinsics, extrinsics, TF tree)")
            print("    │   ├── camera_intrinsics.yaml")
            print("    │   ├── lidar_camera_extrinsics.yaml")
            print("    │   ├── imu_lidar_extrinsics.yaml")
            print("    │   └── tf_tree.yaml")
            print("    ├── kitti_format/        (Ready for Annotation)")
            print("    │   ├── image_2/         (Synchronized RGB frames)")
            print("    │   ├── velodyne/        (Float32 .bin PointClouds)")
            print("    │   ├── velodyne_pcd/    (Binary .pcd for CVAT/CloudCompare)")
            print("    │   ├── calib/           (Standard KITTI .txt calib matrices)")
            print("    │   ├── poses/           (6-DOF poses: poses.txt)")
            print("    │   ├── imu/             (Raw high-rate IMU: imu_data.csv)")
            print("    │   ├── timestamps/      (lidar, camera, imu, sync_pairs)")
            print("    │   └── metadata.json    (Full dataset specifications)")
            print("    ├── quality_report/      (Quality Assurance & Auditing)")
            print("    │   ├── verification_overlay_sample.jpg")
            print("    │   ├── synchronization_report.json")
            print("    │   ├── sensor_frequency_report.json")
            print("    │   ├── dropped_frames.json")
            print("    │   └── recording_health_report.json")
            print("    └── README.md            (Dataset session overview & usage)")
            print("===========================================================")
        else:
            print("[WARNING] Auto-export encountered an issue. Raw bag is safe in:")
            print(f"  {final_bag}")
    else:
        print(f"[!] Bag file: {final_bag}")

if __name__ == "__main__":
    main()
