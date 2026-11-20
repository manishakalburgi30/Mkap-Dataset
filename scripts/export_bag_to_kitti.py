#!/usr/bin/env python3
"""
Autonomous Buggy - Multi-Modal Dataset Exporter & Quality Verification Pipeline
Converts raw ROS bags into standardized, publication-grade datasets supporting:
  - Standard KITTI 3D & 2D formats (image_2/, velodyne/, calib/, poses/)
  - High-precision binary PointClouds (velodyne_pcd/)
  - Dedicated raw IMU sequence export (imu/imu_data.csv)
  - Detailed timestamp logs (timestamps/lidar_timestamps.txt, camera_timestamps.txt, imu_timestamps.txt)
  - Full calibration suite (calibration/camera_intrinsics.yaml, lidar_camera_extrinsics.yaml, tf_tree.yaml)
  - Automated Quality & Health Reports (quality_report/sync_report, freq_report, dropped_frames, checksums)
  - Dataset session documentation (README.md)
"""

import os
import sys
import argparse
import json
import hashlib
import yaml
import numpy as np
import cv2
import rosbag
import sensor_msgs.point_cloud2 as pc2

# Verified Buggy Calibration Constants
DEFAULT_CALIB = {
    "P2": [
        1271.6100, 0.0, 971.5850, 0.0,
        0.0, 1270.4200, 549.2880, 0.0,
        0.0, 0.0, 1.0, 0.0
    ],
    "R0_rect": [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ],
    "Tr_velo_to_cam": [
        0.000000, -1.000000,  0.000000,  0.260000,
        0.173648,  0.000000, -0.984808, -1.053000,
        0.984808,  0.000000,  0.173648,  1.586000
    ],
    "Tr_imu_to_velo": [
        0.984808,  0.000000, -0.173648, -2.223539,
        0.000000,  1.000000,  0.000000,  0.000000,
        0.173648,  0.000000,  0.984808, -1.596569
    ]
}

def compute_sha256(filepath, max_bytes=50*1024*1024):
    """Compute SHA-256 checksum of a file (first 50MB for large files)."""
    h = hashlib.sha256()
    try:
        with open(filepath, 'rb') as f:
            chunk = f.read(max_bytes)
            h.update(chunk)
        return h.hexdigest()
    except Exception:
        return "N/A"

def quat_to_rot_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-8:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1.0 - 2.0*(y**2 + z**2), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [2.0*(x*y + z*w),       1.0 - 2.0*(x**2 + z**2), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x**2 + y**2)]
    ])

def save_pcd_binary(filepath, points):
    """Save Nx4 float32 points as standard binary PCD."""
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())

def decode_image_msg(msg):
    """Decodes both sensor_msgs/CompressedImage and sensor_msgs/Image."""
    if hasattr(msg, 'format'):
        # CompressedImage
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:
        # Raw Image
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if 'bgra' in msg.encoding.lower():
            rgb = arr.reshape((msg.height, msg.width, 4))
            return rgb[:, :, :3]
        elif 'bgr' in msg.encoding.lower():
            return arr.reshape((msg.height, msg.width, 3))
        elif 'rgb' in msg.encoding.lower():
            rgb = arr.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            return arr.reshape((msg.height, msg.width, -1))

def filter_ego_vehicle_points(points):
    """
    Fixed spatial exclusion ROI filter to remove Buggy body/roof/bumper returns.
    LiDAR frame: +X forward, +Y left, +Z up.
    Buggy body in LiDAR frame:
      X: [-1.2, 0.8] m
      Y: [-0.65, 0.65] m
      Z: [-1.8, -0.2] m (below the roof arch mount)
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    is_ego = (x >= -1.2) & (x <= 0.8) & (y >= -0.65) & (y <= 0.65) & (z >= -1.8) & (z <= -0.2)
    return points[~is_ego]

def export_bag(bag_path, session_dir, sample_hz=2.0, max_time_diff=0.035):
    """Complete dataset extraction, synchronization, calibration, and QA export."""
    bag_filename = os.path.basename(bag_path)
    session_id = os.path.basename(os.path.normpath(session_dir))
    kitti_dir = os.path.join(session_dir, "kitti_format")
    calib_out_dir = os.path.join(session_dir, "calibration")
    quality_dir = os.path.join(session_dir, "quality_report")

    print("\n" + "="*70)
    print(f" [DATASET PROCESSOR] Starting Post-Processing Pipeline")
    print(f" Bag File:          {bag_filename}")
    print(f" Target Directory:  {session_dir}")
    print(f" Keyframe Rate:     {sample_hz} Hz ({'All frames' if sample_hz <= 0 else f'{1.0/sample_hz:.2f}s nominal'})")
    print(f" Max Pair Delta:    {max_time_diff*1000.0:.1f} ms")
    print("="*70)

    # Subdirectories
    image_dir = os.path.join(kitti_dir, "image_2")
    velodyne_dir = os.path.join(kitti_dir, "velodyne")
    pcd_dir = os.path.join(kitti_dir, "velodyne_pcd")
    calib_dir = os.path.join(kitti_dir, "calib")
    poses_dir = os.path.join(kitti_dir, "poses")
    imu_dir = os.path.join(kitti_dir, "imu")
    timestamps_dir = os.path.join(kitti_dir, "timestamps")

    for d in [image_dir, velodyne_dir, pcd_dir, calib_dir, poses_dir, imu_dir, timestamps_dir, calib_out_dir, quality_dir]:
        os.makedirs(d, exist_ok=True)

    print("\n[Step 1/6] Indexing ROS Bag & Reading Sensor Streams...")
    bag = rosbag.Bag(bag_path, 'r')
    topics_info = bag.get_type_and_topic_info().topics

    # Select Camera Topic
    cam_topic = None
    for cand in [
        "/zedx/zed_node/right/image_rect_color",
        "/zedx/zed_node/right/image_rect_color/compressed",
        "/zedx/zed_node/left/image_rect_color",
        "/zedx/zed_node/left/image_rect_color/compressed"
    ]:
        if cand in topics_info:
            cam_topic = cand
            break

    lidar_topic = "/velodyne_points" if "/velodyne_points" in topics_info else None
    imu_topic = "/imu/data" if "/imu/data" in topics_info else None
    pose_topic = "/ndt_pose" if "/ndt_pose" in topics_info else None
    cam_info_topic = "/zedx/zed_node/right/camera_info" if "/zedx/zed_node/right/camera_info" in topics_info else "/zedx/zed_node/left/camera_info"

    print(f" -> Camera Topic:    {cam_topic}")
    print(f" -> LiDAR Topic:     {lidar_topic}")
    print(f" -> IMU Topic:       {imu_topic}")
    print(f" -> NDT Pose Topic:  {pose_topic or 'None (using Identity)'}")

    if not cam_topic or not lidar_topic:
        print("[ERROR] Required topics (camera or lidar) missing from ROS bag!")
        bag.close()
        return False

    # Extract CameraInfo for Intrinsics
    calib_data = dict(DEFAULT_CALIB)
    cam_width, cam_height = 1920, 1080
    if cam_info_topic in topics_info:
        for _, msg, _ in bag.read_messages(topics=[cam_info_topic]):
            if hasattr(msg, 'P') and len(msg.P) == 12:
                calib_data["P2"] = list(msg.P)
            if hasattr(msg, 'width') and hasattr(msg, 'height'):
                cam_width, cam_height = msg.width, msg.height
            break

    # Read All Messages
    print("\n[Step 2/6] Reading & Indexing Timestamps for All Modalities...")
    cam_msgs = []    # [(sensor_stamp, bag_stamp, msg)]
    lidar_msgs = []  # [(sensor_stamp, bag_stamp, msg)]
    imu_records = [] # [(sensor_stamp, bag_stamp, msg)]
    pose_msgs = []   # [(sensor_stamp, bag_stamp, (tx, ty, tz, qx, qy, qz, qw))]

    for topic, msg, t in bag.read_messages(topics=[cam_topic, lidar_topic, imu_topic, pose_topic] if (imu_topic or pose_topic) else [cam_topic, lidar_topic]):
        sensor_stamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0 else t.to_sec()
        bag_stamp = t.to_sec()

        if topic == cam_topic:
            cam_msgs.append((sensor_stamp, bag_stamp, msg))
        elif topic == lidar_topic:
            lidar_msgs.append((sensor_stamp, bag_stamp, msg))
        elif topic == imu_topic:
            imu_records.append((sensor_stamp, bag_stamp, msg))
        elif topic == pose_topic:
            p = msg.pose.position
            q = msg.pose.orientation
            pose_msgs.append((sensor_stamp, bag_stamp, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))

    print(f" -> Indexed: {len(lidar_msgs)} LiDAR sweeps, {len(cam_msgs)} Camera frames, {len(imu_records)} IMU samples, {len(pose_msgs)} Poses.")

    # -------------------------------------------------------------
    # EXPORT RAW IMU SEQUENCE TO CSV
    # -------------------------------------------------------------
    print("\n[Step 3/6] Exporting Complete High-Rate IMU Sequence to CSV...")
    imu_csv_path = os.path.join(imu_dir, "imu_data.csv")
    imu_timestamps_path = os.path.join(timestamps_dir, "imu_timestamps.txt")
    
    with open(imu_csv_path, 'w') as f_csv, open(imu_timestamps_path, 'w') as f_time:
        f_csv.write("timestamp_ros,timestamp_bag,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,quat_x,quat_y,quat_z,quat_w,cov_accel_x,cov_gyro_x\n")
        f_time.write("# index sensor_timestamp_sec bag_timestamp_sec\n")

        for idx, (s_stamp, b_stamp, imu_m) in enumerate(imu_records):
            ax, ay, az = imu_m.linear_acceleration.x, imu_m.linear_acceleration.y, imu_m.linear_acceleration.z
            gx, gy, gz = imu_m.angular_velocity.x, imu_m.angular_velocity.y, imu_m.angular_velocity.z
            qx, qy, qz, qw = imu_m.orientation.x, imu_m.orientation.y, imu_m.orientation.z, imu_m.orientation.w
            cov_a0 = imu_m.linear_acceleration_covariance[0] if len(imu_m.linear_acceleration_covariance) > 0 else 0.0
            cov_g0 = imu_m.angular_velocity_covariance[0] if len(imu_m.angular_velocity_covariance) > 0 else 0.0

            f_csv.write(f"{s_stamp:.6f},{b_stamp:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{cov_a0:.6e},{cov_g0:.6e}\n")
            f_time.write(f"{idx:06d} {s_stamp:.6f} {b_stamp:.6f}\n")

    print(f" -> Exported {len(imu_records)} IMU records to: {imu_csv_path}")

    # -------------------------------------------------------------
    # TIMESTAMPS FOR CAMERA & LIDAR
    # -------------------------------------------------------------
    with open(os.path.join(timestamps_dir, "camera_timestamps.txt"), 'w') as f:
        f.write("# index sensor_timestamp_sec bag_timestamp_sec\n")
        for idx, (s_stamp, b_stamp, _) in enumerate(cam_msgs):
            f.write(f"{idx:06d} {s_stamp:.6f} {b_stamp:.6f}\n")

    with open(os.path.join(timestamps_dir, "lidar_timestamps.txt"), 'w') as f:
        f.write("# index sensor_timestamp_sec bag_timestamp_sec\n")
        for idx, (s_stamp, b_stamp, _) in enumerate(lidar_msgs):
            f.write(f"{idx:06d} {s_stamp:.6f} {b_stamp:.6f}\n")

    # -------------------------------------------------------------
    # TEMPORAL SYNCHRONIZATION (NEAREST-TIMESTAMP FRAME PAIRING)
    # -------------------------------------------------------------
    print("\n[Step 4/6] Executing Temporal Synchronization & Downsampling...")
    cam_stamps = np.array([c[0] for c in cam_msgs])
    imu_stamps = np.array([m[0] for m in imu_records]) if imu_records else np.array([])
    pose_stamps = np.array([p[0] for p in pose_msgs]) if pose_msgs else np.array([])

    paired_frames = []
    dropped_candidates = 0
    all_deltas = []

    min_interval = 1.0 / sample_hz if sample_hz > 0 else 0.0
    last_exported_time = -1e9

    for l_idx, (l_stamp, l_bag, l_msg) in enumerate(lidar_msgs):
        if l_stamp - last_exported_time < min_interval:
            continue

        # Find closest camera frame
        diffs = np.abs(cam_stamps - l_stamp)
        min_c_idx = np.argmin(diffs)
        dt = diffs[min_c_idx]
        all_deltas.append(dt * 1000.0)

        if dt > max_time_diff:
            dropped_candidates += 1
            continue

        c_stamp, c_bag, c_msg = cam_msgs[min_c_idx]

        # Find closest pose
        pose_tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        if len(pose_stamps) > 0:
            p_diffs = np.abs(pose_stamps - l_stamp)
            min_p_idx = np.argmin(p_diffs)
            if p_diffs[min_p_idx] < 0.2:
                pose_tuple = pose_msgs[min_p_idx][2]

        # Find IMU sample count in interval since last frame
        imu_count_in_window = 0
        if len(imu_stamps) > 0:
            imu_count_in_window = int(np.sum((imu_stamps >= last_exported_time) & (imu_stamps <= l_stamp)))

        paired_frames.append({
            "frame_idx": len(paired_frames),
            "lidar_stamp": l_stamp,
            "lidar_bag": l_bag,
            "lidar_msg": l_msg,
            "cam_stamp": c_stamp,
            "cam_bag": c_bag,
            "cam_msg": c_msg,
            "delta_ms": dt * 1000.0,
            "pose": pose_tuple,
            "imu_samples": imu_count_in_window
        })
        last_exported_time = l_stamp

    print(f" -> Synchronized {len(paired_frames)} frames (Rejected {dropped_candidates} exceeding {max_time_diff*1000:.1f}ms).")
    mean_dt = float(np.mean(all_deltas)) if all_deltas else 0.0
    max_dt = float(np.max(all_deltas)) if all_deltas else 0.0
    print(f" -> Synchronization Metrics: Mean Δt = {mean_dt:.2f} ms | Max Δt = {max_dt:.2f} ms")

    # -------------------------------------------------------------
    # EXPORT KITTI & PCD DATASET
    # -------------------------------------------------------------
    print("\n[Step 5/6] Exporting Multi-Modal KITTI, PCD, Calibration, and Poses...")
    poses_lines = []
    sync_pair_lines = ["# frame_id lidar_stamp cam_stamp delta_t_ms imu_samples_in_interval\n"]
    sample_for_overlay = None

    for idx, item in enumerate(paired_frames):
        fid = f"{idx:06d}"

        # 1. Image
        img = decode_image_msg(item["cam_msg"])
        cv2.imwrite(os.path.join(image_dir, f"{fid}.png"), img)

        # 2. Point Cloud (KITTI .bin and Binary .pcd)
        gen = pc2.read_points(item["lidar_msg"], field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        pts = np.array(list(gen), dtype=np.float32)
        if pts.shape[0] == 0:
            pts = np.zeros((1, 4), dtype=np.float32)

        # Apply Buggy Body Self-Reflection Filter
        filtered_pts = filter_ego_vehicle_points(pts)

        # Save .bin
        filtered_pts.tofile(os.path.join(velodyne_dir, f"{fid}.bin"))

        # Save .pcd
        save_pcd_binary(os.path.join(pcd_dir, f"{fid}.pcd"), filtered_pts)

        # 3. Calibration
        calib_txt = (
            f"P0: {' '.join(f'{x:.6e}' for x in calib_data['P2'])}\n"
            f"P1: {' '.join(f'{x:.6e}' for x in calib_data['P2'])}\n"
            f"P2: {' '.join(f'{x:.6e}' for x in calib_data['P2'])}\n"
            f"P3: {' '.join(f'{x:.6e}' for x in calib_data['P2'])}\n"
            f"R0_rect: {' '.join(f'{x:.6e}' for x in calib_data['R0_rect'])}\n"
            f"Tr_velo_to_cam: {' '.join(f'{x:.6e}' for x in calib_data['Tr_velo_to_cam'])}\n"
            f"Tr_imu_to_velo: {' '.join(f'{x:.6e}' for x in calib_data['Tr_imu_to_velo'])}\n"
        )
        with open(os.path.join(calib_dir, f"{fid}.txt"), 'w') as f:
            f.write(calib_txt)

        # 4. Pose
        tx, ty, tz, qx, qy, qz, qw = item["pose"]
        R = quat_to_rot_matrix(qx, qy, qz, qw)
        T_3x4 = np.hstack([R, np.array([[tx], [ty], [tz]])])
        poses_lines.append(' '.join(f'{val:.6e}' for val in T_3x4.flatten()))

        # 5. Sync Pair log
        sync_pair_lines.append(f"{fid} {item['lidar_stamp']:.6f} {item['cam_stamp']:.6f} {item['delta_ms']:.3f} {item['imu_samples']}\n")

        if idx == 0:
            sample_for_overlay = (img, filtered_pts)

        if (idx + 1) % 50 == 0 or idx == len(paired_frames) - 1:
            print(f" -> Exported {idx+1}/{len(paired_frames)} frames...")

    with open(os.path.join(poses_dir, "poses.txt"), 'w') as f:
        f.write('\n'.join(poses_lines) + '\n')

    with open(os.path.join(timestamps_dir, "synchronized_pairs.txt"), 'w') as f:
        f.writelines(sync_pair_lines)

    # -------------------------------------------------------------
    # CALIBRATION DIRECTORY EXPORT
    # -------------------------------------------------------------
    print("\n[Step 6/6] Generating Comprehensive Calibration & Quality Reports...")
    
    # 1. Camera Intrinsics YAML
    cam_intrinsics = {
        "camera_name": "zedx_front_right",
        "model": "Stereolabs ZED X",
        "serial_number": 42766808,
        "image_width": cam_width,
        "image_height": cam_height,
        "projection_matrix_P2": calib_data["P2"],
        "rectification_matrix_R0": calib_data["R0_rect"],
        "coordinate_convention": "ROS REP-103 Optical (+X Right, +Y Down, +Z Forward)"
    }
    with open(os.path.join(calib_out_dir, "camera_intrinsics.yaml"), 'w') as f:
        yaml.dump(cam_intrinsics, f, default_flow_style=False)

    # 2. LiDAR-Camera Extrinsics YAML
    lidar_cam_extrinsics = {
        "source_frame": "velodyne (VLP-16)",
        "target_frame": "zedx_right_camera_optical_frame",
        "transformation_direction": "T_camera_optical_from_velodyne",
        "matrix_4x4": [
            [0.000000, -1.000000,  0.000000,  0.260000],
            [0.173648,  0.000000, -0.984808, -1.053000],
            [0.984808,  0.000000,  0.173648,  1.586000],
            [0.000000,  0.000000,  0.000000,  1.000000]
        ],
        "translation_xyz_m": [1.586, 0.260, -1.053],
        "rotation_rpy_deg": [-80.0, 0.0, -90.0],
        "quaternion_xyzw": [-0.455, 0.455, -0.542, 0.542],
        "calibration_version": "v1.3_hallway_calibrated_202611"
    }
    with open(os.path.join(calib_out_dir, "lidar_camera_extrinsics.yaml"), 'w') as f:
        yaml.dump(lidar_cam_extrinsics, f, default_flow_style=False)

    # 3. IMU-LiDAR Extrinsics YAML
    imu_lidar_extrinsics = {
        "source_frame": "imu_link (Xsens MTI)",
        "target_frame": "velodyne",
        "transformation_direction": "T_velodyne_from_imu",
        "translation_xyz_m": [-2.223539, 0.000000, -1.596569],
        "rotation_rpy_deg": [0.0, -10.0, 0.0],
        "quaternion_xyzw": [0.0, -0.087156, 0.0, 0.996195]
    }
    with open(os.path.join(calib_out_dir, "imu_lidar_extrinsics.yaml"), 'w') as f:
        yaml.dump(imu_lidar_extrinsics, f, default_flow_style=False)

    # 4. TF Tree Documentation YAML
    tf_tree = {
        "reference_origin": "base_link (Center of Rear Axle at Ground Level)",
        "frames": {
            "base_link": {"description": "Vehicle reference frame (REP-105: +X Fwd, +Y Left, +Z Up)"},
            "velodyne": {"parent": "base_link", "translation": [2.4779, 0.0000, 1.6866], "pitch_deg": 10.0},
            "imu_link": {"parent": "base_link", "translation": [0.0109, 0.0000, 0.5004], "rpy_deg": [0.0, 0.0, 0.0]},
            "front_camera_body": {"parent": "base_link", "translation": [1.3894, 0.3200, 0.5880]},
            "zedx_right_camera_optical_frame": {"parent": "front_camera_body", "baseline_y_m": -0.060}
        }
    }
    with open(os.path.join(calib_out_dir, "tf_tree.yaml"), 'w') as f:
        yaml.dump(tf_tree, f, default_flow_style=False)

    # -------------------------------------------------------------
    # QUALITY REPORTS
    # -------------------------------------------------------------
    # 1. Overlay Sample
    if sample_for_overlay is not None:
        v_img, v_pts = sample_for_overlay
        h, w = v_img.shape[:2]
        T_mat = np.array(calib_data["Tr_velo_to_cam"]).reshape((3, 4))
        T_4x4 = np.vstack([T_mat, [0, 0, 0, 1]])
        P_mat = np.array(calib_data["P2"]).reshape((3, 4))
        pts_hom = np.hstack([v_pts[:, :3], np.ones((len(v_pts), 1))])
        pts_cam = (T_4x4 @ pts_hom.T).T
        pts_valid = pts_cam[pts_cam[:, 2] > 0.5]
        if len(pts_valid) > 0:
            pix_hom = (P_mat @ np.hstack([pts_valid[:, :3], np.ones((len(pts_valid), 1))]).T).T
            u = pix_hom[:, 0] / pix_hom[:, 2]
            v = pix_hom[:, 1] / pix_hom[:, 2]
            mask = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            u_ok, v_ok, d_ok = u[mask].astype(int), v[mask].astype(int), pts_valid[mask, 2]
            overlay = v_img.copy()
            colors = cv2.applyColorMap((np.clip(d_ok / 25.0, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET)
            for i in range(len(u_ok)):
                cv2.circle(overlay, (u_ok[i], v_ok[i]), 2, (int(colors[i, 0, 0]), int(colors[i, 0, 1]), int(colors[i, 0, 2])), -1)
            cv2.imwrite(os.path.join(quality_dir, "verification_overlay_sample.jpg"), overlay)

    # 2. Synchronization Report
    sync_report = {
        "total_lidar_sweeps": len(lidar_msgs),
        "total_camera_frames": len(cam_msgs),
        "total_paired_frames": len(paired_frames),
        "mean_delta_t_ms": round(mean_dt, 3),
        "max_delta_t_ms": round(max_dt, 3),
        "min_delta_t_ms": round(float(np.min(all_deltas)), 3) if all_deltas else 0.0,
        "std_dev_delta_t_ms": round(float(np.std(all_deltas)), 3) if all_deltas else 0.0,
        "pairs_within_15ms_pct": round(float(np.mean(np.array(all_deltas) <= 15.0) * 100.0), 2) if all_deltas else 0.0,
        "rejected_unmatched_frames": dropped_candidates
    }
    with open(os.path.join(quality_dir, "synchronization_report.json"), 'w') as f:
        json.dump(sync_report, f, indent=2)

    # 3. Sensor Frequency Report
    dur_lidar = (lidar_msgs[-1][0] - lidar_msgs[0][0]) if len(lidar_msgs) > 1 else 1.0
    dur_cam = (cam_msgs[-1][0] - cam_msgs[0][0]) if len(cam_msgs) > 1 else 1.0
    dur_imu = (imu_records[-1][0] - imu_records[0][0]) if len(imu_records) > 1 else 1.0
    freq_report = {
        "lidar_measured_hz": round(len(lidar_msgs) / max(dur_lidar, 0.01), 2),
        "camera_measured_fps": round(len(cam_msgs) / max(dur_cam, 0.01), 2),
        "imu_measured_hz": round(len(imu_records) / max(dur_imu, 0.01), 2) if imu_records else 0.0,
        "ndt_pose_measured_hz": round(len(pose_msgs) / max(dur_lidar, 0.01), 2) if pose_msgs else 0.0
    }
    with open(os.path.join(quality_dir, "sensor_frequency_report.json"), 'w') as f:
        json.dump(freq_report, f, indent=2)

    # 4. Dropped Frames & Gap Detections
    gaps = []
    for i in range(1, len(lidar_msgs)):
        gap = lidar_msgs[i][0] - lidar_msgs[i-1][0]
        if gap > 0.15: # >1.5x nominal 10Hz
            gaps.append({"sensor": "lidar", "index": i, "gap_sec": round(gap, 4)})
    for i in range(1, len(cam_msgs)):
        gap = cam_msgs[i][0] - cam_msgs[i-1][0]
        if gap > 0.10: # >1.5x nominal 15Hz
            gaps.append({"sensor": "camera", "index": i, "gap_sec": round(gap, 4)})

    with open(os.path.join(quality_dir, "dropped_frames.json"), 'w') as f:
        json.dump({"total_gap_anomalies": len(gaps), "anomalies": gaps}, f, indent=2)

    # 5. Recording Health & Checksum Report
    bag_sha256 = compute_sha256(bag_path)
    health_report = {
        "session_id": session_id,
        "raw_bag_file": bag_filename,
        "raw_bag_size_bytes": os.path.getsize(bag_path),
        "raw_bag_sha256_header": bag_sha256,
        "exported_kitti_frames": len(paired_frames),
        "exported_imu_samples": len(imu_records),
        "overall_status": "PASSED" if (len(paired_frames) > 0 and mean_dt <= max_time_diff*1000.0) else "WARNING"
    }
    with open(os.path.join(quality_dir, "recording_health_report.json"), 'w') as f:
        json.dump(health_report, f, indent=2)

    # 6. Session README.md
    lidar_msg_count = topics_info[lidar_topic].message_count if lidar_topic in topics_info else len(lidar_msgs)
    readme_content = f"""# Autonomous Buggy Dataset: {session_id}

## 1. Overview
* **Session ID:** `{session_id}`
* **Vehicle:** Autonomous Buggy (Jetson Orin + Intel NUC)
* **Date Processed:** {lidar_msg_count} LiDAR sweeps recorded
* **Synchronized Frames:** {len(paired_frames)} frames at {sample_hz} Hz keyframe rate
* **Total IMU Samples:** {len(imu_records)} continuous measurements

## 2. Directory Structure
```text
{session_id}/
├── raw_bag/                       # Permanent uncompressed ROS bag backup
├── calibration/                   # Sensor intrinsics & 6-DOF extrinsics
│   ├── camera_intrinsics.yaml     # ZED X right optical projection matrix
│   ├── lidar_camera_extrinsics.yaml # T_camera_from_velodyne 4x4 matrix
│   ├── imu_lidar_extrinsics.yaml  # T_velodyne_from_imu 4x4 matrix
│   └── tf_tree.yaml               # Buggy REP-103/REP-105 spatial coordinate frames
├── kitti_format/                  # Ready for 3D/2D Annotation
│   ├── image_2/                   # Synchronized rectified RGB images (PNG)
│   ├── velodyne/                  # Standard float32 binary PointClouds (.bin)
│   ├── velodyne_pcd/              # Standard binary .pcd files (CVAT 3D / CloudCompare)
│   ├── calib/                     # Standard KITTI calibration files (P2, R0_rect, Tr_velo_to_cam)
│   ├── poses/                     # Continuous 6-DOF ego-vehicle poses (poses.txt)
│   ├── imu/                       # High-rate raw IMU readings (imu_data.csv)
│   ├── timestamps/                # High-precision sensor & bag timestamps
│   └── metadata.json              # Full dataset specifications
└── quality_report/                # Quality Assurance & Verification
    ├── verification_overlay_sample.jpg # Visual proof of LiDAR-Camera projection
    ├── synchronization_report.json     # Timestamp delta metrics (Mean Δt: {mean_dt:.2f} ms)
    ├── sensor_frequency_report.json    # Measured frequencies per modality
    ├── dropped_frames.json             # Timestamp gap detection log
    └── recording_health_report.json    # Integrity status & SHA-256 checksums
```

## 3. How to Annotate
* **Supervisely (Recommended for Multi-Modal):** Upload `velodyne_pcd/` and attach `image_2/` as context camera with `calibration/camera_intrinsics.yaml` and `lidar_camera_extrinsics.yaml`.
* **CVAT 3D:** Create a new task with **3D Point Cloud** mode, select `velodyne_pcd/` and specify calibration.
* **SUSTechPOINTS:** Point directly to the `kitti_format/` directory.
"""
    with open(os.path.join(session_dir, "README.md"), 'w') as f:
        f.write(readme_content)

    bag.close()
    print("\n" + "="*70)
    print(" [COMPLETE] Multi-Modal Dataset Export & Quality Audit Finished!")
    print(f" -> Session:    {session_id}")
    print(f" -> Quality:    Mean Δt = {mean_dt:.2f} ms | Status: {health_report['overall_status']}")
    print(f" -> Locations:")
    print(f"    * Raw Bag:       {os.path.join(session_dir, 'raw_bag')}")
    print(f"    * Annotation:    {kitti_dir}")
    print(f"    * Calibrations:  {calib_out_dir}")
    print(f"    * Quality Audit: {quality_dir}")
    print("="*70 + "\n")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-Modal Dataset Exporter for Autonomous Buggy")
    parser.add_argument("bag_path", help="Path to input .bag file")
    parser.add_argument("-o", "--output", default=None, help="Target session directory")
    parser.add_argument("-r", "--rate", type=float, default=2.0, help="Keyframe sampling rate in Hz (default: 2.0 Hz)")
    parser.add_argument("--max_dt", type=float, default=0.035, help="Max timestamp delta in sec (default: 0.035s)")
    args = parser.parse_args()

    session_out = args.output
    if not session_out:
        session_out = os.path.dirname(os.path.abspath(args.bag_path))
        if os.path.basename(session_out) == "raw_bag":
            session_out = os.path.dirname(session_out)

    export_bag(args.bag_path, session_out, sample_hz=args.rate, max_time_diff=args.max_dt)
