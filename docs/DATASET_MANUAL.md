# Autonomous Buggy Multi-Modal Dataset Collection & Annotation Specification

**Vehicle:** Autonomous Buggy  
**Sensors:** Velodyne VLP-16 (3D LiDAR), Stereolabs ZED X (Stereo Vision), Xsens MTi-680G (IMU), SICK TiM 2D LiDARs, Wheel Encoders  
**Compute Architecture:** Intel NUC Master (`192.168.0.228:11311`) & NVIDIA Jetson Orin Front Compute (`192.168.0.244`)  
**Network Link:** Gigabit Ethernet (`eth0`) with Chrony Microsecond PTP/NTP Clock Sync  

---

## 1. Executive Summary & Research Methodology

The objective of this pipeline is to create a **synchronized, timestamped, calibrated, multi-modal autonomous vehicle perception dataset** that is directly compatible with downstream 3D/2D annotation platforms (e.g. Supervisely, CVAT 3D) and reusable across multiple autonomy domains:
* **3D Object Detection & 3D Tracking:** (PointPillars, CenterPoint, PV-RCNN)
* **Camera-LiDAR Multi-Modal Fusion:** (BEVFusion, TransFusion, PointPainting)
* **Visual-Inertial & LiDAR-Inertial Odometry:** (LIO-SAM, FAST-LIO, VINS-Mono)
* **Localization & High-Definition Mapping:** (NDT Matching, Cartographer)

### 9-Phase Dataset Lifecycle
```
Phase 1: Sensor Configuration & PTP Sync ────► Verified HW Clocks (<0.05ms)
Phase 2: Stage 1 Raw Data Acquisition   ────► High-Throughput ROS Bags (.bag)
Phase 3: Temporal Synchronization       ────► Nearest-Timestamp Matching (Δt < 20ms)
Phase 4: Multi-Modal Sensor Calibration ────► 6-DOF TF & Intrinsics Suite
Phase 5: Stage 2 Processing & Export    ────► KITTI .bin/.pcd, Images, IMU CSV
Phase 6: Quality Assurance & Audit      ────► QA Reports, Frequency & Checksums
Phase 7: Platform Annotation            ────► Supervisely / CVAT 3D Cuboids
Phase 8: Dataset Versioning & Release   ────► Open Format for Model Training
Phase 9: Downstream Benchmarking        ────► 3D Detection & Tracking Evaluation
```

---

## 2. Directory Structure: The Hybrid KITTI-IMU Standard

Standard KITTI was designed around cameras and LiDARs but omitted high-rate raw IMU sequences. We adopt a **hybrid KITTI + High-Rate Inertial + Quality Audit** structure where standard tools can ingest the `kitti_format/` folder directly without missing inertial and calibration telemetry:

```text
session_YYYYMMDD_HHMMSS/
├── raw_bag/
│   └── session_YYYYMMDD_HHMMSS.bag        # Permanent uncompressed ROS bag backup
├── session_info.yaml                      # High-level recording session metadata
├── calibration/                           # Complete Multi-Sensor Calibration Suite
│   ├── camera_intrinsics.yaml             # ZED X right optical projection & distortion
│   ├── lidar_camera_extrinsics.yaml       # T_camera_from_velodyne 4x4 matrix
│   ├── imu_lidar_extrinsics.yaml          # T_velodyne_from_imu 4x4 matrix
│   └── tf_tree.yaml                       # Buggy REP-103/REP-105 spatial coordinate frames
├── kitti_format/                          # Ready for 3D/2D Annotation
│   ├── image_2/                           # Rectified RGB images (000000.png, ...)
│   ├── velodyne/                          # Standard float32 binary PointClouds (.bin)
│   ├── velodyne_pcd/                      # Standard binary .pcd files (CVAT 3D / CloudCompare)
│   ├── calib/                             # KITTI calibration text files (P2, R0_rect, Tr_velo_to_cam)
│   ├── poses/
│   │   └── poses.txt                      # Continuous 6-DOF ego-vehicle trajectory
│   ├── imu/
│   │   └── imu_data.csv                   # Continuous 100 Hz raw IMU measurements
│   ├── timestamps/                        # Dual Timestamps (Sensor Header vs Bag Time)
│   │   ├── lidar_timestamps.txt           # Index, sensor_stamp_sec, bag_stamp_sec
│   │   ├── camera_timestamps.txt          # Index, sensor_stamp_sec, bag_stamp_sec
│   │   ├── imu_timestamps.txt             # Index, sensor_stamp_sec, bag_stamp_sec
│   │   └── synchronized_pairs.txt         # Paired frame IDs, delta_t, IMU count
│   └── metadata.json                      # Full dataset schema & export parameters
├── quality_report/                        # Automated Quality Assurance & Verification
│   ├── verification_overlay_sample.jpg    # Automatic projection proof of alignment
│   ├── synchronization_report.json        # Timestamp delta metrics (Mean, Max, StdDev)
│   ├── sensor_frequency_report.json       # Measured logging frequency per sensor
│   ├── dropped_frames.json                # Gap and message drop anomaly detections
│   └── recording_health_report.json       # SHA-256 checksums & integrity audit status
└── README.md                              # Standalone dataset documentation
```

---

## 3. Sensor Synchronization & Timestamping Methodology

### 3.1 Hardware Clock Synchronization
Both onboard computers are synchronized via Chrony over Gigabit Ethernet:
* **Grandmaster Clock:** Intel NUC (`192.168.0.228`)
* **Client Clock:** Jetson Orin (`192.168.0.244`)
* **Measured Offset:** $\mathbf{< 0.035\,\text{ms}}$ (sub-millisecond alignment).

### 3.2 Dual Timestamp Preservation
For every message, two timestamps are recorded:
1. `header.stamp`: Hardware sensor acquisition timestamp.
2. `bag_timestamp`: Arrival timestamp on the ROS logger.

### 3.3 Nearest-Timestamp Pairing Algorithm
1. **Reference Clock:** LiDAR sweeps `/velodyne_points` serve as the reference timebase.
2. **Nearest Neighbor Search:** For each LiDAR sweep at $t_L$, find the camera frame at $t_C$ minimizing $|\Delta t| = |t_L - t_C|$.
3. **Acceptance Threshold:** If $|\Delta t| \le 35\,\text{ms}$ (configurable via `--max_dt`), the pair is accepted. If $|\Delta t| > 35\,\text{ms}$, the candidate is rejected and logged in `quality_report/synchronization_report.json`.
4. **IMU Window Association:** Rather than downsampling the 100 Hz IMU to match the 2 Hz or 10 Hz LiDAR rate, the complete IMU stream is preserved in `imu/imu_data.csv`. The exact count and range of IMU samples between consecutive keyframes are logged in `timestamps/synchronized_pairs.txt`.

---

## 4. Multi-Sensor Calibration & Coordinate Conventions

### 4.1 Coordinate Standard Definitions (REP-103 & REP-105)
```
Vehicle Base Frame (base_link):
  +X: Forward (direction of travel)
  +Y: Left
  +Z: Upwards

3D LiDAR Frame (velodyne):
  +X: Forward
  +Y: Left
  +Z: Upwards (pitched +10.0 deg downwards relative to vehicle horizontal)

Camera Optical Frame (zedx_right_camera_optical_frame):
  +X: Rightwards (along image columns)
  +Y: Downwards (along image rows)
  +Z: Forward (optical axis into the scene)

IMU Frame (imu_link):
  +X: Forward, +Y: Left, +Z: Up
```

### 4.2 Verified 6-DOF Transformation Matrices

#### A. LiDAR $\to$ Right Camera Optical ($T_{\text{camera\_opt} \leftarrow \text{velodyne}}$)
*Direction:* Transforms a point $[X_L, Y_L, Z_L, 1]^T$ in Velodyne coordinates into Camera Optical coordinates $[X_C, Y_C, Z_C, 1]^T$:

$$\mathbf{T}_{\text{velo}}^{\text{cam\_opt}} = \begin{bmatrix}
0.000000 & -1.000000 & 0.000000 & \mathbf{0.260000} \\
0.173648 &  0.000000 & -0.984808 & \mathbf{-1.053000} \\
0.984808 &  0.000000 &  0.173648 & \mathbf{1.586000} \\
0.000000 &  0.000000 &  0.000000 & 1.000000
\end{bmatrix}$$

* **Translation Vector:** $[X = 1.586\,\text{m},\; Y = 0.260\,\text{m},\; Z = -1.053\,\text{m}]$
* **Euler Angles (RPY):** $[-80.000^\circ, 0.000^\circ, -90.000^\circ]$
* **Quaternion $[x, y, z, w]$:** $[-0.455, 0.455, -0.542, 0.542]$

#### B. IMU $\to$ LiDAR ($T_{\text{velodyne} \leftarrow \text{imu}}$)
* **Translation Vector:** $[-2.223539\,\text{m},\; 0.000000\,\text{m},\; -1.596569\,\text{m}]$
* **Euler Angles (RPY):** $[0.0^\circ, -10.0^\circ, 0.0^\circ]$

---

## 5. Raw IMU Sequence Export Specification

Raw IMU measurements are exported to `kitti_format/imu/imu_data.csv`:

```csv
timestamp_ros,timestamp_bag,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,quat_x,quat_y,quat_z,quat_w,cov_accel_x,cov_gyro_x
1795164885.430543,1795164885.431367,-0.131098,0.229060,9.753937,0.004140,-0.002263,0.000940,-0.010430,0.009516,0.968722,-0.247748,0.0,0.0
```

* **Linear Acceleration Units:** $\text{m/s}^2$ (including gravity vector $\approx 9.8\,\text{m/s}^2$ on $+Z$).
* **Angular Velocity Units:** $\text{rad/s}$.
* **Orientation:** Unit quaternion in ENU world frame.

---

## 6. Preprocessing & Self-Reflection Filtering

### Fixed Spatial Exclusion ROI Filter
To prevent the vehicle's own structure from introducing false obstacle returns, points falling within the Buggy's physical body box are removed during export:
$$\text{IsEgoVehicle} = (-1.2\,\text{m} \le X_L \le 0.8\,\text{m}) \land (-0.65\,\text{m} \le Y_L \le 0.65\,\text{m}) \land (-1.8\,\text{m} \le Z_L \le -0.2\,\text{m})$$
* **Raw Preservation:** The raw ROS bag retains 100% of unfiltered points; filtering is applied reproducibly during export.

---

## 7. Quality Assurance & Integrity Auditing

Every session generates four automated JSON reports under `quality_report/`:

1. **`synchronization_report.json`:**
   * Mean, median, max, and standard deviation of $|\Delta t|$.
   * Percentage of frames paired with $|\Delta t| \le 15\,\text{ms}$.
   * Count of rejected frames exceeding `--max_dt`.
2. **`sensor_frequency_report.json`:**
   * Measured operational rate for LiDAR (target 10 Hz), Camera (target 15 Hz), IMU (target 100 Hz), and NDT Pose (target 10 Hz).
3. **`dropped_frames.json`:**
   * Detects timestamp anomalies where the interval between consecutive messages exceeds $1.5\times$ nominal period ($> 150\,\text{ms}$ for LiDAR, $> 100\,\text{ms}$ for Camera).
4. **`recording_health_report.json`:**
   * SHA-256 cryptographic checksums for raw bag files to verify data integrity across network transfers.
   * Total message counts and overall session health status (`PASSED` / `WARNING`).

---

## 8. Annotation Tool Evaluation & Workflows

### 8.1 Supervisely (Recommended for Multi-Modal Sensor Fusion)
* **Format:** Multi-modal Point Cloud Episodes.
* **Why it fits:** Supervisely natively associates camera images with LiDAR sweeps, using camera projection matrices and extrinsics to project 3D cuboids directly into 2D camera views in real time.
* **Import Path:** Load `velodyne_pcd/` as point cloud episodes, attach `image_2/` as context camera, and specify `calibration/camera_intrinsics.yaml` and `lidar_camera_extrinsics.yaml`.

### 8.2 CVAT 3D (Recommended for Point Cloud Labeling)
* **Format:** Standard binary PCD (`velodyne_pcd/`) or float32 BIN (`velodyne/`).
* **Import Path:** Select **3D Point Cloud** task type, upload `.pcd` files, and provide `calib/` directory.

---

## 9. Operational Collection Protocol & Scenario Diversity

To build a generalizable dataset for research, data collection should follow an environmental matrix:

| Category | Target Scenarios | Priority |
| :--- | :--- | :--- |
| **Environments** | Indoor narrow hallway, wide atrium, outdoor asphalt road, parking area | High |
| **Dynamic Objects** | Pedestrians walking, crossing, standing; bicycles; moving buggies | High |
| **Static Objects** | Doorways, pillars, walls, traffic cones, signboards, trash cans | High |
| **Motion Profiles** | Straight acceleration ($0 \to 15\,\text{km/h}$), steady cruising, $90^\circ$ turns, sudden braking | Medium |
| **Lighting** | Bright morning, noon direct sunlight, indoor fluorescent, dusk/low light | Medium |

---

## 10. How to Run

### Start Recording:
```bash
./record_dataset.sh
```

### Stop & Auto-Export:
Press **`Ctrl+C`**. The script cleanly flushes the raw bag and automatically runs the full export and QA pipeline.

### Standalone Re-Exporting with Custom Parameters:
```bash
python3 /home/infy/dataset_recorder/export_bag_to_kitti.py \
    /home/infy/datasets/session_YYYYMMDD_HHMMSS/raw_bag/session.bag \
    -o /home/infy/datasets/session_YYYYMMDD_HHMMSS \
    -r 2.0 \
    --max_dt 0.035
```
