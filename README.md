# Mkap-Dataset: Autonomous Electric Buggy Multi-Modal Perception, Sensor Calibration & Standardized Dataset Suite

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![Hardware](https://img.shields.io/badge/Compute-NVIDIA%20Jetson%20Orin%20%2B%20Intel%20NUC-green.svg)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![Sync](https://img.shields.io/badge/Clock%20Sync-Chrony%20PTP%2FNTP%20(%3C0.05ms)-orange.svg)]()

An end-to-end multi-modal perception, real-time spatial calibration, and dataset collection platform developed for an Autonomous Electric Passenger Buggy operating in campus environments.

---

## 1. Problem Statement
Autonomous ground vehicles operating in crowded, low-speed campus environments encounter unstructured obstacles, mixed pedestrian-vehicle traffic, narrow pathways, and variable illumination. Achieving reliable 3D perception requires tightly coupled multi-modal sensor fusion across 3D LiDAR, stereo cameras, and inertial measurement units (IMUs). 

However, deploying such systems in real-world environments introduces critical challenges:
* **Spatial Calibration Errors:** Geometric misalignment between LiDAR and camera coordinate frames produces projection errors, making 3D bounding box projection onto 2D image planes impossible.
* **Temporal Desynchronization & Latency:** Sensor sampling rate discrepancies and inter-compute network latency introduce spatial jitter ($\delta \mathbf{p} \approx \mathbf{v} \cdot \Delta t$) under vehicle motion.
* **Lack of Standardized Open Datasets:** Most existing benchmark datasets (e.g., KITTI, Waymo, nuScenes) are collected with high-speed passenger cars on wide highways or city roads, lacking ground-truth benchmarks for narrow campus pedestrian corridors, low-lying obstacles, and tight turning radii.
* **Ego-Vehicle Self-Reflections:** LiDAR beams reflect off the vehicle's own roof frame and hood, creating phantom obstacle points that disrupt 3D object detectors.

---

## 2. Objectives
1. **Sensor Integration & Dual-Compute Synchronization:** Interconnect a heterogeneous sensor payload (Velodyne VLP-16 3D LiDAR, Stereolabs ZED X stereo camera, Xsens MTi-680G IMU, and SICK 2D safety LiDARs) across an Intel NUC and NVIDIA Jetson Orin with sub-millisecond clock synchronization ($< 0.05\,\text{ms}$) via Chrony PTP/NTP over Gigabit Ethernet.
2. **High-Precision 6-DOF Extrinsic Calibration:** Mathematically derive and empirically calibrate the spatial transformation matrix ($T_{\text{velo}}^{\text{cam\_opt}}$) between the 3D LiDAR and camera optical frame, verified via dense edge contouring.
3. **Elimination of Perception Latency:** Resolve multi-second inter-machine video transmission lag to achieve real-time, low-latency ($< 30\,\text{ms}$) frustum rendering and point cloud overlay.
4. **Standardized Hybrid KITTI-IMU Dataset Pipeline:** Implement an automated, two-stage data collection and processing suite (`record_dataset.sh`) that captures raw ROS bags and exports them into standard KITTI format (`.bin`), binary point clouds (`.pcd`), rectified images (`.png`), continuous $100\,\text{Hz}$ IMU telemetry (`.csv`), and NDT 6-DOF poses.
5. **Quality Assurance & Verification:** Automate dataset health checks including temporal synchronization histograms, sensor publish frequency tracking, frame-drop audits, and SHA-256 data integrity checksums.

---

## 3. Team Members
* **Manisha Kalburgi** – Research Lead & Developer ([GitHub: @manishakalburgi30](https://github.com/manishakalburgi30))
* **Autonomous Buggy Research & Engineering Team** – Robotics & Autonomous Systems Laboratory

---

## 4. Technologies & Tools Used

### Hardware Architecture
| Category | Component / Model | Specifications & Communication Interface |
| :--- | :--- | :--- |
| **Vehicle Platform** | Electric Passenger Buggy | Converted drive-by-wire (DbW) chassis with CAN motor and steering actuation |
| **Master Compute** | Intel NUC | Core i7, Ubuntu 18.04 LTS, Autoware.AI, NDT Localization, MPC Control (`192.168.0.228`) |
| **Front AI Compute** | NVIDIA Jetson Orin | 64GB / Ampere GPU, Ubuntu 20.04 (L4T), GMSL2 Deserializer, Dataset Exporter (`192.168.0.244`) |
| **3D LiDAR** | Velodyne Puck (VLP-16) | 16 channels, $360^\circ$ H-FOV, $\pm 15^\circ$ V-FOV, 100m range, $10\,\text{Hz}$ UDP Ethernet |
| **Stereo Camera** | Stereolabs ZED X | Dual global-shutter CMOS, GMSL2 coaxial interface, hardware sync, $960\times 600$ at $15-30\,\text{Hz}$ |
| **Inertial (IMU)** | Xsens MTi-680G | 9-axis industrial MEMS IMU, $100\,\text{Hz}$ linear acceleration & angular velocity via USB/serial |
| **2D Safety LiDARs**| SICK TiM 5xx (3 units) | Planar collision avoidance envelope (Front-Left, Front-Right, Rear-Center) |
| **Networking** | Gigabit Ethernet Switch | Dedicated inter-machine local bus with hardware-level PTP/NTP clock synchronization |

### Software & Frameworks
* **Robotics Middleware:** ROS Melodic / ROS Noetic
* **Autonomous Driving Stack:** Autoware.AI (NDT Matching, Ray Ground Filter, MPC Waypoint Follower)
* **Computer Vision & Math:** OpenCV, NumPy, SciPy, PCL (Point Cloud Library)
* **Time Synchronization:** Chrony (PTP / NTP Master-Slave daemon)
* **Annotation Ecosystem:** Supervisely 3D Point Cloud Tool, CVAT 3D, OpenPCDet

---

## 5. Current Project Progress

```
[Phase 1] Sensor Interfacing & Network Configuration ────► [COMPLETED]
[Phase 2] Clock Synchronization (< 0.05ms jitter)    ────► [COMPLETED]
[Phase 3] 6-DOF Camera-LiDAR Extrinsic Calibration   ────► [LOCKED & VERIFIED]
[Phase 4] Latency Diagnosis & Zero-Copy Optimization ────► [RESOLVED (< 30ms)]
[Phase 5] Two-Stage Dataset Recorder & Auto-Exporter ────► [COMPLETED & VERIFIED]
[Phase 6] Hybrid KITTI-IMU & QA Integrity Pipeline   ────► [COMPLETED]
[Phase 7] Academic Paper Technical Specification     ────► [COMPILED]
[Phase 8] Dynamic Campus Pilot Collection & Labeling ────► [IN PROGRESS]
```

### Detailed Accomplishments:
1. **Extrinsics Locked:** Calibrated transformation matrix from Velodyne VLP-16 to ZED X right optical frame:
   $$\mathbf{T}_{\text{velo}}^{\text{cam\_opt}} = \begin{bmatrix} 0.000 & -1.000 & 0.000 & 0.260 \\ 0.174 & 0.000 & -0.985 & -1.053 \\ 0.985 & 0.000 & 0.174 & 1.586 \\ 0.000 & 0.000 & 0.000 & 1.000 \end{bmatrix}$$
   * Translation: $[X=1.586\,\text{m}, Y=0.260\,\text{m}, Z=-1.053\,\text{m}]$
   * Roll/Pitch/Yaw: $[-80.0^\circ, 0.0^\circ, -90.0^\circ]$
   * Verification: Laser rings cleanly contour physical doorway frames, corners, and pedestrians without tilt or drift.
2. **Lag Resolved:** Eliminated a 3–4 second delay caused by TCP socket serialization, achieving direct, real-time ($< 30\,\text{ms}$) frustum rendering in RViz.
3. **Ego-Vehicle Filter:** Implemented 3D bounding box suppression $[-1.2 \le X \le 0.8] \times [-0.65 \le Y \le 0.65] \times [-1.8 \le Z \le -0.2]\,\text{m}$ to remove roof arch and bumper self-reflections.
4. **Automated Dataset Tooling:** Tested and verified `./record_dataset.sh` on live hardware, generating:
   * `kitti_format/velodyne/` (.bin) and `velodyne_pcd/` (.pcd)
   * `kitti_format/image_2/` (.png)
   * `kitti_format/imu/imu_data.csv` (100 Hz sequence with full covariances and quaternions)
   * `calibration/` standalone YAMLs and `quality_report/` audit reports with SHA-256 validation.

---

## 6. Repository Structure
```text
Autonomous-Buggy/
├── README.md                                  # Main project documentation
├── .gitignore                                 # Excludes large binaries, bags, and point clouds
├── config/                                    # Sensor calibration & TF configuration
│   ├── extrinsic_calibration.yaml             # 6-DOF Camera-LiDAR transformation
│   ├── camera_intrinsics.yaml                 # ZED X focal length, principal point, P2
│   ├── imu_lidar_extrinsics.yaml              # IMU to LiDAR spatial transform
│   └── tf_tree.yaml                           # System-wide static transform tree
├── scripts/                                   # Operational scripts and tooling
│   ├── record_dataset.sh                      # One-command interactive recording launcher
│   ├── record_dataset.py                      # Pre-flight check & high-throughput bag recorder
│   ├── export_bag_to_kitti.py                 # Multi-modal KITTI/PCD/IMU exporter & QA engine
│   └── start_front_camera.sh                  # ZED X driver startup script
└── docs/                                      # Research & operational manuals
    ├── DATASET_MANUAL.md                      # Operational guide for collection & labeling
    └── RESEARCH_PAPER_SPECIFICATION.md        # Comprehensive technical report for academic paper
```

---

## 7. Quick Start: Data Collection

### 1. Launch Interactive Recording
Run the master collection script from the root directory:
```bash
./scripts/record_dataset.sh
```
* Runs automated pre-flight checks on ROS Master, LiDAR, IMU, and Camera.
* Displays live recording status (duration, bag file size in MB).

### 2. Stop & Export
Press `Ctrl+C` to end recording. The system will:
1. Flush and close the raw ROS bag in `raw_bag/`.
2. Automatically trigger Stage 2 export at $2.0\,\text{Hz}$.
3. Generate standard KITTI formats, binary `.pcd` files, 100 Hz IMU CSV, and QA verification reports.

---

## 8. License
This project is open-sourced under the MIT License - see the LICENSE file for details.
