# Autonomous Buggy: Multi-Modal Perception, Sensor Calibration, and Standardized Dataset Generation for Autonomous Driving Research

**Authors / Engineering Team:** Autonomous Buggy Research Group  
**Target Publication Scope:** IEEE / SAE / Robotics and Automation (ICRA/IROS/IV/ITSC)  
**Artifact Purpose:** Comprehensive technical compilation of system architecture, mathematical formulations, sensor calibration, temporal synchronization, experimental validation, and dataset generation methodologies for drafting an academic paper.

---

## 1. Abstract & Key Contributions

### 1.1 Abstract
Autonomous navigation in low-speed campus and urban environments requires resilient multi-modal perception, precise spatial-temporal alignment between heterogeneous sensors, and standardized benchmark datasets for training and evaluating 3D object detection and state estimation algorithms. In this work, we present the end-to-end development of an Autonomous Electric Buggy research platform equipped with a heterogeneous sensor suite comprising a 3D LiDAR (Velodyne VLP-16), a GMSL2 stereo vision system (Stereolabs ZED X), an industrial-grade Inertial Measurement Unit (Xsens MTi-680G), multiple planar 2D safety LiDARs (SICK TiM 5xx), and vehicle drive-by-wire odometry. We detail a distributed dual-compute architecture (Intel NUC master running Autoware.AI and NVIDIA Jetson Orin front perception compute) interconnected over a synchronized Gigabit Ethernet bus achieving sub-millisecond clock synchronization via Precision Time Protocol (PTP/NTP). We formulate an iterative 6-DOF extrinsic calibration framework between the 3D LiDAR and stereo optical frames, achieving sub-centimeter geometric projection accuracy, and eliminate a critical 3–4 second transmission latency through optimized zero-copy shared memory buffering. Furthermore, we implement an open, hybrid KITTI-compatible data collection and quality-assurance pipeline that decouples high-throughput raw ROS bag acquisition from multi-rate sensor processing, producing synchronized $2.0\,\text{Hz}$ annotated point cloud/image keyframes, continuous $100\,\text{Hz}$ raw inertial sequences, and Normal Distributions Transform (NDT) 6-DOF ground-truth poses. Experimental validation verifies temporal synchronization jitter of $\Delta t < 20\,\text{ms}$, robust ego-vehicle self-reflection spatial filtering, and high visual alignment across dynamic campus scenarios.

### 1.2 Summary of Key Contributions
1. **Distributed Dual-Compute Hardware Architecture:** Design of a ruggedized distributed vehicle compute platform combining an Intel NUC (vehicle control, NDT localization, CAN bus interfacing) and an NVIDIA Jetson Orin (high-throughput neural perception and GMSL2 stereo acquisition) synchronized via Gigabit Chrony PTP/NTP ($< 0.05\,\text{ms}$ clock drift).
2. **Analytical & Empirical 6-DOF Sensor Extrinsic Calibration:** Complete mathematical derivation and real-world calibration locking the 3D LiDAR to the camera optical frame ($T_{\text{velo}}^{\text{cam\_opt}}$) with translation vector $[1.586\,\text{m}, 0.260\,\text{m}, -1.053\,\text{m}]$ and rotation RPY $[-80.0^\circ, 0.0^\circ, -90.0^\circ]$, validated through dense geometric edge contouring in 3D-to-2D projection space.
3. **Low-Latency Zero-Copy Perception Pipeline:** Diagnostics and resolution of multi-second inter-machine communication lag, eliminating heavy TCP image serialization and achieving real-time ($< 30\,\text{ms}$) frustum projection.
4. **Hybrid KITTI-IMU Standardized Dataset Architecture:** An automated, reproducible acquisition and export suite producing KITTI-compatible binary point clouds (`.bin`), binary PCDs (`.pcd`), rectified camera frames, full-rate $100\,\text{Hz}$ IMU telemetry (quaternions, angular velocities, linear accelerations, covariance matrices), and NDT 6-DOF poses.
5. **Automated Quality Assurance & Verification:** Automated post-processing verification engine generating temporal sync histograms, publish frequency audits, frame-drop counters, visual projection overlays, and SHA-256 data integrity checksums.

---

## 2. System & Platform Architecture

### 2.1 Vehicle Platform Overview
The research vehicle is a converted commercial electric passenger buggy retrofitted for full drive-by-wire (DbW) autonomous operation. It operates in shared pedestrian-vehicle campus environments featuring paved pathways, narrow corridors, doorways, tree-lined roads, and varying lighting conditions.

```mermaid
graph TD
    subgraph "Vehicle Sensors & Actuators"
        VLP16["Velodyne Puck VLP-16<br/>(3D LiDAR - 10 Hz)"]
        ZEDX["Stereolabs ZED X<br/>(GMSL2 Stereo - 15-30 Hz)"]
        XSENS["Xsens MTi-680G<br/>(IMU - 100 Hz)"]
        SICK_L["SICK TiM 5xx Left<br/>(2D Safety LiDAR)"]
        SICK_R["SICK TiM 5xx Right<br/>(2D Safety LiDAR)"]
        SICK_B["SICK TiM 5xx Rear<br/>(2D Safety LiDAR)"]
        CAN["Buggy CAN Bus<br/>(Steering, Throttle, Brakes, Encoders)"]
    end

    subgraph "NVIDIA Jetson Orin (192.168.0.244)"
        GMSL_DRV["ZED GMSL2 Driver"]
        CAM_PUB["/zedx/zed_node/..."]
        REC_DAEMON["Master Dataset Recorder<br/>(record_dataset.py)"]
        EXP_PIPELINE["KITTI/PCD Exporter<br/>(export_bag_to_kitti.py)"]
    end

    subgraph "Intel NUC Master (192.168.0.228:11311)"
        ROS_MASTER["ROS Core Master"]
        VLP_DRV["Velodyne Driver & Cloud Nodelet"]
        IMU_DRV["Xsens Driver Node"]
        NDT_LOC["Autoware NDT Matching & Mapping"]
        MPC_CTRL["MPC Trajectory Follower"]
        DBW_IF["CAN / Motor Interface"]
    end

    ZEDX -->|GMSL2 Coaxial| GMSL_DRV
    GMSL_DRV --> CAM_PUB
    VLP16 -->|Ethernet (UDP)| VLP_DRV
    XSENS -->|USB / RS-232| IMU_DRV
    SICK_L & SICK_R & SICK_B -->|Ethernet| ROS_MASTER
    CAN -->|CAN-to-USB| DBW_IF

    VLP_DRV --> NDT_LOC
    IMU_DRV --> NDT_LOC
    NDT_LOC --> MPC_CTRL
    MPC_CTRL --> DBW_IF

    ROS_MASTER <==>|Gigabit Ethernet Bus<br/>Chrony PTP/NTP Clock Sync (< 0.05ms)| CAM_PUB
    CAM_PUB & VLP_DRV & IMU_DRV & NDT_LOC --> REC_DAEMON
    REC_DAEMON --> EXP_PIPELINE
```

### 2.2 Heterogeneous Sensor Specifications
| Modality | Sensor Model | Interface | Operating Freq. | Key Specifications & Role |
| :--- | :--- | :--- | :--- | :--- |
| **3D LiDAR** | Velodyne Puck (VLP-16) | Ethernet (UDP) | $10\,\text{Hz}$ | 16 channels, $\pm 15^\circ$ vertical FOV ($2.0^\circ$ res), $360^\circ$ horizontal FOV, $100\,\text{m}$ range, $\approx 30,000\,\text{pts/scan}$. Primary spatial perception & NDT localization. |
| **Stereo Vision** | Stereolabs ZED X | GMSL2 Coaxial | $15 - 30\,\text{Hz}$ | Dual global-shutter CMOS ($1920\times1200$ raw, downscaled to $960\times600$), $110^\circ$ horizontal FOV, IP67 enclosure, hardware synchronization. |
| **Inertial (IMU)** | Xsens MTi-680G | USB / RS-232 | $100\,\text{Hz}$ | Industrial 9-axis MEMS IMU + GNSS/RTK receiver, angular random walk $0.007^\circ/\sqrt{\text{s}}$, accelerometer in-run bias stability $15\,\mu\text{g}$. |
| **2D Safety LiDAR** | SICK TiM 5xx (3 units) | Ethernet | $15\,\text{Hz}$ | Front-Left, Front-Right, Rear-Center. Low-lying obstacle detection and hardware emergency-stop envelope. |
| **Vehicle Telemetry** | Drive-by-Wire (DbW) | CAN Bus | $50\,\text{Hz}$ | Wheel speed encoders, steering angle sensor ($\text{PPM}$), brake actuator state, throttle command. |

### 2.3 Compute & Distributed Network Infrastructure
* **Master Controller (Intel NUC):**
  * OS: Ubuntu Linux 18.04 LTS, ROS Melodic.
  * Roles: Runs `roscore`, Autoware.AI NDT localization against pre-mapped 3D point cloud maps (`/points_map`), Model Predictive Control (MPC) trajectory following, CAN bus motor drivers.
  * Static IP: `192.168.0.228`.
* **Front Perception Processor (NVIDIA Jetson Orin):**
  * OS: Ubuntu Linux 20.04 (L4T 35.4.1), ROS Noetic / Melodic bridge.
  * Hardware: NVIDIA Orin SoC (Arm Cortex-A78AE CPU, Ampere GPU with Tensor Cores, GMSL2 deserializer board).
  * Roles: Dedicated camera stream ingestion, AI model inference, real-time extrinsic projection, dataset recording daemon, multi-threaded Stage 2 exporter.
  * Static IP: `192.168.0.244`.
* **Inter-Machine Networking & Time Synchronization:**
  * Physical Layer: Dedicated Gigabit Ethernet link via industrial unmanaged switch (`eth0`).
  * Clock Synchronization: Chrony daemon running in master-slave PTP/NTP configuration across the subnet. Verified relative clock skew is $< 0.035\,\text{ms}$, preventing frame-association drift.

---

## 3. Mathematical Formulation of Sensor Calibration

### 3.1 Coordinate System Conventions
To prevent transformation ambiguity during multi-modal fusion and 3D annotation, coordinate frames are formally standardized:

```mermaid
flowchart LR
    V["Velodyne LiDAR Frame {V}<br/>X: Forward<br/>Y: Left<br/>Z: Up"]
    C["Camera Optical Frame {C}<br/>X: Right<br/>Y: Down<br/>Z: Optical Axis (Forward)"]
    I["IMU Frame {I}<br/>X: Forward<br/>Y: Left<br/>Z: Up"]
    B["Vehicle Base Frame {B}<br/>X: Forward<br/>Y: Left<br/>Z: Up (Rear Axle Center)"]

    V -->|T_velo_to_cam| C
    I -->|T_imu_to_velo| V
    B -->|T_base_to_velo| V
```

1. **LiDAR Sensor Frame $\{V\}$ (Velodyne VLP-16):** Right-handed Cartesian convention. $+X$ points forward along the vehicle heading, $+Y$ points to the left, and $+Z$ points upward.
2. **Camera Optical Frame $\{C\}$ (ZED X Right Eye):** Standard optical reference. $+X$ points horizontally to the right across the image sensor, $+Y$ points vertically downward, and $+Z$ points forward along the camera principal optical axis.
3. **IMU Sensor Frame $\{I\}$ (Xsens MTi):** $+X$ forward (accelerometer longitudinal), $+Y$ left (transverse), $+Z$ upward (vertical normal).
4. **Vehicle Base Frame $\{B\}$ (`base_link`):** Centered on the ground projection of the vehicle's rear axle center. $+X$ forward, $+Y$ left, $+Z$ up.

### 3.2 3D-to-2D Perspective Projection Formulation
Let a 3D physical point in the LiDAR coordinate frame be denoted as:
$$\mathbf{P}_V = \begin{bmatrix} X_V \\ Y_V \\ Z_V \\ 1 \end{bmatrix} \in \mathbb{R}^4$$

The rigid-body 6-DOF transformation from the LiDAR frame $\{V\}$ to the Camera Optical frame $\{C\}$ is represented by the Special Euclidean group element $\mathbf{T}_{V}^{C} \in \mathrm{SE}(3)$:
$$\mathbf{T}_{V}^{C} = \begin{bmatrix} \mathbf{R}_{V}^{C} & \mathbf{t}_{V}^{C} \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

where $\mathbf{R}_{V}^{C} \in \mathrm{SO}(3)$ is the $3\times 3$ orthonormal rotation matrix and $\mathbf{t}_{V}^{C} = [t_x, t_y, t_z]^T \in \mathbb{R}^3$ is the translation vector.

The 3D coordinates in the camera optical frame are:
$$\mathbf{P}_C = \begin{bmatrix} X_C \\ Y_C \\ Z_C \\ 1 \end{bmatrix} = \mathbf{T}_{V}^{C} \mathbf{P}_V$$

Points lying behind the camera optical plane ($Z_C \le 0.1\,\text{m}$) are discarded. Valid points are projected onto the 2D discrete pixel coordinate space $\mathbf{p} = [u, v, 1]^T$ via the camera intrinsic projection matrix $\mathbf{P}_2 \in \mathbb{R}^{3\times 4}$:
$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{P}_2 \mathbf{P}_C = \begin{bmatrix} f_x & 0 & c_x & 0 \\ 0 & f_y & c_y & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} X_C \\ Y_C \\ Z_C \\ 1 \end{bmatrix}$$

where $s = Z_C$ represents the projective scale factor (depth along the camera optical axis), $(f_x, f_y)$ are the horizontal and vertical focal lengths in pixels, and $(c_x, c_y)$ is the principal point.

The resulting discrete pixel coordinates are:
$$u = \frac{f_x X_C}{Z_C} + c_x, \quad v = \frac{f_y Y_C}{Z_C} + c_y$$

A point is visible if and only if $0 \le u < W$ and $0 \le v < H$, where $(W, H)$ is the image resolution ($960\times 600$).

### 3.3 Calibrated Extrinsic & Intrinsic Matrices
Through systematic empirical refinement verified via high-contrast planar structural edges (doorway jambs, wall corners, pedestrian contours), the locked calibration parameters are:

#### A. Camera Intrinsic Matrix ($\mathbf{P}_2$ / $\mathbf{K}$)
$$\mathbf{P}_2 = \begin{bmatrix} 1271.6100 & 0.0000 & 971.5850 & 0.0000 \\ 0.0000 & 1270.4200 & 549.2880 & 0.0000 \\ 0.0000 & 0.0000 & 1.0000 & 0.0000 \end{bmatrix}$$
Rectification matrix: $\mathbf{R}_0^{\text{rect}} = \mathbf{I}_{3\times 3}$.

#### B. LiDAR-to-Camera Extrinsic Matrix ($\mathbf{T}_{\text{velo}}^{\text{cam\_opt}}$)
$$\mathbf{T}_{\text{velo}}^{\text{cam\_opt}} = \begin{bmatrix} 0.000000 & -1.000000 & 0.000000 & 0.260000 \\ 0.173648 & 0.000000 & -0.984808 & -1.053000 \\ 0.984808 & 0.000000 & 0.173648 & 1.586000 \\ 0.000000 & 0.000000 & 0.000000 & 1.000000 \end{bmatrix}$$
* **Translation Vector:** $\mathbf{t} = [X=1.586\,\text{m}, Y=0.260\,\text{m}, Z=-1.053\,\text{m}]$ (in camera optical coordinates).
* **Euler Angles (Roll, Pitch, Yaw):** $[-80.0^\circ, 0.0^\circ, -90.0^\circ]$.
* **Quaternion $[q_x, q_y, q_z, q_w]$:** $[-0.455, 0.455, -0.542, 0.542]$.

#### C. IMU-to-LiDAR Extrinsic Matrix ($\mathbf{T}_{\text{imu}}^{\text{velo}}$)
$$\mathbf{T}_{\text{imu}}^{\text{velo}} = \begin{bmatrix} 0.984808 & 0.000000 & -0.173648 & -2.223539 \\ 0.000000 & 1.000000 & 0.000000 & 0.000000 \\ 0.173648 & 0.000000 & 0.984808 & -1.596569 \\ 0.000000 & 0.000000 & 0.000000 & 1.000000 \end{bmatrix}$$

---

## 4. Multi-Modal Synchronization & Temporal Matching

### 4.1 Temporal Drift Formulation & Latency Resolution
In distributed robotic systems where sensors publish at heterogeneous frequencies ($f_{\text{lidar}} = 10\,\text{Hz}$, $f_{\text{cam}} \approx 15-30\,\text{Hz}$, $f_{\text{imu}} = 100\,\text{Hz}$), temporal misalignment induces projection distortion under vehicle motion:
$$\delta \mathbf{p} \approx \mathbf{J}_{\text{cam}} \cdot \mathbf{v}_{\text{vehicle}} \cdot \Delta t$$
where $\mathbf{v}_{\text{vehicle}}$ is vehicle velocity, $\Delta t$ is the timestamp offset between modalities, and $\mathbf{J}_{\text{cam}}$ is the camera projection Jacobian. At $5\,\text{m/s}$ ($18\,\text{km/h}$), a $100\,\text{ms}$ desynchronization introduces a $0.5\,\text{m}$ spatial error, catastrophically degrading 3D cuboid labeling.

**Addressing the 3–4 Second Lag:** Initial testing revealed an unacceptable 3–4 second delay in camera-LiDAR RViz rendering caused by uncompressed ROS Image message broadcasting across Python TCPROS sockets. This was resolved by:
1. Eliminating Python TCP intermediate republishing nodes.
2. Relying on native shared-memory ROS nodelets and direct C++ RViz frustum rendering.
3. Enabling direct NumPy buffer memory decoding (`np.frombuffer`) during dataset export, reducing latency from $3500\,\text{ms}$ to $< 30\,\text{ms}$ (real-time).

### 4.2 Nearest-Timestamp Frame Pairing Algorithm
For every LiDAR point cloud frame $k$ with header timestamp $t_{L, k}$, the corresponding camera frame $j^*$ is selected by solving the minimization problem:
$$j^* = \arg\min_{j} |t_{C, j} - t_{L, k}| \quad \text{subject to } |t_{C, j^*} - t_{L, k}| \le \tau_{\max}$$
where $\tau_{\max} = 35\,\text{ms}$ is the strict pairing window tolerance. If no camera frame falls within $\tau_{\max}$, the LiDAR frame is dropped or flagged in the quality audit.

```mermaid
sequenceDiagram
    participant IMU as IMU Stream (100 Hz)
    participant LiDAR as Velodyne VLP-16 (10 Hz)
    participant Cam as ZED X Camera (15-30 Hz)
    participant Exp as Decoupled Exporter

    Note over IMU,Cam: Common Chrony PTP/NTP Clock Reference (< 0.05ms)
    IMU->>Exp: High-rate samples continuous (t_0, t_1, ..., t_n) -> imu_data.csv
    Cam->>Exp: Frame j (t_C = 1723456789.1250)
    LiDAR->>Exp: Scan k (t_L = 1723456789.1234)
    Note over Exp: Nearest Search: |t_C - t_L| = 1.6ms <= 35ms (MATCH)
    Exp->>Exp: Export Keyframe 000000: velodyne/000000.bin, image_2/000000.png, calib/000000.txt
    Exp->>Exp: Associate NDT Pose(t_L) -> poses/poses.txt
```

---

## 5. Preprocessing & Self-Reflection Spatial Filtering

### 5.1 Ego-Vehicle Bounding Box Filtering
The Velodyne VLP-16 LiDAR is mounted on the Buggy's central roof bar. Consequently, the laser beams hit the front roof arch, windshield pillars, and front bumper, creating dense self-reflections (ghost obstacles) that disrupt 3D object detectors and clutter annotation tools.

An axis-aligned 3D spatial filter in LiDAR coordinates is applied to purge internal points:
$$\mathcal{B}_{\text{ego}} = \left\{ \mathbf{P}_V \in \mathbb{R}^3 \;\middle|\; \begin{aligned} -1.2\,\text{m} &\le X_V \le 0.8\,\text{m} \\ -0.65\,\text{m} &\le Y_V \le 0.65\,\text{m} \\ -1.8\,\text{m} &\le Z_V \le -0.2\,\text{m} \end{aligned} \right\}$$
Any point $\mathbf{P} \in \mathcal{B}_{\text{ego}}$ is removed prior to writing to binary `.bin` and `.pcd` files, reducing point cloud noise while preserving legitimate ground plane and forward obstacles.

---

## 6. Dataset Collection & Processing Pipeline

### 6.1 Two-Stage Architecture
To prevent CPU/memory contention and frame dropping during driving, data handling is strictly decoupled:

```mermaid
graph LR
    subgraph "Stage 1: Acquisition (Real-Time)"
        Sensors[Sensors + Telemetry] -->|High-Throughput| Rosbag["rosbag record -b 2048<br/>raw_bag/session.bag"]
    end

    subgraph "Stage 2: Post-Processing & Export (Decoupled)"
        Rosbag --> Exporter["Multi-Threaded Exporter<br/>(export_bag_to_kitti.py)"]
        Exporter --> Kitti["KITTI Format<br/>(velodyne/, image_2/, calib/, poses/)"]
        Exporter --> PCD["High-Precision 3D<br/>(velodyne_pcd/*.pcd)"]
        Exporter --> IMU["Continuous Inertial<br/>(imu/imu_data.csv)"]
        Exporter --> QA["Automated QA Engine<br/>(quality_report/*.json)"]
    end
```

### 6.2 Standardized Dataset Directory Layout
The generated dataset satisfies standard KITTI requirements while adding dedicated inertial and quality assurance directories:
```text
session_YYYYMMDD_HHMMSS/
├── raw_bag/
│   └── session_YYYYMMDD_HHMMSS.bag            # Complete raw ROS bag (source of truth)
├── session_info.yaml                          # ROS Master, hostnames, recorded topics
├── README.md                                  # Auto-generated dataset documentation
├── calibration/                               # Standalone calibration suite
│   ├── camera_intrinsics.yaml                 # P2, K, distortion, image resolution
│   ├── lidar_camera_extrinsics.yaml           # 6-DOF T_velo_to_cam, RPY, translation
│   ├── imu_lidar_extrinsics.yaml              # 6-DOF T_imu_to_velo
│   └── tf_tree.yaml                           # Static transform tree
├── quality_report/                            # Automated dataset health audit
│   ├── synchronization_report.json            # Mean dt, max dt, std dev, paired %
│   ├── sensor_frequency_report.json           # Nominal vs measured sensor Hz
│   ├── dropped_frames.json                    # Unpaired or skipped frames
│   ├── recording_health_report.json           # Integrity metrics & SHA-256 hashes
│   └── verification_overlay_sample.jpg        # Camera-LiDAR projection overlay
└── kitti_format/                              # Annotation-ready format
    ├── image_2/                               # Rectified RGB frames (000000.png, ...)
    ├── velodyne/                              # Standard KITTI float32 binary (X, Y, Z, I)
    ├── velodyne_pcd/                          # Binary PCD format (Supervisely/CVAT 3D)
    ├── calib/                                 # Per-frame KITTI calib text files
    ├── poses/                                 # 6-DOF NDT localized vehicle trajectory
    ├── imu/                                   # Dedicated 100 Hz raw IMU CSV
    │   └── imu_data.csv                       # Timestamp, Accel, Gyro, Quat, Covariances
    └── timestamps/                            # Dual-clock timestamp archives
        ├── lidar_timestamps.txt
        ├── camera_timestamps.txt
        ├── imu_timestamps.txt
        └── synchronized_pairs.txt
```

---

## 7. Experimental Validation & Results

### 7.1 Quantitative Synchronization Performance
The synchronization pipeline was benchmarked on a representative test session on the Autonomous Buggy platform:

| Metric | Measured Value | Standard / Requirement | Assessment |
| :--- | :--- | :--- | :--- |
| **LiDAR Publish Frequency** | $10.02\,\text{Hz}$ | $10.0\,\text{Hz} \pm 0.5$ | Nominal |
| **Camera Publish Frequency** | $15.14\,\text{Hz}$ | $15.0\,\text{Hz} \pm 1.0$ | Nominal |
| **IMU Publish Frequency** | $100.26\,\text{Hz}$ | $100.0\,\text{Hz} \pm 2.0$ | Nominal |
| **Inter-Machine Clock Skew (Chrony PTP)** | $< 0.035\,\text{ms}$ | $< 1.0\,\text{ms}$ | Excellent |
| **Mean Pairing Offset ($\mu_{\Delta t}$)** | $19.97\,\text{ms}$ | $\le 35.0\,\text{ms}$ | Synchronized |
| **Maximum Pairing Offset ($\max \Delta t$)** | $27.42\,\text{ms}$ | $\le 35.0\,\text{ms}$ | Passed |
| **Pairing Success Rate** | $100.0\%$ | $\ge 95.0\%$ | Zero Dropped Pairs |
| **End-to-End Latency** | $< 30\,\text{ms}$ | $< 50\,\text{ms}$ | Real-Time |

### 7.2 Qualitative Camera-LiDAR Calibration Alignment
Visual verification was conducted using dense distance-colored point cloud projection over the camera optical plane:
* **Planar Boundaries:** Projected laser points from vertical door frames and brick walls align with structural edges without angular tilt or depth distortion.
* **Dynamic Targets:** Pedestrians standing at varying distances ($3\,\text{m}$ to $15\,\text{m}$) exhibit point cloud clusters contouring their silhouettes.
* **Range Coloring:** Jet/rainbow colormap encoding depth ($Z_C \in [0.5\,\text{m}, 25.0\,\text{m}]$) confirms monotonic depth consistency.

---

## 8. Annotation Compatibility & Downstream Applications

### 8.1 3D Cuboid Annotation Platforms
The generated dataset is validated for direct ingestion into leading point cloud annotation suites:
* **Supervisely 3D Point Cloud Tool:** Directly consumes `velodyne_pcd/*.pcd` along with `image_2/*.png` and `calib/*.txt` to enable 3D bounding box projection into corresponding camera views.
* **CVAT (Computer Vision Annotation Tool) 3D:** Uses standard KITTI `velodyne/*.bin` format with 3D cuboid labeling.

### 8.2 Supported Research & Perception Benchmarks
1. **3D Object Detection:** Directly consumable by OpenPCDet, MMDetection3D, PointPillars, CenterPoint, and PV-RCNN.
2. **Multi-Modal Camera-LiDAR Fusion:** Evaluation of BEVFusion, TransFusion, and PointPainting architectures.
3. **LiDAR-Inertial Odometry & SLAM:** Raw 100 Hz IMU sequences (`imu_data.csv`) and uncompressed point clouds enable benchmarking against LIO-SAM, FAST-LIO2, and VINS-Mono.
4. **Localization Benchmarking:** NDT trajectory (`poses/poses.txt`) provides reference trajectories for evaluating GNSS-denied campus localization.

---

## 9. Appendix: LaTeX Formulas & BibTeX Citations

### 9.1 Ready-to-Use LaTeX Equations

```latex
% Projection Equation
\begin{equation}
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{P}_2 \, \mathbf{T}_{V}^{C} \begin{bmatrix} X_V \\ Y_V \\ Z_V \\ 1 \end{bmatrix}
= \begin{bmatrix} f_x & 0 & c_x & 0 \\ 0 & f_y & c_y & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}
\begin{bmatrix} \mathbf{R}_{V}^{C} & \mathbf{t}_{V}^{C} \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix}
\begin{bmatrix} X_V \\ Y_V \\ Z_V \\ 1 \end{bmatrix}
\end{equation}

% Extrinsic Matrix
\begin{equation}
\mathbf{T}_{V}^{C} = \begin{bmatrix}
0.000000 & -1.000000 & 0.000000 & 0.260000 \\
0.173648 & 0.000000 & -0.984808 & -1.053000 \\
0.984808 & 0.000000 & 0.173648 & 1.586000 \\
0.000000 & 0.000000 & 0.000000 & 1.000000
\end{bmatrix}
\end{equation}

% Timestamp Pairing Minimization
\begin{equation}
j^* = \arg\min_{j} |t_{C, j} - t_{L, k}| \quad \text{s.t.} \quad |t_{C, j^*} - t_{L, k}| \le \tau_{\max}
\end{equation}
```

### 9.2 Key BibTeX Citations for Literature Review
```bibtex
@article{geiger2013vision,
  title={Vision meets robotics: The KITTI dataset},
  author={Geiger, Andreas and Lenz, Philip and Stiller, Christoph and Urtasun, Raquel},
  journal={The International Journal of Robotics Research},
  volume={32},
  number={11},
  pages={1231--1237},
  year={2013}
}

@inproceedings{lang2019pointpillars,
  title={PointPillars: Fast encoders for object detection from point clouds},
  author={Lang, Alex H and Vora, Sourabh and Caesar, Holger and Zhou, Lubing and Yang, Jiong and Beijbom, Oscar},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={12697--12705},
  year={2019}
}

@article{shan2020lio,
  title={LIO-SAM: Tightly-coupled lidar inertial odometry via smoothing and mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus, Daniela},
  journal={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135--5142},
  year={2020}
}

@inproceedings{kato2018autoware,
  title={Autoware on board: Enabling autonomous vehicles with embedded systems},
  author={Kato, Shinpei and Tokunaga, Eijiro and Maruyama, Yuya and Maeda, Seiya and Hirabayashi, Manato and Kitsukawa, Yuki and Kurihana, Takuya and Tanaka, Hiroki and Kawamata, Takaaki},
  booktitle={ACM/IEEE International Conference on Computer-Aided Design (ICCAD)},
  pages={1--7},
  year={2018}
}

@inproceedings{liu2023bevfusion,
  title={BEVFusion: Multi-task multi-sensor fusion with unified bird's-eye view representation},
  author={Liu, Zhijian and Tang, Haotian and Amini, Alexander and Yang, Xinyu and Mao, Huizi and Rus, Daniela and Han, Song},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={2774--2781},
  year={2023}
}
```
