# FUTURE.md — ouster-ros

> Official Ouster LiDAR ROS2 driver — connects to physical Ouster sensors or replays PCAP files, publishes organized PointCloud2, range/signal/reflec/near-IR panoramic images, IMU data, and LaserScan.
> Last updated: 2026-03-05

## Purpose
Hardware driver and processing pipeline for Ouster LiDAR sensors (OS-0, OS-1, OS-2, etc.). Converts raw LiDAR + IMU packets to ROS2 topics. The organized PointCloud2 output (height > 1) is used by `lidar_2d_detection` and other perception packages for 3D back-projection. Part of the sensing subsystem. Upstream Ouster official package.

## Sub-Packages

| Package | Description |
|---------|-------------|
| `ouster_ros` | Main driver — sensor, cloud processing, image, pcap replay |
| `ouster_sensor_msgs` | Custom message types (`PacketMsg`) and services (`GetConfig`, `SetConfig`) |

## Nodes

| Node | Executable | Purpose |
|------|-----------|---------|
| `OusterDriver` | `os_driver` | **Recommended** — combined sensor+cloud+image+IMU in single process |
| `OusterSensor` | `os_sensor` | Sensor connection only; outputs raw packets |
| `OusterCloud` | `os_cloud` | Cloud/image/IMU processing from packets |
| `OusterImage` | `os_image` | Panoramic image generation only |
| `OusterPcap` | `os_pcap` | Replay from PCAP file |

All nodes are **ROS2 Lifecycle nodes** (`rclcpp_lifecycle`). Must be explicitly configured and activated (launch files handle this via lifecycle events).

## Design Pattern

**Lifecycle + composable node pattern:**

1. `os_driver` is a `LifecycleNode`. Launch triggers `TRANSITION_CONFIGURE` → `TRANSITION_ACTIVATE`.
2. On configure: connects to sensor (TCP), fetches metadata (sensor_info), validates config.
3. On activate: starts UDP listener threads for LiDAR and IMU packets → `LidarPacketHandler` / `ImuPacketHandler` → processors.
4. `proc_mask` parameter controls which processing pipelines are enabled: `IMU|PCL|SCAN|IMG|RAW|TLM`.

**Lock-free ring buffer** (`lock_free_ring_buffer.h`): Receives UDP packets from socket thread without blocking.

**Organized point cloud:** Published as `PointCloud2` with `height = num_beams` (e.g., 64 or 128) and `width = num_cols` (typically 1024/2048/4096). This allows pixel-indexed access for `lidar_2d_detection`'s back-projection.

## ROS Interfaces

### Publishers (from `os_driver` with default `ouster_ns=ouster`)

| Topic | Type | Notes |
|-------|------|-------|
| `ouster/points` | `sensor_msgs/PointCloud2` | Organized cloud (primary output) |
| `ouster/imu` | `sensor_msgs/Imu` | IMU data |
| `ouster/range_image` | `sensor_msgs/Image` | Range panoramic (16-bit) |
| `ouster/reflec_image` | `sensor_msgs/Image` | Reflectivity panoramic |
| `ouster/signal_image` | `sensor_msgs/Image` | Signal/intensity panoramic |
| `ouster/near_ir_image` | `sensor_msgs/Image` | Near-IR panoramic |
| `ouster/scan` | `sensor_msgs/LaserScan` | 2D laser scan slice |
| `ouster/lidar_packets` | `ouster_sensor_msgs/PacketMsg` | Raw LiDAR packets (if `RAW` in proc_mask) |
| `ouster/imu_packets` | `ouster_sensor_msgs/PacketMsg` | Raw IMU packets |

### Services

| Service | Type | Notes |
|---------|------|-------|
| `ouster/get_config` | `ouster_sensor_msgs/GetConfig` | Get sensor config JSON |
| `ouster/set_config` | `ouster_sensor_msgs/SetConfig` | Set sensor config JSON |
| `ouster/reset` | `std_srvs/Empty` | Soft sensor reset |

### Parameters (key ones for `os_driver`)

| Param | Default | Description |
|-------|---------|-------------|
| `sensor_hostname` | — | Sensor IP or hostname (required for live sensor) |
| `lidar_ip` | — | Host UDP IP for LiDAR packets |
| `imu_ip` | — | Host UDP IP for IMU packets |
| `lidar_port` | `0` | UDP port (0 = auto-assign) |
| `imu_port` | `0` | UDP port (0 = auto-assign) |
| `proc_mask` | `IMU|PCL|SCAN|IMG|RAW|TLM` | Which processors to enable |
| `point_type` | `original` | Point cloud type: `original`, `xyz`, `xyzi`, `xyzir` |
| `organized` | `true` | Maintain organized cloud (required for back-projection) |
| `destagger` | `true` | Destagger scan before publishing |
| `min_range` | `0.0` m | Filter points below this range |
| `max_range` | `1000.0` m | Filter points above this range |
| `v_reduction` | `1` | Vertical beam reduction factor (1=all beams) |
| `scan_ring` | `0` | Ring index for LaserScan output |
| `ptp_utc_tai_offset` | `-37.0` s | PTP timing offset |

## Launch Files

```bash
# Live sensor (recommended — single process)
ros2 launch ouster_ros driver.launch.py \
  params_file:=/path/to/driver_params.yaml \
  ouster_ns:=ouster viz:=false

# Sensor + cloud in separate processes
ros2 launch ouster_ros sensor.independent.launch.py \
  params_file:=/path/to/driver_params.yaml

# PCAP replay
ros2 launch ouster_ros driver_launch.py \
  pcap_file:=/path/to/recording.pcap \
  metadata:=/path/to/sensor.json
```

**Config file:** `config/driver_params.yaml` — must contain `sensor_hostname`, UDP IPs, and processing options. Template in `ouster-ros/ouster-ros/config/`.

## Config Files
- `config/driver_params.yaml`: Primary sensor configuration. Key fields:
  - `sensor_hostname`: IP address of Ouster sensor
  - `udp_dest`: Host IP for receiving UDP packets
  - `lidar_mode`: Scan resolution/rate (e.g., `1024x20`, `2048x10`)
  - `viz`: Enable RViz visualization

## Key Dependencies

| Dependency | Usage |
|------------|-------|
| `ouster-sdk` (C++) | Ouster sensor SDK — packet parsing, calibration, `sensor_info` |
| `pcl_conversions` | PointCloud2 ↔ PCL conversion |
| `tf2_ros` | Static TF for sensor → lidar frames |
| `lifecycle_msgs` | Lifecycle state machine |

## Build Notes

```bash
# Initialize submodules (ouster-sdk is a git submodule)
cd /home/ubuntu/Zelda/Projects/all_repos/ouster-ros
git submodule update --init --recursive

colcon build --packages-up-to ouster_ros
```

**Critical:** `ouster-sdk` is a git submodule. Must run `git submodule update --init` before building.

## Known Issues

| Severity | Description |
|----------|-------------|
| High | Requires `git submodule update --init --recursive` before build — missing submodule causes cryptic build failures |
| Medium | `organized=false` will break `lidar_2d_detection` (requires organized cloud for pixel-indexed back-projection) — always set `organized:=true` for this stack |
| Low | `v_reduction > 1` reduces vertical resolution — downstream consumers like `lidar_2d_detection` assume full resolution; verify compatibility before using |
| Info | Upstream Ouster official package. Pin to a release tag matching the Ouster firmware version in deployment. |
