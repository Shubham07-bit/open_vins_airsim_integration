# System Integration Architecture: ArduPilot SITL + AirSim + OpenVINS

## Complete System Diagram

```
+========================================================================================+
|                          COMPLETE INTEGRATION ARCHITECTURE                               |
+========================================================================================+

 +-------------------+         UDP (port 9002)          +-------------------+
 |                   | <------ PWM[11] servo_packet ----| ArduPilot SITL    |
 |   AirSim          |                                  | (ArduCopter)      |
 |   (Unreal Engine) | ------- JSON sensor data ------->|                   |
 |                   |         UDP (port 9003)          | sim_vehicle.py    |
 |                   |                                  | --model=airsim    |
 +------|------|-----+                                  | --frame=airsim    |
        |      |                                        +-------------------+
        |      |
        |      +-- IMU Data (getImuData API)
        |          angular_velocity [x,y,z] rad/s
        |          linear_acceleration [x,y,z] m/s^2
        |          orientation quaternion
        |          timestamp (nanoseconds)
        |
        +--------- Camera Data (simGetImages API)
                   RGB image (640x480, Scene type 0)
                   numpy array (uint8, BGR)
                   timestamp (nanoseconds)
                   |
                   |  AirSim Python API (msgpack-rpc, port 41451)
                   |
                   v
 +-------------------+
 | ROS2 Bridge Node  |    (airsim_openvins_bridge.py)
 | (Python rclpy)    |
 |                   |
 | Connects to       |
 | AirSim via Python |
 | API client        |
 +------|------|-----+
        |      |
        |      +-- Publishes: /imu  (sensor_msgs/Imu)
        |          - header.stamp = AirSim timestamp (converted to ROS Time)
        |          - angular_velocity (x,y,z)
        |          - linear_acceleration (x,y,z)
        |          - orientation (quaternion)
        |          - Rate: 200 Hz
        |
        +--------- Publishes: /camera/image  (sensor_msgs/Image)
                   - header.stamp = AirSim timestamp (synchronized with IMU)
                   - encoding: mono8 (grayscale for VIO)
                   - width: 640, height: 480
                   - Rate: 20-30 Hz
                   |
                   v
 +-------------------+
 | OpenVINS          |    (run_subscribe_msckf)
 | (ov_msckf)        |
 |                   |
 | Subscribes to:    |
 |   /imu            |
 |   /camera/image   |
 +------|------------+
        |
        +-- Publishes:
            /ov_msckf/poseimu    (PoseWithCovarianceStamped)
            /ov_msckf/odomimu    (Odometry)
            /ov_msckf/pathimu    (Path)
            /ov_msckf/points_msckf (PointCloud2)
            /ov_msckf/points_slam  (PointCloud2)
            TF: global -> imu
```

## Data Flow Summary

```
ArduPilot SITL                AirSim                    ROS2 Bridge            OpenVINS
     |                           |                           |                     |
     |--- PWM servo cmds ------->|                           |                     |
     |    (UDP 9002, binary)     |                           |                     |
     |                           |                           |                     |
     |<-- JSON sensor data ------|                           |                     |
     |    (UDP 9003)             |                           |                     |
     |    {timestamp, imu,       |                           |                     |
     |     gps, pose, velocity}  |                           |                     |
     |                           |                           |                     |
     |                           |<-- getImuData() ---------|                     |
     |                           |    (RPC port 41451)       |                     |
     |                           |--- ImuData response ----->|                     |
     |                           |                           |                     |
     |                           |<-- simGetImages() --------|                     |
     |                           |    (RPC port 41451)       |                     |
     |                           |--- ImageResponse -------->|                     |
     |                           |                           |                     |
     |                           |                           |--- /imu ----------->|
     |                           |                           |    (200 Hz)         |
     |                           |                           |                     |
     |                           |                           |--- /camera/image -->|
     |                           |                           |    (20 Hz)          |
     |                           |                           |                     |
     |                           |                           |    /ov_msckf/odomimu|
     |                           |                           |<----- (pose est) ---|
```

## Connection Details

### ArduPilot SITL <-> AirSim (Built-in)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Protocol | UDP | Bidirectional UDP sockets |
| Control Port | 9002 | ArduPilot sends PWM to AirSim |
| Sensor Port | 9003 | AirSim sends JSON sensor data to ArduPilot |
| IP Address | 127.0.0.1 | Localhost (same machine) |
| Data Format (Control) | Binary `uint16_t[11]` | PWM values for up to 11 rotors |
| Data Format (Sensor) | JSON over UDP | `{timestamp, imu, gps, pose, velocity}` |
| Update Rate | ~400 Hz | Physics step rate |
| Vehicle Type | ArduCopter | AirSim setting `"VehicleType": "ArduCopter"` |

### AirSim <-> ROS2 Bridge (Python API)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Protocol | msgpack-RPC | AirSim Python client library |
| Port | 41451 | Default AirSim API port |
| IP Address | 127.0.0.1 | Localhost |
| IMU API | `client.getImuData()` | Returns angular_velocity, linear_acceleration, orientation |
| Camera API | `client.simGetImages()` | Returns compressed/uncompressed images |
| IMU Rate | 200 Hz | Polling rate in bridge node |
| Camera Rate | 20 Hz | Image capture rate |

### ROS2 Bridge -> OpenVINS (ROS2 Topics)

| Topic | Message Type | Rate | QoS |
|-------|-------------|------|-----|
| `/imu` | `sensor_msgs/msg/Imu` | 200 Hz | SensorDataQoS (BEST_EFFORT, depth=10) |
| `/camera/image` | `sensor_msgs/msg/Image` | 20 Hz | Default (RELIABLE, depth=10) |

## Timestamp Synchronization Strategy

### Problem
AirSim uses its own internal clock (nanoseconds since simulation start). ROS2 uses `rclcpp::Time`.
Camera and IMU data are captured at different rates and may not be perfectly synchronized.

### Solution: Single-Clock Approach

1. **Time Base**: Use AirSim's internal `timestamp` (nanoseconds) as the single source of truth
2. **Conversion**: Convert AirSim nanosecond timestamps to ROS2 `builtin_interfaces/Time`:
   ```python
   ros_time.sec = airsim_timestamp_ns // 1_000_000_000
   ros_time.nanosec = airsim_timestamp_ns % 1_000_000_000
   ```
3. **IMU Timestamps**: Taken directly from `imu_data.time_stamp` (AirSim native)
4. **Camera Timestamps**: Taken from `simGetImages()` response timestamp
5. **use_sim_time**: Set to `false` -- we embed the AirSim clock in message headers
6. **OpenVINS time_offset**: Set `calib_cam_timeoffset: true` to auto-calibrate any residual offset

### Timestamp Flow
```
AirSim Internal Clock (ns)
    |
    +-> imu_data.time_stamp ---------> /imu header.stamp
    |
    +-> image_response.time_stamp ----> /camera/image header.stamp
    |
    +-> Both arrive at OpenVINS with consistent time base
        OpenVINS calib_cam_timeoffset handles residual offset
```

## Latency Analysis

| Stage | Expected Latency | Notes |
|-------|-----------------|-------|
| AirSim physics step | < 1 ms | Internal simulation loop |
| ArduPilot <-> AirSim UDP | < 1 ms | Localhost, minimal overhead |
| AirSim API call (IMU) | 1-2 ms | msgpack-RPC overhead |
| AirSim API call (Image) | 5-15 ms | Image capture + transfer (640x480) |
| ROS2 publish (IMU) | < 1 ms | In-process serialization |
| ROS2 publish (Image) | 1-3 ms | Image serialization |
| OpenVINS feature tracking | 5-20 ms | KLT tracking (300 features) |
| OpenVINS MSCKF update | 2-10 ms | State update |
| **Total Pipeline** | **~15-50 ms** | End-to-end |

### Real-Time Constraints
- IMU data must arrive before camera data for same timestamp (OpenVINS requirement)
- Camera rate (20 Hz = 50ms period) must be stable within ~5ms jitter
- IMU rate (200 Hz = 5ms period) provides 10:1 ratio vs camera (good for preintegration)

## Coordinate Frame Conventions

```
AirSim (NED):           OpenVINS (expected):     Transformation:
  X = North              X = Forward               Direct mapping in sim
  Y = East               Y = Left                  (camera frame matters)
  Z = Down               Z = Up                    IMU: NED->body handled
                                                    by config T_i_b
```

### Frame Transforms
- **AirSim IMU Frame**: NED (North-East-Down) body frame
- **OpenVINS IMU Frame**: Configurable via `T_i_b` in `kalibr_imu_chain.yaml` (set to identity for sim)
- **Camera Frame**: Forward-looking, configured via `T_imu_cam` in `kalibr_imucam_chain.yaml`
- **Camera Position on Drone**: X=0.50m forward, Y=0.00m, Z=0.10m up (from AirSim settings)

## System Requirements

| Component | Requirement |
|-----------|-------------|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Python | 3.10+ |
| AirSim | Built from source (~/AirSim) |
| ArduPilot | Built SITL (~/dev/ardupilot) |
| OpenVINS | Built in workspace (/mnt/rnd/Shubham/openvins_ws) |
| airsim pip | 1.8.1 (installed) |
| GPU | Required for AirSim rendering |
