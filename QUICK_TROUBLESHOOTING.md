# Quick Troubleshooting Guide

## Connection Problems

### Problem: "Unable to bind Airsim sensor_in socket at port 9003"
**Cause**: Port already in use (previous ArduPilot instance still running)
**Fix**:
```bash
# Kill any existing ArduPilot SITL processes
pkill -f arducopter
pkill -f sim_vehicle.py
# Wait a moment, then restart
```

### Problem: "No sensor message received in last 1s"
**Cause**: AirSim is not running or not configured for ArduCopter
**Fix**:
1. Verify AirSim is running and fully loaded (Unreal window visible)
2. Check `~/Documents/AirSim/settings.json` has `"VehicleType": "ArduCopter"`
3. Verify ports match: `UdpPort: 9003` and `ControlPort: 9002`
4. Restart AirSim, then ArduPilot (order matters!)

### Problem: Bridge node "Failed to connect to AirSim"
**Cause**: AirSim API server not ready
**Fix**:
```bash
# Check if AirSim API port is listening
ss -tuln | grep 41451
# If not listed, AirSim hasn't fully started yet - wait longer
```

### Problem: "rpc_error" or "Connection refused" from Python API
**Cause**: AirSim crashed or was restarted
**Fix**: Restart the bridge node after restarting AirSim. The bridge has auto-reconnect
but a fresh start is more reliable.

---

## Timing Issues

### Problem: OpenVINS "initialization failed" or never initializes
**Causes and Fixes**:

1. **No motion**: OpenVINS needs rotation/translation to initialize
   ```
   # In MAVProxy:
   arm throttle
   mode guided
   takeoff 3
   velocity 0.5 0 0 3  # Move to create visual parallax
   ```

2. **IMU not arriving before camera**: Check rates
   ```bash
   ros2 topic hz /imu
   ros2 topic hz /camera/image
   # IMU should be ~200Hz, camera ~20Hz
   ```

3. **Timestamps are zero or wrong**: Check bridge output
   ```bash
   ros2 topic echo /imu --field header.stamp --once
   # sec should be > 0
   ```

4. **Too much noise in init**: Try adjusting init parameters
   ```yaml
   # In estimator_config.yaml:
   init_imu_thresh: 0.3     # Lower = less sensitive (try 0.3 -> 1.0)
   init_window_time: 3.0    # Longer window
   init_max_features: 50    # More features during init
   ```

### Problem: OpenVINS diverges after initialization
**Causes**:
1. **Camera intrinsics wrong**: Verify FOV matches settings.json
   ```
   AirSim FOV: 90 degrees -> fx = fy = 320.0 for 640x480
   Check kalibr_imucam_chain.yaml: intrinsics: [320.0, 320.0, 320.0, 240.0]
   ```

2. **IMU noise parameters too low**: Increase noise density
   ```yaml
   # In kalibr_imu_chain.yaml:
   accelerometer_noise_density: 5.0e-2  # increase from 1.0e-2
   gyroscope_noise_density: 5.0e-3      # increase from 1.0e-3
   ```

3. **Camera-IMU extrinsics wrong**: Verify transform
   ```yaml
   # In kalibr_imucam_chain.yaml:
   # Camera at X=0.50, Y=0.0, Z=-0.10 (NED) from body center
   T_imu_cam:
     - [1.0, 0.0, 0.0, 0.50]
     - [0.0, 1.0, 0.0, 0.00]
     - [0.0, 0.0, 1.0, -0.10]
     - [0.0, 0.0, 0.0, 1.00]
   ```

### Problem: "not enough features tracked"
**Causes**:
1. **Scene too dark/featureless**: AirSim scene needs visual texture
2. **Fast rotation**: Reduce yaw rate or increase `num_pts`
3. **Motion blur**: Reduce `ClockSpeed` in AirSim settings

**Fix**:
```yaml
# In estimator_config.yaml:
num_pts: 400          # More features (was 300)
fast_threshold: 20    # Lower threshold = more corners detected (was 30)
min_px_dist: 5        # Allow features closer together (was 8)
histogram_method: "CLAHE"  # Better contrast normalization
```

---

## Performance Bottlenecks

### Problem: Low FPS / stuttering in AirSim
**Fix**:
```bash
# Reduce resolution
./AirSimNH.sh -ResX=640 -ResY=480 -windowed

# Or reduce clock speed in settings.json
"ClockSpeed": 0.5  # Half speed
```

### Problem: Bridge node drops frames
**Cause**: Image capture is too slow
**Fix**:
1. Reduce camera rate: `camera_rate_hz: 15.0`
2. Reduce image resolution in AirSim settings:
   ```json
   "Width": 480, "Height": 360
   ```
   (Update OpenVINS intrinsics accordingly!)

### Problem: High CPU usage from bridge node
**Cause**: Image conversion in Python is CPU-intensive
**Fix**: The grayscale conversion uses numpy which is efficient, but if still slow:
1. Reduce camera rate
2. Reduce resolution
3. Consider using compressed transport

### Problem: OpenVINS output rate is low (< 15 Hz)
**Cause**: Feature tracking is CPU-bound
**Fix**:
```yaml
# In estimator_config.yaml:
num_pts: 200          # Fewer features (was 300)
num_opencv_threads: 6 # More threads if cores available
downsample_cameras: true  # Halve image for tracking
```

---

## Common Error Messages

| Error | Meaning | Fix |
|-------|---------|-----|
| `Unable to send servo output` | AirSim not accepting motor commands | Restart AirSim first |
| `Failed to parse Vector3f` | Malformed JSON from AirSim | AirSim version mismatch or bug |
| `not enough features` | Camera sees blank/dark area | Add more scene detail or adjust thresholds |
| `ZUPT detected but velocity > max` | Zero-velocity conflict | Tune `zupt_max_velocity` |
| `config file does not exist` | Wrong config path | Check `config_path` parameter |
| `no imu data received` | Bridge not publishing IMU | Check bridge is running and AirSim connected |
| `image timeout` | Camera data not arriving | Check bridge camera thread, AirSim camera config |

---

## Quick Reset Procedure

When things go wrong, reset everything in this order:

```bash
# 1. Kill all components
pkill -f run_subscribe_msckf
pkill -f airsim_openvins_bridge
pkill -f arducopter
pkill -f sim_vehicle.py
pkill -f AirSimNH

# 2. Wait for cleanup
sleep 5

# 3. Restart in order
# Terminal 1: AirSim
cd ~/AirSim/AirSimNH/LinuxNoEditor && ./AirSimNH.sh -ResX=800 -ResY=600 -windowed

# Wait for AirSim to fully load (~30 seconds)

# Terminal 2: ArduPilot
cd ~/dev/ardupilot && sim_vehicle.py -v ArduCopter --model=airsim-copter --console --map

# Wait for "FPS avg" message

# Terminal 3: Integration
source /opt/ros/humble/setup.bash && source /mnt/rnd/Shubham/openvins_ws/install/setup.bash
ros2 launch /mnt/rnd/Shubham/openvins_ws/integration/integration_launch.py
```

---

## Diagnostic Commands

```bash
# Check all running nodes
ros2 node list

# Check all topics with types
ros2 topic list -t

# Check a specific topic's QoS
ros2 topic info /imu -v

# Check system resource usage
htop

# Check GPU (AirSim rendering)
nvidia-smi

# Check network ports
ss -tuln | grep -E "(9002|9003|41451|14550)"

# Quick IMU sanity check (should show ~9.81 on z when stationary)
ros2 topic echo /imu --field linear_acceleration --once

# Quick image sanity check (should show non-zero data)
ros2 topic echo /camera/image --field height --once
ros2 topic echo /camera/image --field width --once
ros2 topic echo /camera/image --field encoding --once
```
