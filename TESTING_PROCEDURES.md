# Testing Procedures: ArduPilot SITL + AirSim + OpenVINS

## Component-by-Component Testing

### Test 1: AirSim Standalone

**Goal**: Verify AirSim runs and exposes APIs correctly.

```bash
# 1. Start AirSim
cd ~/AirSim/AirSimNH/LinuxNoEditor
./AirSimNH.sh -ResX=800 -ResY=600 -windowed

# 2. Run API test
python3 << 'EOF'
import airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()
print("=== AirSim Connected ===")

# Test IMU
for i in range(5):
    imu = client.getImuData()
    print(f"IMU[{i}] ts={imu.time_stamp} "
          f"accel=[{imu.linear_acceleration.x_val:.3f}, "
          f"{imu.linear_acceleration.y_val:.3f}, "
          f"{imu.linear_acceleration.z_val:.3f}]")

# Test camera
imgs = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
])
if imgs and imgs[0].width > 0:
    img = np.frombuffer(imgs[0].image_data_uint8, dtype=np.uint8)
    print(f"Camera: {imgs[0].width}x{imgs[0].height}, "
          f"bytes={len(imgs[0].image_data_uint8)}, "
          f"channels={len(img)//(imgs[0].width*imgs[0].height)}")
else:
    print("WARNING: No camera image - check camera_name in settings")

# Test ground truth
state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"Position: [{pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f}]")

print("=== AirSim Test PASSED ===")
EOF
```

**Expected**: IMU readings with ~9.81 m/s^2 on Z axis, 640x480 image, position near origin.

### Test 2: ArduPilot SITL + AirSim Connection

**Goal**: Verify ArduPilot controls the drone through AirSim.

```bash
# 1. Start AirSim (if not running)
# 2. Start ArduPilot
cd ~/dev/ardupilot
sim_vehicle.py -v ArduCopter --model=airsim-copter --console --map

# 3. In MAVProxy console, wait for GPS lock then:
arm throttle
mode guided
takeoff 5
# Wait 10 seconds
mode land
```

**Expected**:
- Drone lifts off in AirSim window to ~5m altitude
- Hovers stably
- Lands when commanded
- MAVProxy shows no EKF errors

### Test 3: ROS2 Bridge Standalone

**Goal**: Verify bridge publishes correct ROS2 topics.

```bash
# Start bridge
source /opt/ros/humble/setup.bash
python3 /mnt/rnd/Shubham/openvins_ws/integration/airsim_openvins_bridge.py &
BRIDGE_PID=$!

# Wait for startup
sleep 3

# Test IMU topic
echo "=== IMU Topic ==="
timeout 3 ros2 topic hz /imu --window 20
ros2 topic echo /imu --once

# Test camera topic
echo "=== Camera Topic ==="
timeout 5 ros2 topic hz /camera/image --window 5
ros2 topic echo /camera/image --field header --once

# Test camera info
echo "=== Camera Info ==="
ros2 topic echo /camera/camera_info --once

# Cleanup
kill $BRIDGE_PID 2>/dev/null
```

**Expected**:
- IMU at ~200 Hz with valid angular_velocity and linear_acceleration
- Camera at ~20 Hz with mono8 encoding, 640x480
- Camera info with K matrix [320, 0, 320, 0, 320, 240, 0, 0, 1]

### Test 4: OpenVINS with Bridge Data

**Goal**: Verify OpenVINS initializes and produces pose estimates.

```bash
# 1. Start AirSim + ArduPilot (as above)
# 2. Start bridge
# 3. Start OpenVINS

source /opt/ros/humble/setup.bash
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash

ros2 run ov_msckf run_subscribe_msckf --ros-args \
    -p config_path:=/mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config/estimator_config.yaml \
    -p topic_imu:=/imu \
    -p topic_camera0:=/camera/image \
    -p verbosity:=DEBUG

# 4. In MAVProxy, takeoff and move:
#    arm throttle
#    mode guided
#    takeoff 3
#    velocity 1 0 0 5

# 5. Watch for "initialized successfully" in OpenVINS output
```

**Expected**: OpenVINS subscribes, receives data, initializes after motion, publishes odometry.

---

## Flight Maneuver Tests

### Test M1: Hover Stability

```
# MAVProxy commands
arm throttle
mode guided
takeoff 5
# Hover for 30 seconds

# Monitor VIO output
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position
```

**Expected**: VIO position should remain near constant during hover. Drift < 0.5m over 30s.

### Test M2: Square Trajectory

```
# MAVProxy commands
arm throttle
mode guided
takeoff 5

# Fly a 5m square
velocity 1 0 0 5    # Forward 5m
velocity 0 1 0 5    # Right 5m
velocity -1 0 0 5   # Backward 5m
velocity 0 -1 0 5   # Left 5m (back to start)

mode land
```

**Expected**:
- VIO path should show a recognizable square
- End position should be close to start position (within ~2m for sim)
- No VIO divergence or NaN values

### Test M3: Altitude Changes

```
arm throttle
mode guided
takeoff 3

velocity 0 0 -1 5   # Climb 5m (NED: negative Z = up)
velocity 0 0 1 5     # Descend 5m

mode land
```

**Expected**: VIO tracks altitude changes. Vertical position estimate follows actual altitude.

### Test M4: Fast Rotation (Yaw Stress Test)

```
arm throttle
mode guided
takeoff 5

# Yaw rotation (this stresses VIO feature tracking)
condition_yaw 90 45    # Yaw to 90 degrees at 45 deg/s
condition_yaw 180 45
condition_yaw 270 45
condition_yaw 360 45   # Full rotation back

mode land
```

**Expected**: VIO should maintain tracking through rotation. May show slight drift
but should not diverge. Watch for "not enough features" warnings.

---

## Performance Validation

### Data Rate Monitoring

```bash
# Run for 60 seconds and log rates
for i in $(seq 1 12); do
    echo "=== $(date) ==="
    timeout 5 ros2 topic hz /imu --window 50 2>&1 | grep "average rate"
    timeout 5 ros2 topic hz /camera/image --window 10 2>&1 | grep "average rate"
    timeout 5 ros2 topic hz /ov_msckf/odomimu --window 10 2>&1 | grep "average rate" 2>/dev/null
    echo ""
    sleep 5
done
```

### CPU Usage Monitoring

```bash
# Monitor CPU usage of key processes
watch -n 1 'ps aux | grep -E "(airsim|openvins|bridge|AirSimNH|arducopter)" | grep -v grep'
```

### Latency Measurement

```bash
# Compare timestamps between camera input and VIO output
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('latency_checker')

last_cam_stamp = None

def cam_cb(msg):
    global last_cam_stamp
    last_cam_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def odom_cb(msg):
    odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if last_cam_stamp:
        latency = (odom_stamp - last_cam_stamp) * 1000  # ms
        print(f"VIO latency: {latency:.1f} ms")

node.create_subscription(Image, '/camera/image', cam_cb, 10)
node.create_subscription(Odometry, '/ov_msckf/odomimu', odom_cb, 10)

rclpy.spin(node)
EOF
```

---

## Ground Truth Comparison

### Record VIO and Ground Truth

```bash
# Record rosbag with VIO output and ground truth
ros2 bag record /ov_msckf/odomimu /ov_msckf/pathimu /imu /camera/image \
    -o /tmp/vio_test_$(date +%Y%m%d_%H%M%S)

# Record AirSim ground truth in parallel
python3 << 'EOF'
import airsim
import time
import csv

client = airsim.MultirotorClient()
client.confirmConnection()

with open('/tmp/ground_truth.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp_ns', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])

    for i in range(6000):  # 60 seconds at 100Hz
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        ori = state.kinematics_estimated.orientation
        writer.writerow([
            state.timestamp,
            pos.x_val, pos.y_val, pos.z_val,
            ori.w_val, ori.x_val, ori.y_val, ori.z_val
        ])
        time.sleep(0.01)

print("Ground truth recorded to /tmp/ground_truth.csv")
EOF
```

### Evaluate Accuracy

```bash
# After recording, use OpenVINS evaluation tools
# Compare /tmp/ov_estimate.txt with /tmp/ground_truth.csv
# The ov_eval package provides tools for this:
ros2 run ov_eval error_dataset /tmp/ov_groundtruth.txt /tmp/ov_estimate.txt
```
