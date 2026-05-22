# Complete Implementation Plan: ArduPilot SITL + AirSim + OpenVINS

## Phase 0: Prerequisites Verification

### Step 0.1: Verify Software Installations

```bash
# Check ROS2 Humble
source /opt/ros/humble/setup.bash
ros2 --version
# Expected: ros2 0.x.x

# Check OpenVINS build
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash
ros2 pkg list | grep ov_msckf
# Expected: ov_msckf

# Check AirSim Python API
python3 -c "import airsim; print(f'airsim {airsim.__version__}')"
# Expected: airsim 1.8.1

# Check ArduPilot SITL build
ls ~/dev/ardupilot/build/sitl/bin/arducopter
# Expected: file exists
```

### Step 0.2: Verify GPU (Required for AirSim)

```bash
nvidia-smi
# Expected: Shows GPU info and driver version
```

---

## Phase 1: AirSim Configuration

### Step 1.1: Deploy AirSim Settings

```bash
# Backup existing settings
cp ~/Documents/AirSim/settings.json ~/Documents/AirSim/settings.json.bak 2>/dev/null

# Copy optimized VIO settings
cat > ~/Documents/AirSim/settings.json << 'EOF'
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "SpringArmChase",
  "ClockSpeed": 1.0,
  "OriginGeopoint": {
    "Latitude": -35.363261,
    "Longitude": 149.165230,
    "Altitude": 583
  },
  "Vehicles": {
    "Copter": {
      "VehicleType": "ArduCopter",
      "UseSerial": false,
      "LocalHostIp": "127.0.0.1",
      "UdpIp": "127.0.0.1",
      "UdpPort": 9003,
      "ControlPort": 9002,
      "DefaultVehicleState": "Armed",
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "Sensors": {
        "Barometer": { "SensorType": 1, "Enabled": true },
        "Imu":       { "SensorType": 2, "Enabled": true },
        "Gps":       { "SensorType": 3, "Enabled": true },
        "Magnetometer": { "SensorType": 4, "Enabled": true }
      },
      "Cameras": {
        "front_center": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.50, "Y": 0.00, "Z": 0.10,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
EOF
```

**Expected Output**: File created at `~/Documents/AirSim/settings.json`

### Step 1.2: Deploy OpenVINS Config for AirSim

```bash
# Copy config to OpenVINS config directory
cp -r /mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config \
      /mnt/rnd/Shubham/openvins_ws/open_vins/config/airsim_vio

# Also copy to the installed config directory
cp -r /mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config/* \
      /mnt/rnd/Shubham/openvins_ws/install/ov_msckf/share/ov_msckf/config/airsim_vio/ 2>/dev/null || \
mkdir -p /mnt/rnd/Shubham/openvins_ws/install/ov_msckf/share/ov_msckf/config/airsim_vio && \
cp -r /mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config/* \
      /mnt/rnd/Shubham/openvins_ws/install/ov_msckf/share/ov_msckf/config/airsim_vio/
```

**Expected Output**: Config files copied to both source and install directories

---

## Phase 2: Start AirSim

### Step 2.1: Launch AirSim Environment

```bash
# Terminal 1: AirSim
cd ~/AirSim/AirSimNH/LinuxNoEditor
./AirSimNH.sh -ResX=800 -ResY=600 -windowed
```

**Expected Output**:
- Unreal Engine window opens showing the AirSim environment
- Console shows "Waiting for connection..." or similar
- A drone should be visible in the environment

### Step 2.2: Verify AirSim API

```bash
# Terminal 2: Quick API test (in a separate terminal)
python3 -c "
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
print('Connected!')
imu = client.getImuData()
print(f'IMU timestamp: {imu.time_stamp}')
print(f'IMU accel: [{imu.linear_acceleration.x_val:.3f}, {imu.linear_acceleration.y_val:.3f}, {imu.linear_acceleration.z_val:.3f}]')
imgs = client.simGetImages([airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, False)])
if imgs and len(imgs) > 0:
    print(f'Image: {imgs[0].width}x{imgs[0].height}, ts={imgs[0].time_stamp}')
else:
    print('WARNING: No image returned - check camera name in settings.json')
"
```

**Expected Output**:
```
Connected!
IMU timestamp: <some large number>
IMU accel: [0.000, 0.000, -9.810]  (approximately, drone at rest)
Image: 640x480, ts=<some large number>
```

**Troubleshooting**: If image is empty, the camera name in settings.json may not match.
Try `"front_center_custom"` instead of `"front_center"`.

---

## Phase 3: Start ArduPilot SITL

### Step 3.1: Launch ArduPilot with AirSim Backend

```bash
# Terminal 2: ArduPilot SITL
cd ~/dev/ardupilot
sim_vehicle.py -v ArduCopter --model=airsim-copter --console --map
```

**Expected Output**:
```
Starting SITL Airsim type 1
Bind SITL sensor input at 0.0.0.0:9003
AirSim control interface set to 127.0.0.1:9002
FPS avg=XXX.XX
...
APM: ArduCopter V4.x.x
APM: EKF3 IMU0 is using GPS
```

### Step 3.2: Verify ArduPilot-AirSim Connection

In the MAVProxy console (opened by sim_vehicle.py):
```
# Check vehicle status
status

# Check GPS fix
gps

# Check battery (simulated)
battery
```

**Expected Output**: GPS shows valid position, EKF healthy

---

## Phase 4: Start ROS2 Bridge

### Step 4.1: Launch the Bridge Node

```bash
# Terminal 3: ROS2 Bridge
source /opt/ros/humble/setup.bash
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash

python3 /mnt/rnd/Shubham/openvins_ws/integration/airsim_openvins_bridge.py
```

**Expected Output**:
```
[INFO] [airsim_openvins_bridge]: Connected to AirSim at 127.0.0.1
[INFO] [airsim_openvins_bridge]: AirSim-OpenVINS bridge started: IMU@200.0Hz, Camera@20.0Hz, Image:640x480, FOV:90.0deg
[INFO] [airsim_openvins_bridge]: Rates - IMU: ~200.0 Hz, Camera: ~20.0 Hz
```

### Step 4.2: Verify ROS2 Topics

```bash
# Terminal 4: Check topics
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list
# Expected:
#   /imu
#   /camera/image
#   /camera/camera_info

# Check IMU rate
ros2 topic hz /imu
# Expected: average rate: ~200 Hz

# Check camera rate
ros2 topic hz /camera/image
# Expected: average rate: ~20 Hz

# Inspect an IMU message
ros2 topic echo /imu --once

# Check image info
ros2 topic info /camera/image
# Expected: Type: sensor_msgs/msg/Image, Publisher count: 1
```

---

## Phase 5: Start OpenVINS

### Step 5.1: Launch OpenVINS

```bash
# Terminal 5: OpenVINS
source /opt/ros/humble/setup.bash
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash

ros2 run ov_msckf run_subscribe_msckf \
    --ros-args \
    -p verbosity:=INFO \
    -p use_stereo:=false \
    -p max_cameras:=1 \
    -p save_total_state:=true \
    -p config_path:=/mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config/estimator_config.yaml \
    -p topic_imu:=/imu \
    -p topic_camera0:=/camera/image
```

**Expected Output**:
```
[ov_msckf]: subscribing to IMU: /imu
[ov_msckf]: subscribing to cam (mono): /camera/image
...
(after initialization period with motion)
[ov_msckf]: initialized successfully!
[ov_msckf]: state estimate: pos=[x,y,z] vel=[vx,vy,vz]
```

### Step 5.2: Alternative - Use Launch File

```bash
# Terminal 3-5 combined: Use launch file
source /opt/ros/humble/setup.bash
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash

ros2 launch /mnt/rnd/Shubham/openvins_ws/integration/integration_launch.py
```

---

## Phase 6: Initialize VIO

### Step 6.1: Move the Drone to Trigger Initialization

OpenVINS requires motion to initialize. In the MAVProxy console:

```
# Arm and takeoff
arm throttle
mode guided
takeoff 3

# Wait for altitude...
# Then make small movements to generate visual features
velocity 0.5 0 0 2    # forward 0.5 m/s for 2 seconds
velocity 0 0.5 0 2    # right 0.5 m/s for 2 seconds
velocity -0.5 0 0 2   # backward
velocity 0 -0.5 0 2   # left (back to start)
```

**Expected Output** (OpenVINS terminal):
```
[ov_msckf]: initialized successfully!
[ov_msckf]: ZUPT: velocity [0.xxx, 0.xxx, 0.xxx]
```

### Step 6.2: Verify VIO Output

```bash
# Terminal 6: Check OpenVINS output topics
ros2 topic list | grep ov_msckf
# Expected:
#   /ov_msckf/poseimu
#   /ov_msckf/odomimu
#   /ov_msckf/pathimu
#   /ov_msckf/points_msckf
#   /ov_msckf/trackhist

ros2 topic hz /ov_msckf/odomimu
# Expected: ~20 Hz (camera rate)

ros2 topic echo /ov_msckf/odomimu --once
# Expected: Position and velocity estimates
```

---

## Phase 7: Visualization

### Step 7.1: RViz2

```bash
# If not started by launch file
rviz2
```

In RViz2:
1. Set Fixed Frame to `global`
2. Add display: Path -> topic `/ov_msckf/pathimu`
3. Add display: PointCloud2 -> topic `/ov_msckf/points_msckf`
4. Add display: Image -> topic `/ov_msckf/trackhist`

### Step 7.2: Monitor Feature Tracking

```bash
# View tracked features image
ros2 run rqt_image_view rqt_image_view /ov_msckf/trackhist
```

---

## Complete Terminal Layout

```
+-------------------------------+-------------------------------+
| Terminal 1: AirSim            | Terminal 2: ArduPilot SITL    |
| cd ~/AirSim/AirSimNH/...     | cd ~/dev/ardupilot            |
| ./AirSimNH.sh                 | sim_vehicle.py -v ArduCopter  |
|                               |   --model=airsim-copter       |
|                               |   --console --map             |
+-------------------------------+-------------------------------+
| Terminal 3: ROS2 Bridge       | Terminal 4: OpenVINS          |
| source ... setup.bash         | source ... setup.bash         |
| python3 airsim_openvins_      | ros2 run ov_msckf             |
|   bridge.py                   |   run_subscribe_msckf ...     |
+-------------------------------+-------------------------------+
| Terminal 5: Monitoring        | Terminal 6: RViz2             |
| ros2 topic hz /imu            | rviz2                         |
| ros2 topic hz /camera/image   |                               |
| ros2 topic echo ...           |                               |
+-------------------------------+-------------------------------+
```

---

## Quick Start (All Commands)

```bash
# === Terminal 1: AirSim ===
cd ~/AirSim/AirSimNH/LinuxNoEditor && ./AirSimNH.sh -ResX=800 -ResY=600 -windowed

# === Terminal 2: ArduPilot SITL (wait for AirSim to load) ===
cd ~/dev/ardupilot && sim_vehicle.py -v ArduCopter --model=airsim-copter --console --map

# === Terminal 3: Full Integration Launch (wait for ArduPilot connected) ===
source /opt/ros/humble/setup.bash && \
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash && \
ros2 launch /mnt/rnd/Shubham/openvins_ws/integration/integration_launch.py

# === Terminal 2 (MAVProxy): Trigger VIO initialization ===
arm throttle
mode guided
takeoff 3
# wait 5 seconds
velocity 0.5 0 0 3
velocity 0 0.5 0 3
velocity -0.5 0 0 3
velocity 0 -0.5 0 3
```
