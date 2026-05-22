# AirSim Sensor Configuration for VIO

## Current Sensor Setup (from settings.json)

The existing `arducopter-settings.json` at `~/AirSim/AirSimNH/LinuxNoEditor/` configures:

### Sensors
| Sensor | Type ID | Status | Notes |
|--------|---------|--------|-------|
| Barometer | 1 | Enabled | For ArduPilot altitude |
| IMU | 2 | Enabled | Required for VIO |
| GPS | 3 | Enabled | For ArduPilot EKF |
| Magnetometer | 4 | Enabled | For ArduPilot heading |
| LiDAR (front) | 6 | Enabled | Not needed for VIO (can disable) |

### Cameras
| Camera | Resolution | FOV | Position (m) | Use |
|--------|-----------|-----|-------------|-----|
| front_center_custom | 640x480 | 90deg | X:0.50, Y:0.00, Z:0.10 | **VIO primary** |
| front_left_custom | 672x376 | 90deg | X:0.50, Y:-0.06, Z:0.10 | Stereo pair (optional) |
| front_right_custom | 672x376 | 90deg | X:0.50, Y:+0.06, Z:0.10 | Stereo pair (optional) |

## Optimized Settings for VIO

### settings.json (place at ~/Documents/AirSim/settings.json)

```json
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
```

### Optimization Rationale

1. **Single camera (mono)** - Simpler, lower latency, proven with OpenVINS
2. **640x480 resolution** - Matches existing OpenVINS config, good feature density
3. **90 degree FOV** - Wide enough for feature tracking during maneuvers
4. **No LiDAR** - Reduces AirSim computational load, not used by OpenVINS
5. **ImageType 0 (Scene)** - Standard RGB, converted to grayscale in bridge

## Camera Intrinsics Calculation

For AirSim pinhole camera with known FOV and resolution:

```
Width  = 640 pixels
Height = 480 pixels
FOV    = 90 degrees (horizontal)

Focal length (pixels):
  fx = Width / (2 * tan(FOV/2))
  fx = 640 / (2 * tan(45deg))
  fx = 640 / (2 * 1.0)
  fx = 320.0

  fy = fx = 320.0  (square pixels in AirSim)

Principal point (image center):
  cx = Width / 2  = 320.0
  cy = Height / 2 = 240.0
```

### Camera Intrinsic Matrix K

```
K = | 320.0   0.0   320.0 |
    |   0.0  320.0  240.0 |
    |   0.0   0.0    1.0  |
```

### Distortion
AirSim renders perfect pinhole images: **zero distortion**
```
Distortion coefficients (radtan): [0.0, 0.0, 0.0, 0.0]
```

## IMU Configuration

### AirSim IMU Characteristics

AirSim's simulated IMU provides:

| Property | Value | Notes |
|----------|-------|-------|
| Angular velocity | 3-axis (rad/s) | Body frame |
| Linear acceleration | 3-axis (m/s^2) | Body frame, includes gravity |
| Orientation | Quaternion (w,x,y,z) | Available but not used by OpenVINS |
| Internal rate | ~1000 Hz | AirSim physics rate |
| Polling rate | 200 Hz | Bridge node sampling rate |

### IMU Noise Parameters for OpenVINS

AirSim's simulated IMU has low noise compared to real hardware.
These values are tuned for simulation (from existing `kalibr_imu_chain.yaml`):

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `accelerometer_noise_density` | 1.0e-2 | m/s^2/sqrt(Hz) | Accel white noise |
| `accelerometer_random_walk` | 1.0e-2 | m/s^3/sqrt(Hz) | Accel bias diffusion |
| `gyroscope_noise_density` | 1.0e-3 | rad/s/sqrt(Hz) | Gyro white noise |
| `gyroscope_random_walk` | 1.0e-4 | rad/s^2/sqrt(Hz) | Gyro bias diffusion |

These are intentionally set higher than AirSim's actual noise to give OpenVINS
margin for the estimation. If tracking is too noisy, reduce these values.

## AirSim Python API for Sensor Access

### IMU Data

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()

# Get IMU data
imu_data = client.getImuData()
# imu_data.angular_velocity    -> Vector3r (x, y, z) rad/s
# imu_data.linear_acceleration -> Vector3r (x, y, z) m/s^2
# imu_data.orientation         -> Quaternionr (w, x, y, z)
# imu_data.time_stamp          -> int (nanoseconds)
```

### Camera Images

```python
# Request RGB image (Scene type = 0)
responses = client.simGetImages([
    airsim.ImageRequest(
        camera_name="front_center",       # Camera name from settings
        image_type=airsim.ImageType.Scene, # Type 0 = RGB
        pixels_as_float=False,             # uint8 pixels
        compress=False                     # Uncompressed for speed
    )
])

response = responses[0]
# response.image_data_uint8  -> bytes (raw pixel data)
# response.width             -> 640
# response.height            -> 480
# response.time_stamp        -> int (nanoseconds)

import numpy as np
img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
img = img.reshape(response.height, response.width, 3)  # BGR format
```

### Ground Truth Pose (for validation)

```python
# Get true vehicle state (for validating VIO output)
state = client.getMultirotorState()
# state.kinematics_estimated.position    -> Vector3r (NED meters)
# state.kinematics_estimated.orientation -> Quaternionr
# state.kinematics_estimated.linear_velocity -> Vector3r
# state.kinematics_estimated.angular_velocity -> Vector3r
# state.timestamp -> int (nanoseconds)
```

## Camera-IMU Extrinsic Transform

In simulation, the IMU is at the vehicle body center and the camera is offset:

```
Camera position relative to IMU (body frame):
  p_CinI = [0.50, 0.00, -0.10]  (forward, no lateral, up)

  Note: AirSim Z is down (NED), but camera Z=0.10 in settings means
  0.10m above vehicle center, which is -0.10 in NED convention.

Camera rotation relative to IMU:
  R_ItoC = Identity (camera looks forward, aligned with body X axis)

T_imu_cam (4x4 homogeneous):
  | 1  0  0  0.50 |
  | 0  1  0  0.00 |
  | 0  0  1 -0.10 |
  | 0  0  0  1.00 |
```

This is configured in `kalibr_imucam_chain.yaml` for OpenVINS.
