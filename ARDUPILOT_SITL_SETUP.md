# ArduPilot SITL Setup for AirSim Integration

## Overview

ArduPilot SITL has a built-in AirSim backend (`SIM_AirSim.cpp`) that communicates via UDP.
When started with `--model=airsim-copter`, ArduPilot:
1. Sends PWM servo commands to AirSim on port 9002 (binary `uint16_t[11]`)
2. Receives JSON sensor data from AirSim on port 9003 (IMU, GPS, pose, velocity)

## Prerequisites

```bash
# Ensure ArduPilot SITL is built
cd ~/dev/ardupilot
./waf configure --board sitl
./waf copter
```

## Starting ArduPilot SITL

### Basic Launch Command

```bash
cd ~/dev/ardupilot

# Start SITL with AirSim backend
sim_vehicle.py -v ArduCopter \
    --model=airsim-copter \
    --console \
    --map \
    -A "--serial0=udpclient:127.0.0.1:14550"
```

### Parameter Breakdown

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `-v` | `ArduCopter` | Vehicle type (copter) |
| `--model` | `airsim-copter` | Use AirSim SITL backend |
| `--console` | - | Open MAVProxy console |
| `--map` | - | Open MAVProxy map display |
| `-A` | `--serial0=...` | Additional arguments to ArduPilot binary |

### Connection Ports (ArduPilot Side)

When using `--model=airsim-copter`, ArduPilot's `SIM_AirSim.cpp` sets up:

```
Control Output: UDP sendto -> 127.0.0.1:9002  (PWM commands TO AirSim)
Sensor Input:   UDP bind   -> 0.0.0.0:9003    (sensor data FROM AirSim)
```

These match the AirSim settings:
```json
"UdpPort": 9003,      // AirSim sends sensor data TO this port
"ControlPort": 9002   // AirSim receives PWM commands ON this port
```

## Motor Command Interface

### Servo Packet Format (ArduPilot -> AirSim)

```c
// From SIM_AirSim.h
static const int kArduCopterRotorControlCount = 11;
struct servo_packet {
    uint16_t pwm[11];  // PWM values 1000-2000 for each motor
};
```

- **Motors 0-3**: Quad motor outputs (typical mapping)
- **Motors 4-10**: Auxiliary channels
- PWM range: 1000 (idle) to 2000 (full throttle)
- Sent every physics step (~400 Hz)

### AirSim Normalization
AirSim normalizes PWM to 0.0-1.0: `(pwm - 1000) / 1000.0`

## Vehicle State Feedback (AirSim -> ArduPilot)

### JSON Sensor Data Format

AirSim sends this JSON packet via UDP to ArduPilot every physics step:

```json
{
  "timestamp": 1234567890,
  "imu": {
    "angular_velocity": [gx, gy, gz],
    "linear_acceleration": [ax, ay, az]
  },
  "gps": {
    "lat": -35.363261,
    "lon": 149.165230,
    "alt": 583.0
  },
  "pose": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "velocity": {
    "world_linear_velocity": [vx, vy, vz]
  },
  "lidar": {
    "point_cloud": [x0,y0,z0, x1,y1,z1, ...]
  },
  "rc": {
    "channels": [ch1, ch2, ch3, ...]
  },
  "rng": {
    "distances": [d1, d2, ...]
  }
}
```

## AirSim Settings for ArduCopter

The settings file is at `~/Documents/AirSim/settings.json` (user home) or can be specified per-environment.

### Optimized settings.json for VIO Integration

Place this at `~/Documents/AirSim/settings.json`:

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

### Key Settings Explanation

| Setting | Value | Why |
|---------|-------|-----|
| `VehicleType` | `ArduCopter` | Enables ArduPilot SITL backend in AirSim |
| `UseSerial` | `false` | Use UDP instead of serial port |
| `UdpPort` | `9003` | Port AirSim sends sensor data to ArduPilot |
| `ControlPort` | `9002` | Port AirSim receives motor commands from ArduPilot |
| `ClockSpeed` | `1.0` | Real-time (reduce if system can't keep up) |
| Camera `FOV_Degrees` | `90` | Must match OpenVINS camera intrinsics |
| Camera position | `X:0.50, Z:0.10` | Front-mounted, slightly above center |

## Startup Sequence

**Order matters! Start AirSim FIRST, then ArduPilot.**

```bash
# Terminal 1: Start AirSim environment
cd ~/AirSim/AirSimNH/LinuxNoEditor
./AirSimNH.sh -ResX=800 -ResY=600 -windowed

# Terminal 2: Wait for AirSim to fully load, then start ArduPilot SITL
cd ~/dev/ardupilot
sim_vehicle.py -v ArduCopter --model=airsim-copter --console --map

# Terminal 3: Start the ROS2 bridge (after both are connected)
# (see ROS2_BRIDGE_NODE.py)
```

## Verifying Connection

After starting both AirSim and ArduPilot SITL:

1. **ArduPilot console output** should show:
   ```
   Starting SITL Airsim type 1
   Bind SITL sensor input at 0.0.0.0:9003
   AirSim control interface set to 127.0.0.1:9002
   FPS avg=XXX.XX
   ```

2. **No "No sensor message" errors** means data is flowing

3. **MAVProxy** (console window) should show:
   ```
   APM: ArduCopter V4.x.x
   APM: EKF3 IMU0 is using GPS
   ```

## ArduPilot Parameters for AirSim

Load these parameters after connecting (in MAVProxy console):

```
# In MAVProxy console:
param set ARMING_CHECK 0          # Disable arming checks for sim
param set SIM_SPEEDUP 1           # Real-time speed
param set EK3_SRC1_POSXY 3        # GPS for horizontal position
param set EK3_SRC1_VELXY 3        # GPS for horizontal velocity
param set EK3_SRC1_POSZ 1         # Baro for vertical position
param set EK3_SRC1_VELZ 0         # None for vertical velocity
param set EK3_SRC1_YAW 1          # Compass for yaw
```

### Optional: External Navigation (VIO feedback to ArduPilot)

If you want to feed OpenVINS pose estimates back to ArduPilot EKF:

```
param set EK3_SRC2_POSXY 6        # ExternalNav for position
param set EK3_SRC2_VELXY 6        # ExternalNav for velocity
param set EK3_SRC2_POSZ 6         # ExternalNav for height
param set EK3_SRC2_YAW 6          # ExternalNav for yaw
param set VISO_TYPE 1              # Enable visual odometry input
param set VISO_DELAY_MS 50        # Estimated VIO pipeline delay
```

## Basic Flight Test (MAVProxy)

```
# Arm and takeoff
arm throttle
mode guided
takeoff 5

# Move forward 5 meters
velocity 1 0 0 5

# Hover and land
mode land
```
